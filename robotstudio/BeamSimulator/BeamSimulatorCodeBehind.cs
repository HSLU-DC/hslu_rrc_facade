// BeamSimulatorCodeBehind.cs
// SmartComponent code-behind for dynamic beam geometry visualization.
//
// Lifecycle per element:
//   Activate -> raw STL loaded, attached to gripper flange
//   SwapCutA -> raw replaced with cut-A STL
//   SwapCutB -> cut-A replaced with finished STL
//   Release  -> beam detached from gripper, stays at world position
//   Reset    -> all placed beams deleted, state cleared
//
// Placed beams accumulate in the station, building up the facade.

using System;
using System.Collections.Generic;
using System.IO;
using ABB.Robotics.Math;
using ABB.Robotics.RobotStudio;
using ABB.Robotics.RobotStudio.Stations;

namespace BeamSimulator
{
    public class BeamSimulatorCodeBehind : SmartComponentCodeBehind
    {
        // StateCache keys
        private const string KEY_CURRENT_PART = "CurrentPart";
        private const string KEY_CURRENT_ELEMENT = "CurrentElementID";
        private const string KEY_CURRENT_LAYER = "CurrentLayerID";
        private const string KEY_CURRENT_STAGE = "CurrentStage";
        private const string KEY_PLACED_BEAMS = "PlacedBeams";
        private const string KEY_TCP_OFFSET = "CachedTcpOffset";

        // ============================================================
        // Initialization
        // ============================================================

        public override void OnInitialize(SmartComponent component)
        {
            base.OnInitialize(component);
            InitializeStateCache(component);
        }

        public override void OnSimulationStart(SmartComponent component)
        {
            base.OnSimulationStart(component);
            InitializeStateCache(component);

            // Pre-cache the TCP offset before RAPID starts running,
            // to avoid deadlocks when looking up tool data during signal handling.
            try
            {
                Matrix4 offset = LookupToolOffset(component);
                component.StateCache[KEY_TCP_OFFSET] = offset;
            }
            catch { }
        }

        private void InitializeStateCache(SmartComponent component)
        {
            if (!component.StateCache.ContainsKey(KEY_CURRENT_PART))
                component.StateCache[KEY_CURRENT_PART] = null;
            if (!component.StateCache.ContainsKey(KEY_CURRENT_ELEMENT))
                component.StateCache[KEY_CURRENT_ELEMENT] = -1;
            if (!component.StateCache.ContainsKey(KEY_CURRENT_LAYER))
                component.StateCache[KEY_CURRENT_LAYER] = -1;
            if (!component.StateCache.ContainsKey(KEY_CURRENT_STAGE))
                component.StateCache[KEY_CURRENT_STAGE] = "idle";
            if (!component.StateCache.ContainsKey(KEY_PLACED_BEAMS))
                component.StateCache[KEY_PLACED_BEAMS] = new List<Part>();
        }

        // ============================================================
        // Property Changes
        // ============================================================

        public override void OnPropertyValueChanged(
            SmartComponent component,
            DynamicProperty changedProperty,
            object oldValue)
        {
            if (changedProperty.Name == "GeometryFolder")
            {
                string folder = changedProperty.Value as string;
                if (!string.IsNullOrEmpty(folder) && !Directory.Exists(folder))
                {
                    Logger.AddMessage(
                        new LogMessage($"BeamSimulator: GeometryFolder not found: {folder}",
                        LogMessageSeverity.Warning));
                }
            }
            else if (changedProperty.Name == "ToolName")
            {
                // Invalidate cached TCP offset so it gets re-looked-up
                if (component.StateCache.ContainsKey(KEY_TCP_OFFSET))
                    component.StateCache.Remove(KEY_TCP_OFFSET);
            }
        }

        // ============================================================
        // Signal Handling (main dispatcher)
        // ============================================================

        public override void OnIOSignalValueChanged(
            SmartComponent component,
            IOSignal signal)
        {
            InitializeStateCache(component);

            // Only react to rising edges (0 -> 1)
            int value;
            try { value = Convert.ToInt32(signal.Value); }
            catch { return; }

            if (value == 0)
                return;

            switch (signal.Name)
            {
                case "Activate":
                    HandleActivate(component);
                    break;
                case "SwapCutA":
                    HandleSwapCut(component, "cutA");
                    break;
                case "SwapCutB":
                    HandleSwapCut(component, "cutB");
                    break;
                case "Release":
                    HandleRelease(component);
                    break;
                case "Reset":
                    HandleReset(component);
                    break;
            }
        }

        // ============================================================
        // Activate: Load raw beam, attach to gripper
        // ============================================================

        private void HandleActivate(SmartComponent component)
        {
            try
            {
                Logger.AddMessage(new LogMessage("BeamSimulator: Activate triggered"));

                // Read element/layer from signals (handle different value types)
                int elementId = Convert.ToInt32(component.IOSignals["ElementID"].Value);
                int layerId = Convert.ToInt32(component.IOSignals["LayerID"].Value);
                Logger.AddMessage(new LogMessage(
                    $"BeamSimulator: ElementID={elementId}, LayerID={layerId}"));

                component.StateCache[KEY_CURRENT_ELEMENT] = elementId;
                component.StateCache[KEY_CURRENT_LAYER] = layerId;

                // Build filename
                string folder = component.Properties["GeometryFolder"].Value as string;
                Logger.AddMessage(new LogMessage(
                    $"BeamSimulator: GeometryFolder='{folder}'"));

                if (string.IsNullOrEmpty(folder))
                {
                    LogWarning("GeometryFolder not set");
                    PulseReady(component);
                    return;
                }

                string fileName = $"L{layerId}_E{elementId}_raw.stl";
                string filePath = Path.Combine(folder, fileName);
                Logger.AddMessage(new LogMessage(
                    $"BeamSimulator: Loading {filePath} (exists={File.Exists(filePath)})"));

                // Load and attach
                Part part = LoadStl(filePath);
                if (part != null)
                {
                    Logger.AddMessage(new LogMessage(
                        $"BeamSimulator: Part loaded, attaching to gripper..."));
                    AttachToGripper(component, part);
                    component.StateCache[KEY_CURRENT_PART] = part;
                    component.StateCache[KEY_CURRENT_STAGE] = "raw";
                    Logger.AddMessage(new LogMessage(
                        $"BeamSimulator: Activated L{layerId}_E{elementId} (raw)"));
                }
                else
                {
                    LogWarning($"Failed to load: {fileName}");
                }

                PulseReady(component);
            }
            catch (Exception ex)
            {
                LogWarning($"Activate exception: {ex.Message}");
            }
        }

        // ============================================================
        // SwapCut: Replace current geometry with cut variant
        // ============================================================

        private void HandleSwapCut(SmartComponent component, string stage)
        {
            int elementId = (int)component.StateCache[KEY_CURRENT_ELEMENT];
            int layerId = (int)component.StateCache[KEY_CURRENT_LAYER];

            if (elementId < 0)
            {
                LogWarning($"SwapCut({stage}): No active element");
                PulseReady(component);
                return;
            }

            Part oldPart = component.StateCache[KEY_CURRENT_PART] as Part;
            Mechanism mech = FindMechanism(component);

            // Delete old part completely
            if (oldPart != null)
            {
                try { mech?.GetFlange(0).Detach(oldPart); } catch { }
                try { Station.ActiveStation?.GraphicComponents.Remove(oldPart); } catch { }
                try { oldPart.Delete(); } catch { }
                component.StateCache[KEY_CURRENT_PART] = null;
                Logger.AddMessage(new LogMessage("BeamSimulator: Old part removed"));
            }

            // Load new STL and attach at TCP
            string folder = component.Properties["GeometryFolder"].Value as string;
            string fileName = $"L{layerId}_E{elementId}_{stage}.stl";
            string filePath = Path.Combine(folder, fileName);

            Part newPart = LoadStl(filePath);
            if (newPart != null && mech != null)
            {
                Matrix4 tcpOffset = GetToolOffset(component);
                mech.AttachToFlange(newPart, true, tcpOffset);

                component.StateCache[KEY_CURRENT_PART] = newPart;
                component.StateCache[KEY_CURRENT_STAGE] = stage;
                Logger.AddMessage(new LogMessage(
                    $"BeamSimulator: Swapped to {stage} (L{layerId}_E{elementId})"));
            }
            else
            {
                LogWarning($"Failed to load: {fileName}");
                component.StateCache[KEY_CURRENT_PART] = null;
            }

            PulseReady(component);
        }

        // ============================================================
        // Release: Detach beam, stays at world position
        // ============================================================

        private void HandleRelease(SmartComponent component)
        {
            Part currentPart = component.StateCache[KEY_CURRENT_PART] as Part;
            Mechanism mech = FindMechanism(component);

            if (currentPart != null && mech != null)
            {
                // Save world position BEFORE detach
                Matrix4 worldPosition = currentPart.Transform.GlobalMatrix;

                // Detach from flange
                try { mech.GetFlange(0).Detach(currentPart); } catch { }

                // Restore world position (Detach may reset it)
                try { currentPart.Transform.GlobalMatrix = worldPosition; } catch { }

                // Track as placed beam
                var placedBeams = component.StateCache[KEY_PLACED_BEAMS] as List<Part>;
                placedBeams?.Add(currentPart);

                int elementId = (int)component.StateCache[KEY_CURRENT_ELEMENT];
                int layerId = (int)component.StateCache[KEY_CURRENT_LAYER];
                Logger.AddMessage(new LogMessage(
                    $"BeamSimulator: Released L{layerId}_E{elementId} " +
                    $"(total placed: {placedBeams?.Count ?? 0})"));
            }

            component.StateCache[KEY_CURRENT_PART] = null;
            component.StateCache[KEY_CURRENT_STAGE] = "idle";

            PulseReady(component);
        }

        // ============================================================
        // Reset: Delete all placed beams, clear state
        // ============================================================

        private void HandleReset(SmartComponent component)
        {
            try
            {
                Logger.AddMessage(new LogMessage("BeamSimulator: Reset triggered"));

                // Delete current part if any
                Part currentPart = component.StateCache[KEY_CURRENT_PART] as Part;
                Mechanism mech = FindMechanism(component);
                if (currentPart != null)
                {
                    DetachAndDelete(mech, currentPart);
                    Logger.AddMessage(new LogMessage("BeamSimulator: Deleted current part"));
                }

                // Delete all placed beams
                var placedBeams = component.StateCache[KEY_PLACED_BEAMS] as List<Part>;
                int count = 0;
                if (placedBeams != null)
                {
                    count = placedBeams.Count;
                    for (int i = placedBeams.Count - 1; i >= 0; i--)
                    {
                        try
                        {
                            Station station = Station.ActiveStation;
                            station?.GraphicComponents.Remove(placedBeams[i]);
                            placedBeams[i].Delete();
                        }
                        catch { }
                    }
                    placedBeams.Clear();
                }

                // Reset state
                component.StateCache[KEY_CURRENT_PART] = null;
                component.StateCache[KEY_CURRENT_ELEMENT] = -1;
                component.StateCache[KEY_CURRENT_LAYER] = -1;
                component.StateCache[KEY_CURRENT_STAGE] = "idle";

                Logger.AddMessage(new LogMessage(
                    $"BeamSimulator: Reset complete - deleted {count} placed beams"));
                PulseReady(component);
            }
            catch (Exception ex)
            {
                LogWarning($"Reset exception: {ex.Message}");
            }
        }

        // ============================================================
        // Helpers
        // ============================================================

        /// <summary>
        /// Load an STL file as a Part in the station.
        /// </summary>
        private Part LoadStl(string filePath)
        {
            if (!File.Exists(filePath))
            {
                LogWarning($"File not found: {filePath}");
                return null;
            }

            try
            {
                Part part = Part.Load(filePath);
                if (part != null)
                {
                    part.Name = Path.GetFileNameWithoutExtension(filePath);
                    Station station = Station.ActiveStation;
                    station?.GraphicComponents.Add(part);
                }
                return part;
            }
            catch (Exception ex)
            {
                LogWarning($"Failed to load STL: {ex.Message}");
                return null;
            }
        }

        /// <summary>
        /// Attach a Part to the gripper flange with TCP offset.
        /// STL origin = grip center = TCP, so we offset by the tool frame.
        /// </summary>
        private void AttachToGripper(SmartComponent component, Part part)
        {
            Mechanism mech = FindMechanism(component);
            if (mech == null)
            {
                LogWarning("No mechanism found");
                return;
            }

            try
            {
                // Use 3-parameter overload: mount + offset in one call
                // This mounts at flange with the tool offset applied as local transform
                Matrix4 tcpOffset = GetToolOffset(component);
                mech.AttachToFlange(part, true, tcpOffset);

                Logger.AddMessage(new LogMessage("BeamSimulator: Attached at TCP"));
            }
            catch (Exception ex)
            {
                LogWarning($"Attach failed: {ex.Message}");
            }
        }

        /// <summary>
        /// Get the tool TCP offset matrix (flange -> TCP).
        /// Cached to avoid deadlocks when querying the controller during signal handling.
        /// </summary>
        private Matrix4 GetToolOffset(SmartComponent component)
        {
            // Return cached value if available
            if (component.StateCache.ContainsKey(KEY_TCP_OFFSET))
            {
                object cached = component.StateCache[KEY_TCP_OFFSET];
                if (cached is Matrix4 m)
                    return m;
            }

            // First-time lookup (only happens once per session)
            Matrix4 offset = LookupToolOffset(component);
            component.StateCache[KEY_TCP_OFFSET] = offset;
            return offset;
        }

        /// <summary>
        /// Actually look up the tool offset from the station. Called only once.
        /// </summary>
        private Matrix4 LookupToolOffset(SmartComponent component)
        {
            try
            {
                string toolName = component.Properties["ToolName"].Value as string;
                Station station = Station.ActiveStation;

                if (station != null && !string.IsNullOrEmpty(toolName))
                {
                    RsTask task = station.ActiveTask;
                    if (task != null)
                    {
                        RsDataDeclaration[] tools = task.FindDataDeclarationsByType(typeof(RsToolData));
                        foreach (RsDataDeclaration decl in tools)
                        {
                            if (decl is RsToolData tool && tool.Name == toolName)
                            {
                                Matrix4 offset = tool.Frame.Matrix;
                                Logger.AddMessage(new LogMessage(
                                    $"BeamSimulator: Cached tool '{toolName}' offset: " +
                                    $"X={tool.Frame.X:F1} Y={tool.Frame.Y:F1} Z={tool.Frame.Z:F1}"));
                                return offset;
                            }
                        }
                    }
                    LogWarning($"Tool '{toolName}' not found - using identity");
                }
            }
            catch (Exception ex)
            {
                LogWarning($"LookupToolOffset: {ex.Message}");
            }

            return Matrix4.Identity;
        }

        /// <summary>
        /// Find the robot mechanism in the station.
        /// </summary>
        private Mechanism FindMechanism(SmartComponent component)
        {
            string mechName = component.Properties["MechanismName"].Value as string;
            Station station = Station.ActiveStation;

            if (station == null)
                return null;

            foreach (var gc in station.GraphicComponents)
            {
                if (gc is Mechanism mech)
                {
                    if (string.IsNullOrEmpty(mechName) || mech.Name == mechName)
                        return mech;
                }
            }

            return null;
        }

        /// <summary>
        /// Detach a part from mechanism and delete it.
        /// </summary>
        private void DetachAndDelete(Mechanism mech, Part part)
        {
            try { mech?.GetFlange(0).Detach(part); } catch { }
            try { Station.ActiveStation?.GraphicComponents.Remove(part); } catch { }
            try { part.Delete(); } catch { }
        }

        /// <summary>
        /// No-op (Ready signal removed). RAPID uses WaitTime instead.
        /// </summary>
        private void PulseReady(SmartComponent component)
        {
        }

        private void LogWarning(string message)
        {
            Logger.AddMessage(new LogMessage(
                $"BeamSimulator: {message}",
                LogMessageSeverity.Warning));
        }
    }
}
