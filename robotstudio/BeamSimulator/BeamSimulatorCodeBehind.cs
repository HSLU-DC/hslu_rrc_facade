// BeamSimulatorCodeBehind.cs
// SmartComponent code-behind for dynamic beam geometry visualization.
//
// Lifecycle per element:
//   Activate -> raw STL loaded, parented under HeldGroup, attached to gripper flange
//   SwapCutA -> raw replaced with cut-A STL (still under HeldGroup)
//   SwapCutB -> cut-A replaced with finished STL (still under HeldGroup)
//   Release  -> beam detached from gripper, re-parented from HeldGroup to PlacedGroup
//   Reset    -> all parts in HeldGroup + PlacedGroup deleted, state cleared
//
// HeldGroup and PlacedGroup are child SmartComponents inside BeamSimulator,
// created on first use. They give Collision Sets two stable container references:
//   ObjektA: robot + gripper + BeamSimulator/HeldGroup
//   ObjektB: static fixtures + BeamSimulator/PlacedGroup
// Released beams automatically migrate from ObjektA to ObjektB.

using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Linq;
using ABB.Robotics.Controllers;
using ABB.Robotics.Controllers.RapidDomain;
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
        private const string KEY_CACHE_RESTORED = "GeometryFolderCacheRestored";
        private const string KEY_RUNTIME_FOLDER = "RuntimeGeometryFolder";
        private const string KEY_RUNTIME_TOOL = "RuntimeToolName";
        private const string KEY_HELD_GROUP = "HeldGroupRef";
        private const string KEY_PLACED_GROUP = "PlacedGroupRef";

        // Container child SC names. Looked up by name so they survive station reload.
        private const string HELD_GROUP_NAME = "HeldGroup";
        private const string PLACED_GROUP_NAME = "PlacedGroup";

        // RAPID PERS variable names populated by r_HSLU_SimBeamReset.
        // If set and non-empty, they override the SC's Property values.
        private const string RAPID_TASK = "T_ROB1";
        private const string RAPID_MODULE = "HSLU_SimBeam";
        private const string PERS_GEOMETRY_FOLDER = "sim_geometry_folder";
        private const string PERS_TOOL_NAME = "sim_tool_name";

        // Per-station GeometryFolder cache: %APPDATA%\HSLU_BeamSimulator\station_paths.txt
        // Format: <station_path>\t<geometry_folder>\n
        private const string CACHE_DIR_NAME = "HSLU_BeamSimulator";
        private const string CACHE_FILE_NAME = "station_paths.txt";

        // Light wood color applied to every loaded beam body.
        private static readonly Color WoodColor = Color.FromArgb(180, 130, 80);

        // ============================================================
        // Initialization
        // ============================================================

        public override void OnInitialize(SmartComponent component)
        {
            base.OnInitialize(component);
            InitializeStateCache(component);
            TryRestoreGeometryFolder(component);
            EnsureGroups(component);
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
                if (string.IsNullOrEmpty(folder))
                    return;

                if (!Directory.Exists(folder))
                {
                    Logger.AddMessage(
                        new LogMessage($"BeamSimulator: GeometryFolder not found: {folder}",
                        LogMessageSeverity.Warning));
                    return;
                }

                // Remember this folder for the current station so the next time
                // the same .rsstn is opened, the value is auto-populated.
                SaveStationCacheEntry(folder);
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
                case "do_SC_Activate":
                    HandleActivate(component);
                    break;
                case "do_SC_SwapCutA":
                    HandleSwapCut(component, "cutA");
                    break;
                case "do_SC_SwapCutB":
                    HandleSwapCut(component, "cutB");
                    break;
                case "do_SC_Release":
                    HandleRelease(component);
                    break;
                case "do_SC_Reset":
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
                int elementId = Convert.ToInt32(component.IOSignals["go_SC_ElementID"].Value);
                int layerId = Convert.ToInt32(component.IOSignals["go_SC_LayerID"].Value);
                Logger.AddMessage(new LogMessage(
                    $"BeamSimulator: ElementID={elementId}, LayerID={layerId}"));

                component.StateCache[KEY_CURRENT_ELEMENT] = elementId;
                component.StateCache[KEY_CURRENT_LAYER] = layerId;

                // Build filename. Runtime value from RAPID PERS (cached on
                // Reset) wins over the SC Property so students don't have to
                // type the path in RobotStudio.
                string folder = (component.StateCache[KEY_RUNTIME_FOLDER] as string)
                    ?? component.Properties["GeometryFolder"].Value as string;
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
                Part part = LoadStl(component, filePath);
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
                RemoveFromAnyParent(component, oldPart);
                try { oldPart.Delete(); } catch { }
                component.StateCache[KEY_CURRENT_PART] = null;
                Logger.AddMessage(new LogMessage("BeamSimulator: Old part removed"));
            }

            // Load new STL and attach at TCP. Same override precedence as
            // HandleActivate: runtime PERS cache first, property as fallback.
            string folder = (component.StateCache[KEY_RUNTIME_FOLDER] as string)
                ?? component.Properties["GeometryFolder"].Value as string;
            string fileName = $"L{layerId}_E{elementId}_{stage}.stl";
            string filePath = Path.Combine(folder, fileName);

            Part newPart = LoadStl(component, filePath);
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

                // Re-parent: HeldGroup -> PlacedGroup. Migrating the part across
                // SC containers is what makes Collision Sets auto-update — held
                // beam was in ObjektA, placed beam becomes obstacle in ObjektB.
                ReparentToPlaced(component, currentPart);

                // Restore world position (Detach + Re-parent may have moved it)
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

                // RAPID has just written sim_geometry_folder / sim_tool_name
                // into the PERS before pulsing this signal. Pull the current
                // values and cache them for subsequent Activate / TCP lookups.
                RefreshRuntimeOverrides(component);

                // Delete current part if any
                Part currentPart = component.StateCache[KEY_CURRENT_PART] as Part;
                Mechanism mech = FindMechanism(component);
                if (currentPart != null)
                {
                    DetachAndDelete(component, mech, currentPart);
                    Logger.AddMessage(new LogMessage("BeamSimulator: Deleted current part"));
                }

                // Delete all placed beams (clear PlacedGroup children + tracked list)
                var placedBeams = component.StateCache[KEY_PLACED_BEAMS] as List<Part>;
                int count = 0;
                if (placedBeams != null)
                {
                    count = placedBeams.Count;
                    for (int i = placedBeams.Count - 1; i >= 0; i--)
                    {
                        try
                        {
                            RemoveFromAnyParent(component, placedBeams[i]);
                            placedBeams[i].Delete();
                        }
                        catch { }
                    }
                    placedBeams.Clear();
                }

                // Belt-and-suspenders: also clear any orphaned children inside
                // PlacedGroup that aren't tracked in placedBeams (e.g. survivors
                // from a previous session loaded from .rsstn).
                SmartComponent placedGroup = GetPlacedGroup(component);
                if (placedGroup != null)
                {
                    var leftovers = new List<GraphicComponent>();
                    foreach (GraphicComponent gc in placedGroup.GraphicComponents)
                        leftovers.Add(gc);
                    foreach (var gc in leftovers)
                    {
                        try { placedGroup.GraphicComponents.Remove(gc); } catch { }
                        try { (gc as Part)?.Delete(); } catch { }
                    }
                }

                // Reset state
                component.StateCache[KEY_CURRENT_PART] = null;
                component.StateCache[KEY_CURRENT_ELEMENT] = -1;
                component.StateCache[KEY_CURRENT_LAYER] = -1;
                component.StateCache[KEY_CURRENT_STAGE] = "idle";

                // Reset also carries the folder/tool PERS update from RAPID,
                // so the cached TCP offset must be re-looked-up on next Activate.
                if (component.StateCache.ContainsKey(KEY_TCP_OFFSET))
                    component.StateCache.Remove(KEY_TCP_OFFSET);

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
        /// Load an STL file as a Part and parent it under HeldGroup (or, if the
        /// container can't be created, fall back to the BeamSimulator SC itself,
        /// or to the station root as last resort).
        /// </summary>
        private Part LoadStl(SmartComponent component, string filePath)
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
                    AddToHeldContainer(component, part);
                    ApplyWoodColor(part);
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
        /// Add a Part to the HeldGroup container. Falls back to the BeamSimulator
        /// SC itself if HeldGroup can't be created, then to the station root.
        /// </summary>
        private void AddToHeldContainer(SmartComponent component, Part part)
        {
            SmartComponent heldGroup = GetHeldGroup(component);
            if (heldGroup != null)
            {
                try { heldGroup.GraphicComponents.Add(part); return; }
                catch (Exception ex) { LogWarning($"AddToHeld (group): {ex.Message}"); }
            }
            try { component.GraphicComponents.Add(part); return; }
            catch (Exception ex) { LogWarning($"AddToHeld (parent): {ex.Message}"); }
            try { Station.ActiveStation?.GraphicComponents.Add(part); }
            catch (Exception ex) { LogWarning($"AddToHeld (station): {ex.Message}"); }
        }

        /// <summary>
        /// Colour a freshly loaded beam in a light wood tone.
        /// STL loads as a pure mesh - Bodies is empty. Part.Color is inherited from
        /// GraphicComponent and sets the display colour directly on the mesh part.
        /// </summary>
        private void ApplyWoodColor(Part part)
        {
            if (part == null)
                return;
            try
            {
                part.Color = WoodColor;
                Logger.AddMessage(new LogMessage(
                    $"BeamSimulator: Coloured {part.Name}"));
            }
            catch (Exception ex)
            {
                LogWarning($"ApplyWoodColor: {ex.Message}");
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
        /// RAPID PERS sim_tool_name (pushed via sim_beam_reset) overrides the
        /// SC's ToolName Property when populated.
        /// </summary>
        private Matrix4 LookupToolOffset(SmartComponent component)
        {
            try
            {
                string toolName = (component.StateCache[KEY_RUNTIME_TOOL] as string)
                    ?? component.Properties["ToolName"].Value as string;
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
        /// Read the current runtime value of a RAPID PERS string variable
        /// from the virtual controller. Uses the Controller SDK because the
        /// Stations SDK only exposes the declaration-time initial value.
        /// Returns null on any failure so the caller can fall back to a
        /// Property default.
        /// </summary>
        private string ReadPersString(string moduleName, string varName)
        {
            try
            {
                Station station = Station.ActiveStation;
                if (station == null)
                    return null;

                RsIrc5Controller rsCtrl = station.Irc5Controllers
                    .OfType<RsIrc5Controller>()
                    .FirstOrDefault();
                if (rsCtrl == null)
                    return null;

                using (Controller ctrl = Controller.Connect(
                    new Guid(rsCtrl.SystemId), ConnectionType.RobotStudio))
                {
                    ctrl.Logon(UserInfo.DefaultUser);
                    RapidData rd = ctrl.Rapid.GetRapidData(RAPID_TASK, moduleName, varName);
                    object v = rd?.Value;
                    string val = v?.ToString()?.Trim('"');
                    return string.IsNullOrEmpty(val) ? null : val;
                }
            }
            catch (Exception ex)
            {
                LogWarning($"ReadPersString({varName}): {ex.Message}");
                return null;
            }
        }

        /// <summary>
        /// Pull the runtime PERS values (folder, tool) from the controller
        /// and cache them. Called on Reset — the sim_beam_reset call from
        /// Python writes the PERS before pulsing do_SC_Reset, so the latest
        /// values are in place when this runs.
        /// </summary>
        private void RefreshRuntimeOverrides(SmartComponent component)
        {
            string folder = ReadPersString(RAPID_MODULE, PERS_GEOMETRY_FOLDER);
            string tool = ReadPersString(RAPID_MODULE, PERS_TOOL_NAME);

            component.StateCache[KEY_RUNTIME_FOLDER] = folder;
            component.StateCache[KEY_RUNTIME_TOOL] = tool;

            Logger.AddMessage(new LogMessage(
                $"BeamSimulator: Runtime overrides - folder='{folder ?? "(fallback)"}' " +
                $"tool='{tool ?? "(fallback)"}'"));
        }

        /// <summary>
        /// Find the first robot mechanism in the active station.
        /// </summary>
        private Mechanism FindMechanism(SmartComponent component)
        {
            Station station = Station.ActiveStation;
            if (station == null)
                return null;

            foreach (var gc in station.GraphicComponents)
            {
                if (gc is Mechanism mech)
                    return mech;
            }

            return null;
        }

        /// <summary>
        /// Detach a part from mechanism and delete it. Removes the part from
        /// whatever container it's currently parented under (HeldGroup, parent
        /// SC, or station root) before deleting.
        /// </summary>
        private void DetachAndDelete(SmartComponent component, Mechanism mech, Part part)
        {
            try { mech?.GetFlange(0).Detach(part); } catch { }
            RemoveFromAnyParent(component, part);
            try { part.Delete(); } catch { }
        }

        /// <summary>
        /// Remove a part from whichever container holds it. Tries HeldGroup,
        /// PlacedGroup, BeamSimulator SC, and station root in that order. The
        /// SDK doesn't expose a "current parent" lookup, so we just attempt
        /// removal from each plausible container and swallow failures.
        /// </summary>
        private void RemoveFromAnyParent(SmartComponent component, Part part)
        {
            SmartComponent heldGroup = GetHeldGroup(component);
            SmartComponent placedGroup = GetPlacedGroup(component);
            try { heldGroup?.GraphicComponents.Remove(part); } catch { }
            try { placedGroup?.GraphicComponents.Remove(part); } catch { }
            try { component.GraphicComponents.Remove(part); } catch { }
            try { Station.ActiveStation?.GraphicComponents.Remove(part); } catch { }
        }

        /// <summary>
        /// Move a Part from HeldGroup to PlacedGroup. Used on Release so the
        /// beam migrates from "actively held" (ObjektA in collision sets) to
        /// "static obstacle" (ObjektB) in one operation.
        /// </summary>
        private void ReparentToPlaced(SmartComponent component, Part part)
        {
            RemoveFromAnyParent(component, part);
            SmartComponent placedGroup = GetPlacedGroup(component);
            if (placedGroup != null)
            {
                try { placedGroup.GraphicComponents.Add(part); return; }
                catch (Exception ex) { LogWarning($"ReparentToPlaced (group): {ex.Message}"); }
            }
            // Fallbacks if PlacedGroup can't be created
            try { component.GraphicComponents.Add(part); return; }
            catch (Exception ex) { LogWarning($"ReparentToPlaced (parent): {ex.Message}"); }
            try { Station.ActiveStation?.GraphicComponents.Add(part); }
            catch (Exception ex) { LogWarning($"ReparentToPlaced (station): {ex.Message}"); }
        }

        // ============================================================
        // Container groups: HeldGroup + PlacedGroup as nested SmartComponents
        // ============================================================

        /// <summary>
        /// Make sure both container child SCs exist. Called on init and lazily
        /// from getters so the groups appear in the browser as soon as the
        /// BeamSimulator is instantiated.
        /// </summary>
        private void EnsureGroups(SmartComponent component)
        {
            GetHeldGroup(component);
            GetPlacedGroup(component);
        }

        private SmartComponent GetHeldGroup(SmartComponent component)
        {
            return GetOrFindGroup(component, KEY_HELD_GROUP, HELD_GROUP_NAME);
        }

        private SmartComponent GetPlacedGroup(SmartComponent component)
        {
            return GetOrFindGroup(component, KEY_PLACED_GROUP, PLACED_GROUP_NAME);
        }

        private SmartComponent GetOrFindGroup(SmartComponent component,
                                              string cacheKey, string groupName)
        {
            // Cached reference — fastest path
            if (component.StateCache.ContainsKey(cacheKey))
            {
                if (component.StateCache[cacheKey] is SmartComponent cached)
                {
                    // Sanity-check: ensure the cached SC is still attached to the parent.
                    // (User could have deleted it manually.)
                    foreach (GraphicComponent gc in component.GraphicComponents)
                    {
                        if (ReferenceEquals(gc, cached))
                            return cached;
                    }
                    component.StateCache.Remove(cacheKey);
                }
            }

            // Look up by name (handles station-reload case where SC tree is
            // restored from .rsstn but StateCache is empty)
            foreach (GraphicComponent gc in component.GraphicComponents)
            {
                if (gc is SmartComponent sc && sc.Name == groupName)
                {
                    component.StateCache[cacheKey] = sc;
                    return sc;
                }
            }

            // Create new
            try
            {
                var newSc = new SmartComponent { Name = groupName };
                component.GraphicComponents.Add(newSc);
                component.StateCache[cacheKey] = newSc;
                Logger.AddMessage(new LogMessage(
                    $"BeamSimulator: Created child SC '{groupName}'"));
                return newSc;
            }
            catch (Exception ex)
            {
                LogWarning($"GetOrFindGroup('{groupName}'): {ex.Message}");
                return null;
            }
        }

        /// <summary>
        /// No-op (Ready signal removed). RAPID uses WaitTime instead.
        /// </summary>
        private void PulseReady(SmartComponent component)
        {
        }

        // ============================================================
        // Per-station GeometryFolder cache
        // ============================================================
        //
        // A tiny key/value store in %APPDATA%\HSLU_BeamSimulator\station_paths.txt
        // that remembers which geometry folder a given .rsstn file was last
        // associated with. Each user sets the path once per station; every
        // subsequent open auto-populates the GeometryFolder property.
        //
        // Format: <station_full_path>\t<geometry_folder>\n
        // Keyed case-insensitively (Windows paths).

        /// <summary>
        /// Auto-populate GeometryFolder from AppData cache if:
        ///   - property is empty or points to a non-existent folder
        ///   - the active station has a file name (saved)
        ///   - the cache has an entry for that station
        /// </summary>
        private void TryRestoreGeometryFolder(SmartComponent component)
        {
            try
            {
                if (component.StateCache.ContainsKey(KEY_CACHE_RESTORED))
                    return;
                component.StateCache[KEY_CACHE_RESTORED] = true;

                string current = component.Properties["GeometryFolder"].Value as string;
                if (!string.IsNullOrEmpty(current) && Directory.Exists(current))
                    return;

                string stationKey = GetStationKey();
                if (string.IsNullOrEmpty(stationKey))
                    return;

                Dictionary<string, string> cache = LoadStationCache();
                if (!cache.TryGetValue(stationKey, out string cached))
                    return;

                if (!Directory.Exists(cached))
                {
                    Logger.AddMessage(new LogMessage(
                        $"BeamSimulator: Cached GeometryFolder no longer exists: {cached}",
                        LogMessageSeverity.Warning));
                    return;
                }

                component.Properties["GeometryFolder"].Value = cached;
                Logger.AddMessage(new LogMessage(
                    $"BeamSimulator: Restored GeometryFolder from cache: {cached}"));
            }
            catch (Exception ex)
            {
                LogWarning($"TryRestoreGeometryFolder: {ex.Message}");
            }
        }

        /// <summary>
        /// Write/overwrite the cache entry for the active station.
        /// </summary>
        private void SaveStationCacheEntry(string geometryFolder)
        {
            try
            {
                string stationKey = GetStationKey();
                if (string.IsNullOrEmpty(stationKey))
                    return;

                Dictionary<string, string> cache = LoadStationCache();
                cache[stationKey] = geometryFolder;
                SaveStationCache(cache);

                Logger.AddMessage(new LogMessage(
                    $"BeamSimulator: Cached GeometryFolder for {Path.GetFileName(stationKey)}"));
            }
            catch (Exception ex)
            {
                LogWarning($"SaveStationCacheEntry: {ex.Message}");
            }
        }

        /// <summary>
        /// Returns the cache key for the active station (lower-cased full path),
        /// or null if the station has not been saved yet.
        /// </summary>
        private string GetStationKey()
        {
            string stationPath = Station.ActiveStation?.FileInfo?.FullName;
            if (string.IsNullOrEmpty(stationPath))
                return null;
            return stationPath.ToLowerInvariant();
        }

        private Dictionary<string, string> LoadStationCache()
        {
            var cache = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
            string path = GetCachePath();
            if (!File.Exists(path))
                return cache;

            try
            {
                foreach (string line in File.ReadAllLines(path))
                {
                    if (string.IsNullOrWhiteSpace(line))
                        continue;
                    string[] parts = line.Split('\t');
                    if (parts.Length == 2)
                        cache[parts[0]] = parts[1];
                }
            }
            catch (Exception ex)
            {
                LogWarning($"LoadStationCache: {ex.Message}");
            }
            return cache;
        }

        private void SaveStationCache(Dictionary<string, string> cache)
        {
            string path = GetCachePath();
            try
            {
                Directory.CreateDirectory(Path.GetDirectoryName(path));
                var lines = new List<string>();
                foreach (KeyValuePair<string, string> kv in cache)
                    lines.Add(kv.Key + "\t" + kv.Value);
                File.WriteAllLines(path, lines);
            }
            catch (Exception ex)
            {
                LogWarning($"SaveStationCache: {ex.Message}");
            }
        }

        private string GetCachePath()
        {
            string appData = Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData);
            return Path.Combine(appData, CACHE_DIR_NAME, CACHE_FILE_NAME);
        }

        private void LogWarning(string message)
        {
            Logger.AddMessage(new LogMessage(
                $"BeamSimulator: {message}",
                LogMessageSeverity.Warning));
        }
    }
}
