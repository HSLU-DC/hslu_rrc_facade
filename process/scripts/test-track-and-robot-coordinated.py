# ==============================
# Path Setup
# ==============================
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# ==============================
# Imports
# ==============================
import time as _time

import compas_rrc as rrc
import _skills.custom_motion as cm

from globals import ROBOT_NAME, TOOL_GRIPPER, W_OBJ_PLACE


# Two test poses with substantial track AND robot motion between them.
POSE_A = {"robax": [0, -25, 25, 0, 90, -180], "extax": [1000.0]}
POSE_B = {"robax": [90, -25, 25, 0, 90,  180], "extax": [2000.0]}

# Time parameter passed to the coordinated move (seconds).
COORD_TIME = 10


def coord_move(r1, label, robax, extax, t):
    """Send a coordinated MoveToJoints and time how long it actually takes."""
    print(f"\n[{label}] target robax={robax} extax={extax} time={t}s")
    t0 = _time.time()
    r1.send_and_wait(cm.MoveToJoints(robax, extax, t, rrc.Zone.FINE))
    dt = _time.time() - t0
    ratio = dt / t if t > 0 else 0
    print(f"[{label}] elapsed wall-time: {dt:.2f}s (target {t}s, ratio {ratio:.2f}x)")


def coord_move_frame(r1, label, frame, extax, t, motion_type=rrc.Motion.JOINT):
    """Send a coordinated MoveToRobtarget and time how long it actually takes."""
    p = frame.point
    mtype_str = "LINEAR" if motion_type == rrc.Motion.LINEAR else "JOINT"
    print(f"\n[{label}] frame X={p.x:.1f} Y={p.y:.1f} Z={p.z:.1f} extax={extax} time={t}s motion={mtype_str}")
    t0 = _time.time()
    r1.send_and_wait(cm.MoveToRobtarget(frame, extax, t, rrc.Zone.FINE, motion_type=motion_type))
    dt = _time.time() - t0
    ratio = dt / t if t > 0 else 0
    print(f"[{label}] elapsed wall-time: {dt:.2f}s (target {t}s, ratio {ratio:.2f}x)")


if __name__ == "__main__":
    ros = rrc.RosClient()
    ros.run()

    r1 = rrc.AbbClient(ros, ROBOT_NAME)
    print("Connected.")

    r1.send(rrc.SetTool(TOOL_GRIPPER))

    # ============================================================
    # JOINT TEST (cm.MoveToJoints)
    # ============================================================
    print("\n========== JOINT TEST ==========")
    coord_move(r1, "A (settle)", POSE_A["robax"], POSE_A["extax"], COORD_TIME)
    coord_move(r1, "A->B (test)", POSE_B["robax"], POSE_B["extax"], COORD_TIME)
    coord_move(r1, "B->A (return)", POSE_A["robax"], POSE_A["extax"], COORD_TIME)

    # ============================================================
    # ROBTARGET TEST (cm.MoveToRobtarget)
    # ============================================================
    print("\n========== ROBTARGET TEST ==========")
    r1.send(rrc.SetWorkObject(W_OBJ_PLACE))

    # Use current TCP as frame_a, then offset for frame_b. Both axes (TCP + track)
    # change between the two moves so coordination is exercised.
    frame_a = r1.send_and_wait(rrc.GetFrame())
    print(f"Current TCP (frame_a): X={frame_a.point.x:.1f} Y={frame_a.point.y:.1f} Z={frame_a.point.z:.1f}")

    frame_b = frame_a.copy()
    frame_b.point.y += 300
    frame_b.point.z += 100

    coord_move_frame(r1, "FRAME A (settle)", frame_a, POSE_A["extax"], COORD_TIME, motion_type=rrc.Motion.LINEAR)
    coord_move_frame(r1, "FRAME A->B (test)", frame_b, POSE_B["extax"], COORD_TIME, motion_type=rrc.Motion.LINEAR)
    coord_move_frame(r1, "FRAME B->A (return)", frame_a, POSE_A["extax"], COORD_TIME, motion_type=rrc.Motion.LINEAR)

    print("\nFinished.")
    print("Reading: ratio ~1.0 = parallel coordinated motion.")
    print("         ratio ~2.0 = sequential (track then robot, or vice versa).")

    ros.close()
    ros.terminate()
