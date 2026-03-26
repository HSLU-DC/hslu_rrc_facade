# test_connection.py
"""Simple connectivity test: Docker -> ROS -> ABB Controller.

Run this first to verify the system is working before production.
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import compas_rrc as rrc
from globals import ROBOT_NAME, TOOL_GRIPPER


def test_connection():
    print("=" * 50)
    print("CONNECTION TEST")
    print("=" * 50)

    # Step 1: ROS
    print("\n1. Connecting to ROS...")
    try:
        ros = rrc.RosClient()
        ros.run()
        print("   [OK] ROS connected")
    except Exception as e:
        print(f"   [FEHLER] ROS connection failed: {e}")
        print("\n   Ist Docker gestartet? -> cd docker && docker compose up -d")
        return False

    # Step 2: ABB
    print("\n2. Connecting to ABB controller...")
    try:
        r1 = rrc.AbbClient(ros, ROBOT_NAME)
        print("   [OK] ABB client created")
    except Exception as e:
        print(f"   [FEHLER] ABB client failed: {e}")
        ros.close()
        return False

    # Step 3: Read position
    print("\n3. Reading current robot position...")
    try:
        r1.send(rrc.SetTool(TOOL_GRIPPER))
        joints, extax = r1.send_and_wait(rrc.GetJoints())
        frame = r1.send_and_wait(rrc.GetFrame())
        print(f"   [OK] Joints: {[f'{j:.1f}' for j in joints]}")
        print(f"   [OK] ExtAx:  {[f'{e:.1f}' for e in extax]}")
        print(f"   [OK] TCP:    X={frame.point.x:.1f} Y={frame.point.y:.1f} Z={frame.point.z:.1f}")
    except Exception as e:
        print(f"   [FEHLER] Could not read position: {e}")
        print("\n   Ist der Controller eingeschaltet und im AUTO-Modus?")
        ros.close()
        return False

    # Cleanup
    ros.close()
    ros.terminate()

    print("\n" + "=" * 50)
    print("ALLE TESTS BESTANDEN")
    print("=" * 50)
    return True


if __name__ == "__main__":
    success = test_connection()
    sys.exit(0 if success else 1)
