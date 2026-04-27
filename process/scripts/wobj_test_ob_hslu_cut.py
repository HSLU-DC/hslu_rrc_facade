# Imports
import compas
from compas.geometry import Frame, Rotation
import compas_rrc as rrc


TOOL = 't_HSLU_virtual_Spike'

W_OBJ = 'ob_HSLU_Cut'

# Acceleration, Override, and Max Speed
ACC = 50  # Unit [%]
RAMP = 50  # Unit [%]
OVERRIDE = 100  # Unit [%]
MAX_TCP = 300  # Unit [mm/s]

# Joint positions
ROBOT_HOME_POS = [-70, 20, 40, 20, 20, 0]

# External joints
ROBOT_EXTERNAL_AXIS = []

# Speeds
SPEED = 50  # mm/s




# Erstelle ein Frame bei (0, 0, 0) mit einer negativen Z-Achse
origin = [0, 0, 0]
xaxis = [0.7071067811865476, 0, 0.7071067811865475]  # Standard X-Achse
yaxis = [0, -1, 0]  # Damit die Z-Achse negativ ist, Y nach -1 zeigen lassen

frame = Frame(origin, xaxis, yaxis)
# ==============================================================================
# Define functions
# ==============================================================================

# ==============================================
# Move to the home position
# ==============================================

def move_home():
    abb.send(rrc.MoveToJoints(ROBOT_HOME_POS, ROBOT_EXTERNAL_AXIS, SPEED, rrc.Zone.Z50))



if __name__ == '__main__':
    # Create Ros Client
    ros = rrc.RosClient()
    ros.run()

    # Create ABB Client
    abb = rrc.AbbClient(ros, 'rob1')
    print('Connected.')

    # Set tool to gripper 
    abb.send(rrc.SetTool(TOOL))

    # Set work object to table
    abb.send(rrc.SetWorkObject(W_OBJ))    

    # Set Acceleration
    abb.send(rrc.SetAcceleration(ACC, RAMP))

    # Set Max Speed
    abb.send(rrc.SetMaxSpeed(OVERRIDE, MAX_TCP))

    # Make sure to start in home position
    move_home()

    abb.send(rrc.Stop())

    frame.point.z += 80
    abb.send(rrc.MoveToFrame(frame, SPEED, rrc.Zone.FINE, rrc.Motion.JOINT))

    abb.send(rrc.Stop())

    frame.point.z -= 70
    abb.send(rrc.MoveToFrame(frame, 10, rrc.Zone.FINE, rrc.Motion.LINEAR))

    abb.send(rrc.Stop())

    frame.point.z -= 10
    abb.send(rrc.MoveToFrame(frame, 5, rrc.Zone.FINE, rrc.Motion.LINEAR))

    abb.send(rrc.Stop())

    frame.point.y += 200
    abb.send(rrc.MoveToFrame(frame, 5, rrc.Zone.FINE, rrc.Motion.LINEAR))

    abb.send(rrc.Stop())

    frame.point.z += 50
    abb.send(rrc.MoveToFrame(frame, 50, rrc.Zone.FINE, rrc.Motion.LINEAR))

    # Return to home position
    move_home()

    # End of Code
    print('Finished')

    # Close client
    ros.close()
    ros.terminate()