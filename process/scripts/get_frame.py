# Imports
import compas
from compas.geometry import Frame, Rotation
import compas_rrc as rrc


TOOL = 't_HSLU_virtual_Spike'

W_OBJ = 'wobj0'

# Acceleration, Override, and Max Speed
ACC = 50  # Unit [%]
RAMP = 50  # Unit [%]
OVERRIDE = 100  # Unit [%]
MAX_TCP = 300  # Unit [mm/s]

# Joint positions
ROBOT_HOME_POS = [0, -40, 40, 0, 90, 0]

# External joints
ROBOT_EXTERNAL_AXIS = []

# Speeds
SPEED = 300  # mm/s




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

    frame = abb.send_and_wait(rrc.GetFrame())

    # Print received values
    print(frame)

    robot_joints, external_axes = abb.send_and_wait(rrc.GetJoints())

    print('Robot joints:', robot_joints)

    # End of Code
    print('Finished')

    # Close client
    ros.close()
    ros.terminate()