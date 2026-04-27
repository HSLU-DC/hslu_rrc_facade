# Imports
import compas
from compas.geometry import Frame, Rotation
import compas_rrc as rrc



TOOL = 't_HSLU_virtual_Spike'

W_OBJ = 'ob_HSLU_Pick_small'

# Acceleration, Override, and Max Speed
ACC = 100  # Unit [%]
RAMP = 100  # Unit [%]
OVERRIDE = 100  # Unit [%]
MAX_TCP = 1000  # Unit [mm/s]

# Joint positions
ROBOT_HOME_POS = [0, 10, 40, 0, 40, 0]

# External joints
ROBOT_EXTERNAL_AXIS = []

# Speeds
SPEED = 500  # mm/s




# Erstelle ein Frame bei (0, 0, 0) mit einer negativen Z-Achse
origin = [0, 0, 0]
xaxis = [1, 0, 0]  # Standard X-Achse
yaxis = [0, -1, 0]  # Damit die Z-Achse negativ ist, Y nach -1 zeigen lassen

frame = Frame(origin, xaxis, yaxis)
# ==============================================================================
# Define functions
# ==============================================================================

# ==============================================
# Move to the home position
# ==============================================

def move_home():
    abb.send_and_wait(rrc.MoveToJoints(ROBOT_HOME_POS, ROBOT_EXTERNAL_AXIS, SPEED, rrc.Zone.FINE))



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

    abb.send(rrc.MoveToFrame(frame, SPEED, rrc.Zone.FINE, rrc.Motion.JOINT))

    # End of Code
    print('Finished')

    # Close client
    ros.close()
    ros.terminate()