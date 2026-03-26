from compas_fab.backends.ros.messages import ROSmsg

from compas_rrc.common import ExecutionLevel
from compas_rrc.common import FeedbackLevel
from compas_rrc.motion import Motion

INSTRUCTION_PREFIX = "r_Gudel_HSLU_"

__all__ = [
    "MoveToJoints",
    "MoveToRobtarget",
]


class MoveToJoints(ROSmsg):
    """Move to joints is a call that moves the robot and the external axes with axes values.

    Examples
    --------
    .. code-block:: python

        # Get joints
        robot_joints, external_axes = abb.send_and_wait(rrc.GetJoints())

        # Print received values
        print(robot_joints, external_axes)

        # Change value and move to new position
        robot_joints.rax_1 += 15
        speed = 100 # Unit [mm/s]
        done = abb.send_and_wait(rrc.MoveToJoints(robot_joints, external_axes, speed, rrc.Zone.FINE))

    RAPID Instruction: ``MoveAbsJ``

    .. include:: ../abb-reference.rst

    """

    def __init__(
        self, joints, ext_axes, speed, zone, feedback_level=FeedbackLevel.NONE
    ):
        """Create a new instance of the instruction.

        Parameters
        ----------
        joints : :class:`compas_rrc.RobotJoints` or :obj:`list` of :obj:`float`
            Robot joint positions.
        ext_axes : :class:`compas_rrc.ExternalAxes` or :obj:`list` of :obj:`float`
            External axes positions.
        speed : :obj:`float`
            Integer specifying TCP translational speed in mm/s. Min=``0.01``.
        zone : :class:`Zone`
            Zone data. Predefined in the robot controller,
            only Zone :attr:`Zone.FINE` will do a stop point all others are fly by points
        feedback_level : :obj:`int`
            Defines the feedback level requested from the robot. Defaults to :attr:`FeedbackLevel.NONE`.
            Use  :attr:`FeedbackLevel.DONE` and :attr:`Zone.FINE` together to make sure
            the motion planner has executed the instruction fully.
        """
        if speed <= 0:
            raise ValueError(
                "Speed must be higher than zero. Current value={}".format(speed)
            )

        self.instruction = INSTRUCTION_PREFIX + "MoveToJoints"
        self.feedback_level = feedback_level
        self.exec_level = ExecutionLevel.ROBOT

        joints = joints or []
        if len(joints) > 6:
            raise ValueError("Only up to 6 joints are supported")
        joints_pad = [0.0] * (6 - len(joints))

        ext_axes = ext_axes or []
        if len(ext_axes) > 6:
            raise ValueError("Only up to 6 external axes are supported")

        ext_axes_pad = [0.0] * (6 - len(ext_axes))
        self.string_values = []
        self.float_values = (
            list(joints) + joints_pad + list(ext_axes) + ext_axes_pad + [speed, zone]
        )


class MoveToRobtarget(ROSmsg):
    """Move to robtarget with coordinated track motion.

    Moves the robot in cartesian space while coordinating with the external track axis.
    Both robot and track move simultaneously and finish at the same time.

    Examples
    --------
    .. code-block:: python

        # Koordinierte Bewegung: Frame + Track
        done = abb.send_and_wait(cm.MoveToRobtarget(
            frame=target_frame,
            ext_axes=[track_pos],  # Track position in mm
            time=3,                # Motion duration in seconds
            zone=rrc.Zone.FINE,
            motion_type=Motion.JOINT
        ))

    RAPID Instruction: ``r_Gudel_HSLU_MoveTo`` (MoveJ or MoveL with track coordination)

    """

    def __init__(
        self,
        frame,
        ext_axes,
        time,
        zone,
        motion_type=Motion.JOINT,
        feedback_level=FeedbackLevel.NONE,
    ):
        """Create a new instance of the instruction.

        Parameters
        ----------
        frame : :class:`compas.geometry.Frame`
            Target frame.
        ext_axes : :obj:`list` of :obj:`float`
            External axes positions. First value is track position in mm.
        time : :obj:`float`
            Motion duration in seconds. Robot and track coordinate to finish together.
        zone : :class:`Zone`
            Zone data. Predefined in the robot controller.
        motion_type : :class:`Motion`
            Motion type. Defaults to :attr:`Motion.JOINT`.
        feedback_level : :obj:`int`
            Defines the feedback level requested from the robot.
        """
        if time <= 0:
            raise ValueError(
                "Time must be higher than zero. Current value={}".format(time)
            )

        self.feedback_level = feedback_level
        self.exec_level = ExecutionLevel.ROBOT

        pos = list(frame.point)
        rot = list(frame.quaternion)

        ext_axes = ext_axes or []
        if len(ext_axes) > 6:
            raise ValueError("Only up to 6 external axes are supported")
        ext_axes_pad = [0.0] * (6 - len(ext_axes))

        self.string_values = ["J"] if motion_type == Motion.JOINT else ["L"]
        self.float_values = pos + rot + list(ext_axes) + ext_axes_pad + [time, zone]

        self.instruction = INSTRUCTION_PREFIX + "MoveTo"
