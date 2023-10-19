"""This module holds the TrajectoryGenerator components
"""
import enum
from enum import Enum
import math
import time
from typing import Callable, NamedTuple

from droneresponse_mathtools import Lla
from ruckig import InputParameter, Result, Ruckig, Trajectory
from dr_onboard_autonomy.gimbal.data_types import Quaternion
from dr_onboard_autonomy.logutils import DebounceLogger
import rospy
import geometry_msgs.msg
import mavros_msgs.msg
import sensor_msgs.msg
from tf.transformations import euler_from_quaternion
import numpy as np

from dr_onboard_autonomy.mavros_layer import MAVROSDrone
from dr_onboard_autonomy.briar_helpers import BriarLla, convert_Lla_to_LlaDict, convert_tuple_to_LlaDict
from dr_onboard_autonomy.states.BaseState import BaseState
from dr_onboard_autonomy.states.components.Setpoint import Setpoint
from dr_onboard_autonomy.message_senders import RepeatTimer


def _load_constraints(drone: MAVROSDrone):
    params = {
        "MPC_ACC_HOR": False,
        "MPC_ACC_UP_MAX": False,
        "MPC_ACC_DOWN_MAX": False,
        "MPC_JERK_AUTO": False,
        "MPC_YAWRAUTO_MAX": False,
    }
    for param_name in params:
        params[param_name] = drone.get_param_real(param_name)

    return params


def _find_yaw(q: Quaternion) -> float:
    """Find the yaw angle from a quaternion

    The yaw angle is around the up axis.

    Returns the drone's yaw angle in radians where 0 is East
    """
    tup = q.x, q.y, q.z, q.w
    euler = euler_from_quaternion(tup)
    return euler[2]


def _find_angular_distance(a: float, b: float) -> float:
    """Find the angular distance between two angles

    The angular distance is the amount of rotation needed to spin from angle a
    to angle b.
    """
    v0 = math.cos(a), math.sin(a)
    v1 = math.cos(b), math.sin(b)

    dot_product = np.dot(v0, v1)

    '''
    handler rare cases where floating point error results in dot products just slightly outside
    of the acos() domain of -1.0 to 1.0
    '''
    if dot_product > 1.0 and dot_product < 1.0001:
        dot_product = 1.0

    if dot_product < -1.0 and dot_product > -1.0001:
        dot_product = -1.0
    
    return math.acos(dot_product)


def _find_rotation_axis(a: float, b: float) -> float:
    """Find the axis of rotation between two angles

    There are two possible axes of rotation: up or down. This function returns
    1 for up and -1 for down.
    """

    v0 = np.array([math.cos(a), math.sin(a), 0])
    v1 = np.array([math.cos(b), math.sin(b), 0])


    # if v0 and v1 are the same (or nearly the same)
    # then return 1 (up)
    e = np.dot(v0, v1)
    if e >= 0 and 1 - e < 1e-6:
        return 1

    rotaiton_axis = np.cross(v0, v1)

    if rotaiton_axis[2] >= 0:
        return 1
    else:
        return -1


class TrajectoryGenerator:
    def __init__(self, state: BaseState, setpoint_driver: Setpoint = None):
        self.dr_state = state
        self.drone: MAVROSDrone = state.drone
        self.message_senders = state.message_senders
        self.reusable_message_senders = state.reusable_message_senders
        self.handlers = state.handlers

        # We need a way for of running two trajectories at the same time:
        # 1. one for the translational motion
        # 2. one for the drone's yaw
        # This is because the drone's yaw is needed to point the camera at the
        # subject.
        if setpoint_driver is None:
            setpoint_driver = Setpoint(self.dr_state)
        self.setpoint_driver = setpoint_driver

        self.constraints = _load_constraints(self.drone)
        # self.max_horizontal_acceleration: float = abs(constraints["MPC_ACC_HOR"])
        # self.max_down_acceleration: float = abs(constraints["MPC_ACC_DOWN_MAX"])
        # self.max_up_acceleration: float = abs(constraints["MPC_ACC_UP_MAX"])
        # self.max_jerk: float = abs(constraints["MPC_JERK_AUTO"])
        # self.max_yaw_velocity: float = constraints["MPC_YAWRAUTO_MAX"]

    def start(self):
        self.setpoint_driver.start()

    def stop(self):
        self.setpoint_driver.stop()

    def pause(self):
        pass

    def resume(self):
        pass


Velocity = NamedTuple("Velocity", north=float, east=float, down=float)


Acceleration = NamedTuple("Acceleration", north=float, east=float, down=float)

# how long to wait for the drone to catch up to the setpoint
# the setpoints lead the drone. This leads to jerky motion when the drone arrives at the waypoint.
# This is because we're near the final position and the last setpoint value has been calculated
# But we've not yet arrived at the final position.
# to give the drone time to actually arrive, this state will delay transitioning until
# this much time has passed:
TRANSITION_DELAY = 2.0 # seconds


class _DroneData:
    """ we need a nested component to handle sensor data
    
    All the trajectory generators need some common sensor data.
    Wiring all of this sensor data together is non-trivial.
    This component exists so that we can reuse the code that gathers sensor data.

    This component keeps track of the drone's:

    - mode (so we can see if we're in OFFBOARD mode)
    - position using BriarLla
    - velocity in NED
    - acceleration in NED
    """

    def __init__(self, base_state: BaseState):
        self.message_senders = base_state.message_senders
        self.reusable_message_senders = base_state.reusable_message_senders
        self.handlers = base_state.handlers

        message_sender_names = ["state", "imu", "velocity", "position"]
        for msg_sender_name in message_sender_names:
            msg_sender = self.reusable_message_senders.find(msg_sender_name)
            self.message_senders.add(msg_sender)

        self.handlers.add_handler("state", self._on_state_message)
        self.handlers.add_handler("imu", self._on_imu_message)
        self.handlers.add_handler("velocity", self._on_velocity_message)
        self.handlers.add_handler("position", self._on_position_message)

        self._mode: str = None
        self._position: BriarLla = None
        self._velocity_ned: Velocity = None
        self._acceleration_ned: Acceleration = None
        self._attitude: Quaternion = None

    @property
    def mode(self) -> str:
        return self._mode

    @property
    def position(self) -> BriarLla:
        return self._position

    @property
    def velocity(self) -> Velocity:
        return self._velocity_ned

    @property
    def acceleration(self) -> Acceleration:
        """Return the linear acceleration as a tuple with (north, east, down) components in m/s^2
        """
        return self._acceleration_ned

    @property
    def attitude(self) -> Quaternion:
        """Return the drone's attitude as a Quaternion
        """
        return self._attitude

    def is_data_available(self):
        return all([
            self._mode is not None,
            self._position is not None,
            self._velocity_ned is not None,
            self._acceleration_ned is not None,
            self._attitude is not None,
        ])

    def reset(self):
        """clear all the data
        This is used in case we resume. We need a way to clear out the old data.
        That way we can operate on fresh data.
        """
        self._mode: str = None
        self._position: BriarLla = None
        self._velocity_ned: Velocity = None
        self._acceleration_ned: Acceleration = None

    def _on_state_message(self, msg):
        state_msg: mavros_msgs.msg.State = msg['data']
        self._mode = state_msg.mode

    def _on_imu_message(self, msg):
        imu_msg: sensor_msgs.msg.Imu = msg['data']

        x = imu_msg.orientation.x
        y = imu_msg.orientation.y
        z = imu_msg.orientation.z
        w = imu_msg.orientation.w
        self._attitude = Quaternion(x=x, y=y, z=z, w=w)

        a_enu = imu_msg.linear_acceleration
        east = a_enu.x
        north = a_enu.y
        down = a_enu.z * -1.0
        self._acceleration_ned = Acceleration(north, east, down)

    def _on_velocity_message(self, msg):
        vel_msg: geometry_msgs.msg.TwistStamped = msg["data"]
        vel_enu = vel_msg.twist.linear
        east = vel_enu.x
        north = vel_enu.y
        down = vel_enu.z * -1.0
        self._velocity_ned = Velocity(north, east, down)

    def _on_position_message(self, msg):
        pos_msg: sensor_msgs.msg.NavSatFix = msg["data"]
        lat = pos_msg.latitude
        lon = pos_msg.longitude
        alt_wgs84 = pos_msg.altitude
        pos_tup_wgs84 = (lat, lon, alt_wgs84)
        lla_dict_wgs84 = convert_tuple_to_LlaDict(pos_tup_wgs84)
        self._position = BriarLla(lla_dict_wgs84, is_amsl=False)


TrajectoryFunction = Callable[[], BriarLla]

class WaypointTrajectory(TrajectoryGenerator):

    class State(Enum):
        STARTING = enum.auto()
        MOVING = enum.auto()
        PAUSED = enum.auto()
        DONE = enum.auto()

    def __init__(self, state: BaseState):
        super().__init__(state)
        self.data: _DroneData = _DroneData(state)

        # self.message_senders.add(RepeatTimer("trajectory_update", 1/50))
        self.handlers.add_handler(Setpoint.TIMER_MESSAGE_NAME, self._on_trajectory_update, is_active=False)

        self._internal_state = self.State.STARTING
        self._final_position: BriarLla = None
        self._speed_limit: float = None

        self._t0: float = None
        self._tf: float = None # t final the time when the trajectory is done
        self._initial_position: BriarLla = None
        self._trajectory_func: TrajectoryFunction = None
        self._trajectory_duration: float = None

        self.yaw_trajectory: YawTrajectory = YawTrajectory(state, self.setpoint_driver, self.data)

        self.debouce_logger = DebounceLogger()

    def start(self):
        super().start()

    @property
    def yaw(self) -> str:
        """The yaw target in radians
        The yaw is rotation around the up axis, where 0 is East.
        """
        return self.yaw_trajectory.yaw
    
    @yaw.setter
    def yaw(self, yaw: float):
        self.yaw_trajectory.yaw = yaw

    def pause(self):
        rospy.loginfo("Pausing trajectory")
        self.yaw_trajectory.pause()
        if self.data.is_data_available():
            self.setpoint_driver.lla = self.data.position.amsl.tup
        else:
            self.setpoint_driver.velocity = (0, 0, 0)

        self._internal_state = self.State.PAUSED

    def resume(self):
        self._internal_state = self.State.STARTING
        self.data.reset()
        self.setpoint_driver.start()
        self.yaw_trajectory.resume()

    def is_done(self):
        return self._internal_state == self.State.DONE


    def fly_to_waypoint(self, waypoint_amsl: Lla, speed_limit: float):
        """
        """
        waypoint_amsl = convert_Lla_to_LlaDict(waypoint_amsl)
        self._final_position = BriarLla(waypoint_amsl, is_amsl=True)
        self._speed_limit = speed_limit
        self._internal_state = self.State.STARTING
        rospy.loginfo("Starting trajectory to waypoint: {}".format(self._final_position))

    def _make_trajectory(self):
        self._initial_position = self.data.position

        inp: InputParameter = InputParameter(3)
        inp.current_position = [0.0, 0.0, 0.0]
        inp.current_velocity = [0.0, 0.0, 0.0] # list(self.data.velocity)
        inp.current_acceleration = [0.0, 0.0, 0.0] # list(self.data.acceleration)

        final_position_ned = self._initial_position.ellipsoid.lla.distance_ned(self._final_position.ellipsoid.lla)
        inp.target_position = [float(x) for x in final_position_ned]
        inp.target_velocity = [0.0, 0.0, 0.0]
        inp.target_acceleration = [0.0, 0.0, 0.0]

        inp.max_velocity = [self._speed_limit, self._speed_limit, self._speed_limit]
        max_horizontal_acceleration = self.constraints["MPC_ACC_HOR"]
        max_down_acceleration = self.constraints["MPC_ACC_DOWN_MAX"]
        max_up_acceleration = self.constraints["MPC_ACC_UP_MAX"]
        inp.max_acceleration = [
            max_horizontal_acceleration,
            max_horizontal_acceleration,
            max_down_acceleration
        ]
        inp.min_acceleration = [
            -1.0 * max_horizontal_acceleration,
            -1.0 * max_horizontal_acceleration,
            -1.0 * max_up_acceleration
        ]
        max_jerk = self.constraints["MPC_JERK_AUTO"]
        inp.max_jerk = [
            max_jerk,
            max_jerk,
            max_jerk
        ]

        otg = Ruckig(3)
        trajectory = Trajectory(3)

        # Calculate the trajectory in an offline manner
        calc_result = otg.calculate(inp, trajectory)
        if calc_result == Result.ErrorInvalidInput:
            raise Exception('Invalid input!')

        self._trajectory_duration = trajectory.duration
        self._t0 = None
        self._tf = None

        def trajectory_func() -> BriarLla:
            if self._t0 is None:
                self._t0 = time.time()
                self._tf = self._t0 + self._trajectory_duration + TRANSITION_DELAY
                t = 0.0
            else:
                t = time.time() - self._t0

            # Then, we can calculate the kinematic state at a given time
            new_position, new_velocity, new_acceleration = trajectory.at_time(t)
            north = new_position[0]
            east = new_position[1]
            down = new_position[2]

            setpoint_lla_ellipsoid = self._initial_position.ellipsoid.lla.move_ned(north, east, down)

            setpoint_wgs84 = convert_Lla_to_LlaDict(setpoint_lla_ellipsoid)
            return BriarLla(setpoint_wgs84, is_amsl=False)

        self._trajectory_func = trajectory_func

    def _on_trajectory_update(self, _):
        _on_trajectory_update_func_table = {
            self.State.STARTING: self._on_starting_update,
            self.State.MOVING: self._on_moving_update,
            self.State.PAUSED: self._on_paused_update,
            self.State.DONE: self._on_done_update,
        }

        func = _on_trajectory_update_func_table[self._internal_state]
        self.debouce_logger.info("WaypointTrajectory Running {} update".format(self._internal_state), 1.0, 1)
        func()

    def _on_starting_update(self):
        if self._is_starting():
            return
        self._internal_state = self.State.MOVING
        self._make_trajectory()

        # we need to run _trajectory_func for the first time so that _on_moving_update
        # can compare self._tf to time.time()
        pos: BriarLla = self._trajectory_func()
        self.setpoint_driver.lla = pos.amsl.tup

    def _is_starting(self):
        """Returns True while until we're ready to start moving.
        We can start our trajectory once:
        - we are in offboard mode
        - we know all the inputs to calculate a trajectory.

        Ruckig requires many inputs to calculate a trajectory.
        We need to know:
        - inital position
        - inital velocity
        - inital acceleration
        - final position
        - final velocity
        - final acceleration
        - speed limit
        - acceleration limits
        """

        is_done_starting = [
            self.data.is_data_available(),
            self.data.mode == "OFFBOARD",
            self._speed_limit != None,
            self._final_position != None,
        ]

        return not all(is_done_starting)

    def _on_moving_update(self):
        if time.time() > self._tf:
            self._internal_state = self.State.DONE

        pos: BriarLla = self._trajectory_func()
        self.setpoint_driver.lla = pos.amsl.tup

    def _on_paused_update(self):
        pass

    def _on_done_update(self):
        pass


class CircleTrajectory(TrajectoryGenerator):
    class State(Enum):
        STARTING = enum.auto()
        MOVING = enum.auto()
        PAUSED = enum.auto()
        DONE = enum.auto()

    def __init__(self, state: BaseState):
        super().__init__(state)
        self.data: _DroneData = _DroneData(state)

        # self.message_senders.add(RepeatTimer("trajectory_update", 1/50))
        # self.handlers.add_handler("trajectory_update", self._on_update)
        
        self.handlers.add_handler(Setpoint.TIMER_MESSAGE_NAME, self._on_update, is_active=False)


        self._internal_state = self.State.STARTING
        self._center_position: BriarLla = None
        self._sweep_angle: float = None # degrees
        self._speed_limit: float = None # meters per second

        self._t0: float = None
        self._tf: float = None # t final the time when the trajectory is done
        self._initial_position: BriarLla = None
        self._trajectory_func: TrajectoryFunction = None
        self._trajectory_duration: float = None

        self.yaw_trajectory: YawTrajectory = YawTrajectory(state, self.setpoint_driver, self.data)
    
    @property
    def yaw(self) -> str:
        """The yaw target in radians
        The yaw is rotation around the up axis, where 0 is East.
        """
        return self.yaw_trajectory.yaw
    
    @yaw.setter
    def yaw(self, yaw: float):
        self.yaw_trajectory.yaw = yaw

    def pause(self):
        self.yaw_trajectory.pause()
        if self.data.is_data_available():
            self.setpoint_driver.lla = self.data.position.amsl.tup
        else:
            self.setpoint_driver.velocity = (0, 0, 0)

        self._internal_state = self.State.PAUSED

    def resume(self):
        self._internal_state = self.State.STARTING
        self.data.reset()
        self.setpoint_driver.start()
        self.yaw_trajectory.resume()

    def fly_circle(self, center_amsl: Lla, sweep_angle: float, speed_limit: float):
        """
        center_pos alt is amsl
        sweep angle is in degrees. Posative for clockwise, negative for counter-clockwise (as seen from above)
        speed_limit is in meters per second
        """
        center_amsl = convert_Lla_to_LlaDict(center_amsl)
        self._center_position = BriarLla(center_amsl, is_amsl=True)
        self._sweep_angle = sweep_angle
        self._speed_limit = speed_limit
        self._internal_state = self.State.STARTING

    def is_done(self):
        return self._internal_state == self.State.DONE

    def _on_update(self, _):
        _on_trajectory_update_func_table = {
            self.State.STARTING: self._on_starting_update,
            self.State.MOVING: self._on_moving_update,
            self.State.PAUSED: self._on_paused_update,
            self.State.DONE: self._on_done_update,
        }

        func = _on_trajectory_update_func_table[self._internal_state]
        func()

    def _on_starting_update(self):
        if self._is_starting():
            return
        self._internal_state = self.State.MOVING
        self._make_trajectory()

        # we need to run the trajectory func for the first time to initialize self._tf
        # that way _on_moving_update can compare self._tf to time.time()
        pos: BriarLla = self._trajectory_func()
        self.setpoint_driver.lla = pos.amsl.tup

    def _is_starting(self):
        is_done_starting = [
            self.data.is_data_available(),
            self.data.mode == "OFFBOARD",
            self._center_position != None,
            self._sweep_angle != None,
            self._speed_limit != None,
        ]

        return not all(is_done_starting)

    def _make_trajectory(self):
        self._initial_position = self.data.position

        inp: InputParameter = InputParameter(1)
        inp.current_position = [0.0]
        inp.current_velocity = [0.0]
        inp.current_acceleration = [0.0]

        center = self._center_position.ellipsoid.lla
        drone_lla = self._initial_position.ellipsoid.lla
        r = CircleTrajectory.find_radius(center, drone_lla)
        sweep_angle = math.radians(self._sweep_angle)
        dist = sweep_angle * r

        inp.target_position = [dist]
        inp.target_velocity = [0.0]
        inp.target_acceleration = [0.0]

        inp.max_velocity = [self._speed_limit]

        max_horizontal_acceleration = self.constraints["MPC_ACC_HOR"]
        inp.max_acceleration = [max_horizontal_acceleration]
        inp.min_acceleration = [-1.0 * max_horizontal_acceleration]

        max_jerk = self.constraints["MPC_JERK_AUTO"]
        inp.max_jerk = [max_jerk]

        otg = Ruckig(1)
        trajectory = Trajectory(1)

        # Calculate the trajectory in an offline manner
        calc_result = otg.calculate(inp, trajectory)
        if calc_result == Result.ErrorInvalidInput:
            raise Exception('Invalid input!')

        self._trajectory_duration = trajectory.duration
        self._t0 = None
        self._tf = None

        def trajectory_func() -> BriarLla:
            if self._t0 is None:
                self._t0 = time.time()
                self._tf = self._t0 + self._trajectory_duration + TRANSITION_DELAY
                t = 0.0
            else:
                t = time.time() - self._t0

            # Then, we can calculate the kinematic state at a given time
            new_position, new_velocity, new_acceleration = trajectory.at_time(t)
            circumferential_pos = new_position[0]

            center = self._center_position.ellipsoid.lla
            start = self._initial_position.ellipsoid.lla
            setpoint_lla_ellipsoid = CircleTrajectory.find_waypoint(center, start, circumferential_pos)

            setpoint_wgs84 = convert_Lla_to_LlaDict(setpoint_lla_ellipsoid)
            return BriarLla(setpoint_wgs84, is_amsl=False)

        self._trajectory_func = trajectory_func

    @staticmethod
    def find_radius(center_position: Lla, start_position: Lla) -> float:
        """Given the center position and the start position find the radius"""
        # find the location above or below the starting point that sits on North East plane with the center_position
        ned_vec = center_position.distance_ned(start_position)
        horizontal_position = center_position.move_ned(ned_vec[0], ned_vec[1], 0.0)

        return center_position.distance(horizontal_position)

    @staticmethod
    def find_waypoint(
        center_position: Lla,
        start_position: Lla,
        circumferential_pos: float,
    ) -> Lla:
        radius = CircleTrajectory.find_radius(center_position, start_position)
        theta0 = CircleTrajectory.find_starting_theta(center_position, start_position)

        delta_theta = circumferential_pos / radius

        theta = theta0 + delta_theta

        north = radius * math.cos(theta)
        east = radius * math.sin(theta)

        horizontal_pos = center_position.move_ned(north, east, 0)
        return Lla(horizontal_pos.lat, horizontal_pos.lon, start_position.alt)

    @staticmethod
    def find_starting_theta(center_position: Lla, start_position: Lla) -> float:
        """Given center pos and starting pos, find starting angle.
        This is the same as the compass heading that points from center position to starting position
        """

        ned_vec = center_position.distance_ned(start_position)
        n, e = ned_vec[0], ned_vec[1]

        return math.atan2(e, n)

    def _on_moving_update(self):
        if time.time() > self._tf:
            self._internal_state = self.State.DONE

        pos: BriarLla = self._trajectory_func()
        self.setpoint_driver.lla = pos.amsl.tup

    def _on_paused_update(self):
        pass

    def _on_done_update(self):
        pass


class YawTrajectory(TrajectoryGenerator):

    BYPASS_FILTER = math.radians(10.0)

    THRESHOLD_ANGLE = math.radians(45.0)

    class State(Enum):
        STARTING = enum.auto()
        HOLDING = enum.auto()
        MOVING = enum.auto()
        PAUSED = enum.auto()

    def __init__(self, state: BaseState, setpoint_driver: Setpoint = None, drone_data: _DroneData = None):
        """
        A trajectory generator that will yaw the drone.
        The yaw trajectory can reuse an existing setpoint_driver. This is useful if you want to
        move the drone while yawing it. if you do not provide a setpoint_driver, then the yaw
        trajectory will create one and start/stop it.

        Args:
            setpoint_driver (Setpoint, optional): The setpoint driver to use. Defaults to None.
            drone_data (_DroneData, optional): The drone data to use. Defaults to None.
        """
        super().__init__(state=state, setpoint_driver=setpoint_driver)

        # this flag is used to determine if we should start/stop the setpoint driver
        # We must play nice with other trajectory generators
        # the yaw trajectory can be used in conjunction with other trajectory generators
        # so we need to be able to reuse an existing setpoint driver
        # if a setpoint_driver is not provided, TrajectoryGenerator.__init__()
        # will create one. In case it was just created, then we must start it, and stop it.
        # However, if a setpoint_driver is provided, we will not start or stop it. Presumably, another
        # trajectory generator created it, and it will start/stop it.
        self._this_owns_setpoint_driver = setpoint_driver is None


        if drone_data is None:
            drone_data = _DroneData(state)
        self._drone_data = drone_data
        self.drone: MAVROSDrone = state.drone

        # self.message_senders.add(RepeatTimer("yaw_trajectory_update", 1/50))
        # self.handlers.add_handler("yaw_trajectory_update", self.update)
        self.handlers.add_handler(Setpoint.TIMER_MESSAGE_NAME, self.update, is_active=False)


        self._internal_state = self.State.STARTING
        self.func_table = {
            self.State.STARTING: self._on_starting_update,
            self.State.HOLDING: self._on_holding_update,
            self.State.MOVING: self._on_moving_update,
            self.State.PAUSED: self._on_paused_update,
        }

        self._yaw_target: float = None
        self._yaw_initial: float = None
        self._yaw_delta: float = None

        self._spin_axis = None

        self._t_delta = None
        self._t_start = None
        self._t_final = None

        self._trajectory_func = None
    
    @property
    def yaw(self) -> str:
        """The yaw target in radians
        The yaw is rotation around the up axis, where 0 is East.
        """
        return self._yaw_target
    
    @yaw.setter
    def yaw(self, yaw: float):
        # TODO: consider using a filter to skip trajectories when the yaw is close to the target
        
        # aircraft_yaw = _find_yaw(self._drone_data.attitude)
        # angular_dist = _find_angular_distance(aircraft_yaw, yaw)
        # # if we're only turning a small amount then just bypass the trajectory
        # if angular_dist < YawTrajectory.BYPASS_FILTER:
        #     self.setpoint_driver.yaw = yaw
        #     self._internal_state = self.State.HOLDING
        #     return
        if self._internal_state == self.State.MOVING:
            return
        self._yaw_target = yaw
        self._internal_state = self.State.STARTING
    
    def start(self):
        if self._this_owns_setpoint_driver:
            self.setpoint_driver.start()

    def stop(self):
        if self._this_owns_setpoint_driver:
            self.setpoint_driver.stop()
    
    def update(self, _):
        # look up the function to call based on the internal state
        func = self.func_table[self._internal_state]
        # call the funciton
        func()

    def make_trajectory(self):
        self._yaw_initial = self._find_inital_yaw()
        self._yaw_delta = _find_angular_distance(self._yaw_initial, self._yaw_target)
        self._spin_axis = _find_rotation_axis(self._yaw_initial, self._yaw_target)

        # now we need a 1-dimensional ruckig trajectory. This will be used to generate the yaw setpoints.

        # NOTE the trajectory will be in degrees, not radians. This is because the PX4 constraints
        # are in degrees. Also the trajectory will go from ZERO to yaw_delta. This way we can avoid the
        # discontinuity at +/- pi degrees. The angular distance between -179 and 179 is 2 degrees,
        # not 358 degrees and we want to spin as little as possible.

        inp: InputParameter = InputParameter(1)
        inp.current_position = [0.0]
        inp.current_velocity = [0.0]
        inp.current_acceleration = [0.0]

        inp.target_position = [math.degrees(self._yaw_delta)]
        inp.target_velocity = [0.0]
        inp.target_acceleration = [0.0]

        max_angular_velocity = self.constraints["MPC_YAWRAUTO_MAX"]
        inp.max_velocity = [max_angular_velocity]
        # to finguire out the max acceleration, we can look at the defaults for linear motion
        # In PX4's mission mode:
        #  - the default max linear velocity is 5 m/s
        #  - the default max linear acceleration is 3 m/s^2
        #  - the default max linear jerk is 4 m/s^3
        
        # So I will use this as a guide. I will assume it takes the same amount of time to accelerate
        # to max rotation speed. For linear acceleration 5 = 3 * t (5 = velocity, 3 = acceleration, t = time)
        # therefore t = 5/3 = 1.67 seconds.
        # 
        # So I will pick a max angular acceleration so that it take 1.67 seconds to reach max rotation speed.

        max_angular_acceleration = max_angular_velocity / 1.67
        inp.max_acceleration = [max_angular_acceleration]

        # to find max angular jerk, we can use the same logic as above. We will assume it takes the same amount of time
        # to reach max angular acceleration. For linear jerk 3 = 4 * t (3 = acceleration, 4 = jerk, t = time)
        # Therefore t = 3/4 = 0.75 seconds.
        # So it should take 0.75 seconds to reach max angular acceleration.
        max_angular_jerk = max_angular_acceleration / 0.75
        inp.max_jerk = [max_angular_jerk]

        otg = Ruckig(1)
        trajectory = Trajectory(1)

        # Calculate the trajectory in an offline manner
        calc_result = otg.calculate(inp, trajectory)
        if calc_result == Result.ErrorInvalidInput:
            raise Exception('Invalid input!')

        
        self._t_delta = trajectory.duration
        self._t_start = None
        self._t_final = None

        yaw_initial_deg = math.degrees(self._yaw_initial)

        def trajectory_func() -> BriarLla:
            if self._t_start is None:
                self._t_start = time.time()
                # self._t_final = self._t_start + self._t_delta + TRANSITION_DELAY
                self._t_final = self._t_start + self._t_delta
                t = 0.0
            else:
                t = time.time() - self._t_start

            # Then, we can calculate the angular displacement given time
            new_position, new_velocity, new_acceleration = trajectory.at_time(t)
            angular_displacement = new_position[0]

            # now we can calculate the yaw setpoint
            # by rotation angular_displacement around the spin axis
            # rotating around the up axis is the same as adding
            # angular_displacement to _yaw_initial
            # and rotating around the down axis is hte same as subtracting the
            # angular displacement from _yaw_initial
            # self._spin_axis is +1 for up and -1 for down
            yaw_setpoint = yaw_initial_deg + angular_displacement * self._spin_axis

            # now we need to address the case where the yaw_setpoint is outside of the range [-180, 180]
            if yaw_setpoint > 180:
                yaw_setpoint -= 360
            elif yaw_setpoint < -180:
                yaw_setpoint += 360
            

            # now we need to convert the yaw setpoint to radians
            yaw_setpoint = math.radians(yaw_setpoint)

            return yaw_setpoint

        self._trajectory_func = trajectory_func
    
    def _find_inital_yaw(self) -> float:
        """Find the initial yaw for the trajectory
        if we're within a small angle of the previous yaw, then start at the previous yaw
        otherwise, start at the current yaw.
        """
        _, _, yaw_setpoint = self.drone.setpoint
        
        yaw_sensor = _find_yaw(self._drone_data.attitude)

        if yaw_setpoint is None:
            return yaw_sensor

        # if abs(_find_angular_distance(sensed_yaw, previous_yaw)) < YawTrajectory.THRESHOLD_ANGLE:
        if _find_angular_distance(yaw_sensor, yaw_setpoint) < YawTrajectory.THRESHOLD_ANGLE:
            return yaw_setpoint
        
        return yaw_sensor

    def _on_starting_update(self):
        if not self._drone_data.is_data_available():
            return
        
        if self._yaw_target is None:
            self._internal_state = self.State.HOLDING
            return

        self._internal_state = self.State.MOVING
        self.make_trajectory()
        self.setpoint_driver.yaw = self._trajectory_func()

    def _on_holding_update(self):
        self.setpoint_driver.yaw = None


    def _on_moving_update(self):
        if time.time() > self._t_final:
            self._internal_state = self.State.HOLDING
        self.setpoint_driver.yaw = self._trajectory_func()

    def _on_paused_update(self):
        pass

    def pause(self):
        self._internal_state = self.State.PAUSED
        self.setpoint_driver.yaw = None
        return 
    
    def resume(self):
        if self._this_owns_setpoint_driver:
            self.setpoint_driver.start()
        self._drone_data.reset()
        self._internal_state = self.State.STARTING


class HoldingTrajectory:
    def __init__(self, state: BaseState, setpoint_driver: Setpoint = None):
        self.dr_state = state

        if setpoint_driver is None:
            setpoint_driver = Setpoint(self.dr_state)
        self.setpoint_driver = setpoint_driver

    def start(self):
        self.setpoint_driver.start(restore_previous_setpoint=True)

    def stop(self):
        self.setpoint_driver.stop()

    def pause(self):
        pass

    def resume(self):
        pass