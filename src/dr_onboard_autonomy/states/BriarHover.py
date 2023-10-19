from typing import Optional

import rospy

from dr_onboard_autonomy.briar_helpers import (
    BriarLla,
    LlaDict,
    convert_LlaDict_to_tuple,
    amsl_to_ellipsoid,
    ellipsoid_to_amsl
)
from dr_onboard_autonomy.message_senders import RepeatTimer
from dr_onboard_autonomy.states.components import Gimbal, UpdateStarePosition
from dr_onboard_autonomy.states.components.trajectory import YawTrajectory, HoldingTrajectory

from .BaseState import BaseState


class BriarHover(BaseState):
    def __init__(self, hover_time:float=10, stare_position:Optional[LlaDict]=None, **kwargs):
        """
        Hovers the drone while pointing the camera.

        Args:
            hover_time: float in seconds. How long the drone will hover.
            stare_position: where the camera looks. Alt is AMSL.
        """
        if "outcomes" in kwargs:
            outcome_set = set(kwargs["outcomes"])
        else:
            outcome_set = set()
        for other_outcome in ["succeeded_hover", "error", "human_control", "abort", "rtl"]:
            outcome_set.add(other_outcome)
        kwargs["outcomes"] = list(outcome_set)

        TrajectoryClass = YawTrajectory
        if stare_position is None:
             TrajectoryClass = HoldingTrajectory

        super().__init__(trajectory_class=TrajectoryClass, **kwargs)

        self.hover_time = hover_time
        self.stare_position = None

        if stare_position is not None:
            stare_position: BriarLla = BriarLla(stare_position, is_amsl=True)
            self.stare_position = stare_position.ellipsoid.tup
            self.gimbal_driver = Gimbal(self)
            self.stare_position_updater = UpdateStarePosition(self, self.gimbal_driver)

        self.message_senders.add(RepeatTimer("hover_timer", hover_time))
        self.handlers.add_handler("hover_timer", self.on_hover_timer)

    def on_entry(self, userdata):
        self.trajectory.start()
        if self.stare_position is not None:
            self.gimbal_driver.start()
            self.gimbal_driver.stare_position = self.stare_position

    def on_hover_timer(self, message):
        return "succeeded_hover"
