import time
import random
from dr_onboard_autonomy.message_senders import RepeatTimer

from .BaseState import BaseState


class Tracking(BaseState):
    def __init__(self, **kwargs):
        kwargs["outcomes"] = ["succeeded_tracking"]
        super().__init__(**kwargs)
        self.message_senders.add(RepeatTimer("transition_time", 10))
        self.handlers.add_handler("transition_time", self.tx_handler)

    def tx_handler(self, msg):
        return "succeeded_tracking"