
import copy
_RELATIVE_ALTITUDE_PROP_NAME = "relative_altitude"
_ALTITUDE_PROP_NAME = "altitude"

def transform_relative_altitude(original_mission: dict, altitude: float):
    """find every object with a `relative_altitude` property, and replace it
    with an `altitude` property
    """
    mission = copy.deepcopy(original_mission)
    # the Takeoff state is a special case, because it has a `altitude` arg that
    # is a relative altitude. So we need to replace that before we do the rest
    # of the mission in this case we can just need to add an `altitude` arg with
    # the value of the `relative_altitude`. Then we can delete the `relative_altitude`
    for state in mission["states"]:
        if _is_takeoff_state(state):
            if "args" not in state:
                continue
            if _RELATIVE_ALTITUDE_PROP_NAME in state["args"].keys():
                state["args"][_ALTITUDE_PROP_NAME] = state["args"][_RELATIVE_ALTITUDE_PROP_NAME]
                del state["args"][_RELATIVE_ALTITUDE_PROP_NAME]
    
    def replace_rel_alt(obj:dict):
        if _RELATIVE_ALTITUDE_PROP_NAME in obj.keys():
            obj[_ALTITUDE_PROP_NAME] = obj[_RELATIVE_ALTITUDE_PROP_NAME] + altitude
            del obj[_RELATIVE_ALTITUDE_PROP_NAME]
        for prop in obj.keys():
            if isinstance(obj[prop], dict):
                replace_rel_alt(obj[prop])
            elif isinstance(obj[prop], list):
                for item in obj[prop]:
                    if isinstance(item, dict):
                        replace_rel_alt(item)

    replace_rel_alt(mission)
    return mission

def _is_takeoff_state(state: dict) -> bool:
    if "class" in state and state["class"] == "Takeoff":
        return True
    if "class" not in state:
        if "name" in state and state["name"] == "Takeoff":
            return True
    return False