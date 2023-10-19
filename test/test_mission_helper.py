import unittest
import json
from dr_onboard_autonomy.mission_helper import transform_relative_altitude

## Test cases
class TestTransformRelativeAltitude(unittest.TestCase):
    def test_transform_relative_altitude_no_relative_alts_found(self):
        take_off_mission = """
            {
                "states": [
                    {
                        "name": "MISSION_PREPARATION",
                        "transitions": [
                            {
                                "target": "OnGround",
                                "condition": "MissionConfigured"
                            }
                        ]
                    },
                    {
                        "name": "Takeoff",
                        "args": {
                        },
                        "transitions": [
                            {
                                "target": "BriarHover",
                                "condition": "succeeded_takeoff"
                            },
                            {
                                "target": "Land",
                                "condition": "failed_takeoff"
                            }
                        ]
                    },
                    {
                        "name": "BriarHover",
                        "args": {
                            "hover_time": 60,
                            "stare_position": {
                                "latitude": 41.606881978397986, 
                                "longitude": -86.3560259537521,
                                "altitude": 229.02
                            }
                        },
                        "transitions": [
                            {
                                "target": "Land",
                                "condition": "succeeded_hover"
                            }
                        ]
                    },
                    {
                        "name": "Land",
                        "transitions": [
                            {
                                "target": "Disarm",
                                "condition": "succeeded_land"
                            }
                        ]
                    },
                    {
                        "name": "Disarm",
                        "transitions": [
                            {
                                "target": "mission_completed",
                                "condition": "succeeded_disarm"
                            }
                        ]
                    }
                ]
            }
        """
        mission = json.loads(take_off_mission)
        altitude = 0
        transformed_mission = transform_relative_altitude(mission, altitude)
        self.assertDictEqual(mission, transformed_mission)
    
    def test_transform_relative_altitude_no_relative_alts_and_no_takeoff_args(self):
        take_off_mission = """
            {
                "states": [
                    {
                        "name": "MISSION_PREPARATION",
                        "transitions": [
                            {
                                "target": "OnGround",
                                "condition": "MissionConfigured"
                            }
                        ]
                    },
                    {
                        "name": "Takeoff",
                        "transitions": [
                            {
                                "target": "BriarHover",
                                "condition": "succeeded_takeoff"
                            },
                            {
                                "target": "Land",
                                "condition": "failed_takeoff"
                            }
                        ]
                    },
                    {
                        "name": "BriarHover",
                        "args": {
                            "hover_time": 60,
                            "stare_position": {
                                "latitude": 41.606881978397986, 
                                "longitude": -86.3560259537521,
                                "altitude": 229.02
                            }
                        },
                        "transitions": [
                            {
                                "target": "Land",
                                "condition": "succeeded_hover"
                            }
                        ]
                    },
                    {
                        "name": "Land",
                        "transitions": [
                            {
                                "target": "Disarm",
                                "condition": "succeeded_land"
                            }
                        ]
                    },
                    {
                        "name": "Disarm",
                        "transitions": [
                            {
                                "target": "mission_completed",
                                "condition": "succeeded_disarm"
                            }
                        ]
                    }
                ]
            }
        """
        mission = json.loads(take_off_mission)
        altitude = 0
        transformed_mission = transform_relative_altitude(mission, altitude)
        self.assertDictEqual(mission, transformed_mission)

    def test_transform_relative_altitude_no_relative_alts_and_takeoff_alt(self):
        take_off_mission = """
            {
                "states": [
                    {
                        "name": "MISSION_PREPARATION",
                        "transitions": [
                            {
                                "target": "OnGround",
                                "condition": "MissionConfigured"
                            }
                        ]
                    },
                    {
                        "name": "Takeoff",
                        "args": {
                            "altitude": 10.0
                        },
                        "transitions": [
                            {
                                "target": "BriarHover",
                                "condition": "succeeded_takeoff"
                            },
                            {
                                "target": "Land",
                                "condition": "failed_takeoff"
                            }
                        ]
                    },
                    {
                        "name": "BriarHover",
                        "args": {
                            "hover_time": 60,
                            "stare_position": {
                                "latitude": 41.606881978397986, 
                                "longitude": -86.3560259537521,
                                "altitude": 229.02
                            }
                        },
                        "transitions": [
                            {
                                "target": "Land",
                                "condition": "succeeded_hover"
                            }
                        ]
                    },
                    {
                        "name": "Land",
                        "transitions": [
                            {
                                "target": "Disarm",
                                "condition": "succeeded_land"
                            }
                        ]
                    },
                    {
                        "name": "Disarm",
                        "transitions": [
                            {
                                "target": "mission_completed",
                                "condition": "succeeded_disarm"
                            }
                        ]
                    }
                ]
            }
        """
        mission = json.loads(take_off_mission)
        altitude = 0
        transformed_mission = transform_relative_altitude(mission, altitude)
        self.assertDictEqual(mission, transformed_mission)

    def test_transform_relative_altitude_with_relative_alt_in_takeoff_state(self):
        """Test that the relative altitude in the Takeoff state is transformed

        The Takeoff state is a special case, because it has a `altitude` arg that
        is already interpreted as a relative altitude. So we need to replace
        property without adding the home altitude to it.
        """
        take_off_mission = """
            {
                "states": [
                    {
                        "name": "MISSION_PREPARATION",
                        "transitions": [
                            {
                                "target": "OnGround",
                                "condition": "MissionConfigured"
                            }
                        ]
                    },
                    {
                        "name": "Takeoff",
                        "args": {
                            "relative_altitude": 10.0
                        },
                        "transitions": [
                            {
                                "target": "BriarHover",
                                "condition": "succeeded_takeoff"
                            },
                            {
                                "target": "Land",
                                "condition": "failed_takeoff"
                            }
                        ]
                    },
                    {
                        "name": "BriarHover",
                        "args": {
                            "hover_time": 60,
                            "stare_position": {
                                "latitude": 41.606881978397986, 
                                "longitude": -86.3560259537521,
                                "altitude": 229.02
                            }
                        },
                        "transitions": [
                            {
                                "target": "Land",
                                "condition": "succeeded_hover"
                            }
                        ]
                    },
                    {
                        "name": "Land",
                        "transitions": [
                            {
                                "target": "Disarm",
                                "condition": "succeeded_land"
                            }
                        ]
                    },
                    {
                        "name": "Disarm",
                        "transitions": [
                            {
                                "target": "mission_completed",
                                "condition": "succeeded_disarm"
                            }
                        ]
                    }
                ]
            }
        """
        mission = json.loads(take_off_mission)
        altitude = 230
        transformed_mission = transform_relative_altitude(mission, altitude)
        expected_mission = """
            {
                "states": [
                    {
                        "name": "MISSION_PREPARATION",
                        "transitions": [
                            {
                                "target": "OnGround",
                                "condition": "MissionConfigured"
                            }
                        ]
                    },
                    {
                        "name": "Takeoff",
                        "args": {
                            "altitude": 10.0
                        },
                        "transitions": [
                            {
                                "target": "BriarHover",
                                "condition": "succeeded_takeoff"
                            },
                            {
                                "target": "Land",
                                "condition": "failed_takeoff"
                            }
                        ]
                    },
                    {
                        "name": "BriarHover",
                        "args": {
                            "hover_time": 60,
                            "stare_position": {
                                "latitude": 41.606881978397986, 
                                "longitude": -86.3560259537521,
                                "altitude": 229.02
                            }
                        },
                        "transitions": [
                            {
                                "target": "Land",
                                "condition": "succeeded_hover"
                            }
                        ]
                    },
                    {
                        "name": "Land",
                        "transitions": [
                            {
                                "target": "Disarm",
                                "condition": "succeeded_land"
                            }
                        ]
                    },
                    {
                        "name": "Disarm",
                        "transitions": [
                            {
                                "target": "mission_completed",
                                "condition": "succeeded_disarm"
                            }
                        ]
                    }
                ]
            }
        """
        expected_mission = json.loads(expected_mission)
        self.assertDictEqual(expected_mission, transformed_mission)

    def test_transform_relative_altitude_with_relative_alt_in_stare_position(self):
        original_mission = """
            {
                "states": [
                    {
                        "name": "MISSION_PREPARATION",
                        "transitions": [
                            {
                                "target": "OnGround",
                                "condition": "MissionConfigured"
                            }
                        ]
                    },
                    {
                        "name": "Takeoff",
                        "args": {
                        },
                        "transitions": [
                            {
                                "target": "BriarHover",
                                "condition": "succeeded_takeoff"
                            },
                            {
                                "target": "Land",
                                "condition": "failed_takeoff"
                            }
                        ]
                    },
                    {
                        "name": "BriarHover",
                        "args": {
                            "hover_time": 60,
                            "stare_position": {
                                "latitude": 41.606881978397986, 
                                "longitude": -86.3560259537521,
                                "relative_altitude": 10.0
                            }
                        },
                        "transitions": [
                            {
                                "target": "Land",
                                "condition": "succeeded_hover"
                            }
                        ]
                    },
                    {
                        "name": "Land",
                        "transitions": [
                            {
                                "target": "Disarm",
                                "condition": "succeeded_land"
                            }
                        ]
                    },
                    {
                        "name": "Disarm",
                        "transitions": [
                            {
                                "target": "mission_completed",
                                "condition": "succeeded_disarm"
                            }
                        ]
                    }
                ]
            }
        """
        original_mission = json.loads(original_mission)
        altitude = 100.0
        transformed_mission = transform_relative_altitude(original_mission, altitude)
        expected_mission = """
            {
                "states": [
                    {
                        "name": "MISSION_PREPARATION",
                        "transitions": [
                            {
                                "target": "OnGround",
                                "condition": "MissionConfigured"
                            }
                        ]
                    },
                    {
                        "name": "Takeoff",
                        "args": {
                        },
                        "transitions": [
                            {
                                "target": "BriarHover",
                                "condition": "succeeded_takeoff"
                            },
                            {
                                "target": "Land",
                                "condition": "failed_takeoff"
                            }
                        ]
                    },
                    {
                        "name": "BriarHover",
                        "args": {
                            "hover_time": 60,
                            "stare_position": {
                                "latitude": 41.606881978397986, 
                                "longitude": -86.3560259537521,
                                "altitude": 110.0
                            }
                        },
                        "transitions": [
                            {
                                "target": "Land",
                                "condition": "succeeded_hover"
                            }
                        ]
                    },
                    {
                        "name": "Land",
                        "transitions": [
                            {
                                "target": "Disarm",
                                "condition": "succeeded_land"
                            }
                        ]
                    },
                    {
                        "name": "Disarm",
                        "transitions": [
                            {
                                "target": "mission_completed",
                                "condition": "succeeded_disarm"
                            }
                        ]
                    }
                ]
            }
        """
        expected_mission = json.loads(expected_mission)
        self.assertDictEqual(expected_mission, transformed_mission)
    
    def test_transform_relative_altitude_with_many_relative_alts(self):
        altitude = 230.0
        original_mission = """
            {
                "states": [
                    {
                        "name": "Takeoff",
                        "transitions": [
                            {
                                "target": "PhasedCircle",
                                "condition": "succeeded_takeoff"
                            },
                            {
                                "target": "Land",
                                "condition": "failed_takeoff"
                            }
                        ]
                    },
                    {
                        "name": "PhasedCircle",
                        "args": {
                            "center_position":{
                                "latitude": 41.60669879284219, 
                                "longitude": -86.35549049728084,
                                "relative_altitude": 1.0
                            },
                            "stare_position": {
                                "latitude": 41.606881978397986, 
                                "longitude": -86.3560259537521,
                                "relative_altitude": 0.0
                            },
                            "pitch": 33.75,
                            "distance": 36.06,
                            "starting_angle": 90,
                            "total_sweep_angle": 135,
                            "speed": 2.5,
                            "cruising_altitude": 15,
                            "number_arcs": 1,
                            "pause_time": 5
                        },
                        "transitions": [
                            {
                                "target": "BriarWaypoint",
                                "condition": "succeeded_circle"
                            }
                        ]
                    },
                    {
                        "name": "BriarWaypoint",
                        "args": {
                            "waypoint": {
                                "latitude": 41.606695509416944,
                                "longitude": -86.35550466673673,
                                "relative_altitude": 10.0
                            },
                            "stare_position": {
                                "latitude": 41.6068925536827,
                                "longitude": -86.35607195393388,
                                "relative_altitude": 0.0
                            },
                            "speed": 2.5
                        },
                        "transitions": [
                            {
                                "target": "Land",
                                "condition": "succeeded_waypoints"
                            }
                        ]
                    },
                    {
                        "name": "Land",
                        "transitions": [
                            {
                                "target": "Disarm",
                                "condition": "succeeded_land"
                            }
                        ]
                    },
                    {
                        "name": "Disarm",
                        "transitions": [
                            {
                                "target": "mission_completed",
                                "condition": "succeeded_disarm"
                            }
                        ]
                    }
                ]
            }
        """
        original_mission = json.loads(original_mission)
        transformed_mission = transform_relative_altitude(original_mission, altitude)
        expected_mission = """
            {
                "states": [
                    {
                        "name": "Takeoff",
                        "transitions": [
                            {
                                "target": "PhasedCircle",
                                "condition": "succeeded_takeoff"
                            },
                            {
                                "target": "Land",
                                "condition": "failed_takeoff"
                            }
                        ]
                    },
                    {
                        "name": "PhasedCircle",
                        "args": {
                            "center_position":{
                                "latitude": 41.60669879284219, 
                                "longitude": -86.35549049728084,
                                "altitude": 231.0
                            },
                            "stare_position": {
                                "latitude": 41.606881978397986, 
                                "longitude": -86.3560259537521,
                                "altitude": 230.0
                            },
                            "pitch": 33.75,
                            "distance": 36.06,
                            "starting_angle": 90,
                            "total_sweep_angle": 135,
                            "speed": 2.5,
                            "cruising_altitude": 15,
                            "number_arcs": 1,
                            "pause_time": 5
                        },
                        "transitions": [
                            {
                                "target": "BriarWaypoint",
                                "condition": "succeeded_circle"
                            }
                        ]
                    },
                    {
                        "name": "BriarWaypoint",
                        "args": {
                            "waypoint": {
                                "latitude": 41.606695509416944,
                                "longitude": -86.35550466673673,
                                "altitude": 240.0
                            },
                            "stare_position": {
                                "latitude": 41.6068925536827,
                                "longitude": -86.35607195393388,
                                "altitude": 230.0
                            },
                            "speed": 2.5
                        },
                        "transitions": [
                            {
                                "target": "Land",
                                "condition": "succeeded_waypoints"
                            }
                        ]
                    },
                    {
                        "name": "Land",
                        "transitions": [
                            {
                                "target": "Disarm",
                                "condition": "succeeded_land"
                            }
                        ]
                    },
                    {
                        "name": "Disarm",
                        "transitions": [
                            {
                                "target": "mission_completed",
                                "condition": "succeeded_disarm"
                            }
                        ]
                    }
                ]
            }
        """
        expected_mission = json.loads(expected_mission)
        self.assertDictEqual(expected_mission, transformed_mission)