from ripple_down_rules.datastructures.case import Case
from types import NoneType
from typing_extensions import Dict, Optional, Union
from ripple_down_rules import *


def conditions_99648325048930205296105700669502885039(case) -> bool:
    def conditions_for_get_rotor_speed_from_wind_speed(wind_speed: float, **kwargs) -> bool:
        """Get conditions on whether it's possible to conclude a value for get_rotor_speed_from_wind_speed.output_  of type ."""
        return True
    return conditions_for_get_rotor_speed_from_wind_speed(**case)


def conclusion_99648325048930205296105700669502885039(case) -> float:
    def get_rotor_speed_from_wind_speed(wind_speed: float, **kwargs) -> float:
        """Get possible value(s) for get_rotor_speed_from_wind_speed.output_  of type ."""
        if wind_speed >= 13:
            return 11.1
        elif wind_speed < 2:
            return 0.0
        else:
            return 10.0
    return get_rotor_speed_from_wind_speed(**case)


