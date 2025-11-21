from typing_extensions import Dict, Optional, Union
from types import NoneType
from ripple_down_rules.datastructures.case import Case, create_case
from .get_rotor_speed_from_wind_speed_output__scrdr_defs import *


attribute_name = 'output_'
conclusion_type = (float,)
mutually_exclusive = True
name = 'output_'
case_type = Dict
case_name = 'get_rotor_speed_from_wind_speed'


def classify(case: Dict, **kwargs) -> Optional[float]:
    if not isinstance(case, Case):
        case = create_case(case, max_recursion_idx=3)

    if conditions_99648325048930205296105700669502885039(case):
        return conclusion_99648325048930205296105700669502885039(case)
    else:
        return None
