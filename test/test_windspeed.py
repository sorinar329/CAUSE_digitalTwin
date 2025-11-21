from krrood.entity_query_language.entity import an, entity
from krrood.entity_query_language.symbol_graph import SymbolGraph
from ripple_down_rules import CaseQuery, GeneralRDR
from semantic_digital_twin.world_description.geometry import Box
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body

from semantic_annotations.semantic_annotations import RotorSpeedSensor, WindSpeedSensor, WindSpeed, Rotor, \
    get_rotor_speed_from_wind_speed


def test_windspeed():
    sensor = WindSpeedSensor()
    wind_speed = WindSpeed()
    wind_speed.has_wind_speed = 10.0
    sensor.measures = wind_speed


    assert sensor.measures.has_wind_speed == 10.0


def test_fit_rule_for_rotor_speed():
    from semantic_digital_twin.world import World
    get_rotor_speed_from_wind_speed(15)