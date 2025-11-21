from __future__ import annotations

from dataclasses import dataclass, field

from krrood.entity_query_language.entity import let, an, entity, inference
from ripple_down_rules import CaseQuery, GeneralRDR, RDRDecorator
from semantic_digital_twin.world_description.geometry import Box
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import SemanticAnnotation, Body



@dataclass(eq=False)
class Tower(SemanticAnnotation):
    body: Body


@dataclass(eq=False)
class RotorBlade(SemanticAnnotation):
    body: Body
    blade_angle: float = field(default=0.0)


@dataclass(eq=False)
class TowerBase(SemanticAnnotation):
    body: Body


@dataclass(eq=False)
class Nacelle(SemanticAnnotation):
    body: Body


@dataclass(eq=False)
class Hub(SemanticAnnotation):
    body: Body


@dataclass(eq=False)
class RotorSpeedSensor(SemanticAnnotation):
    measures: list[float] = field(default_factory=list)

@dataclass(eq=False)
class RotorSpeed(SemanticAnnotation):
    has_rotor_speed: float = field(default=0.0)
    measured_by: RotorSpeedSensor = field(default_factory=RotorSpeedSensor)

@dataclass(eq=False)
class Rotor(SemanticAnnotation):
    body: Body
    has_expected_rotor_speed: float = field(default=0.0)
    has_sensor: RotorSpeedSensor = field(default_factory=RotorSpeedSensor)


@dataclass(eq=False)
class WindSpeedSensor:
    measures: WindSpeed = field(default_factory=lambda: WindSpeed())

@dataclass(eq=False)
class WindSpeed(SemanticAnnotation):
    has_wind_speed: float = field(default=0.0)
    measured_by: WindSpeedSensor = field(default_factory=WindSpeedSensor)

rotor_speed_rdr = RDRDecorator("./", (float,), True, fit=True, update_existing_rules=False)

@rotor_speed_rdr.decorator
def get_rotor_speed_from_wind_speed(wind_speed: float) -> float:
    pass