from dataclasses import dataclass, field
from semantic_digital_twin.world_description.world_entity import SemanticAnnotation, Body



@dataclass(eq=False)
class Tower(SemanticAnnotation):
    body: Body


@dataclass(eq=False)
class RotorBlades(SemanticAnnotation):
    body: Body


@dataclass(eq=False)
class TowerBase(SemanticAnnotation):
    body: Body


@dataclass(eq=False)
class Nacelle(SemanticAnnotation):
    body: Body


@dataclass(eq=False)
class Hub(SemanticAnnotation):
    body: Body
