import threading

import rclpy
from krrood.entity_query_language.symbol_graph import SymbolGraph

from semantic_digital_twin.adapters.viz_marker import VizMarkerPublisher
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.spatial_types.spatial_types import TransformationMatrix
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.geometry import Box, Scale, Color
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body

from src.semantic_annotations.semantic_annotations import Tower

SymbolGraph().clear()
SymbolGraph()

def loading_environment():
    world = World()

    white = Color(1, 1, 1)
    red = Color(1, 1, 0)
    black = Color(0, 0, 0)
    gray = Color(0.74, 0.74, 0.74)
    wood = Color(1, 0.827, 0.6078)

    root_origin = TransformationMatrix()
    root = Body(name=PrefixedName("root"))
    # All the Walls:
    sWall1 = Box(scale=Scale(0.05, 1.00, 3.00), color=gray)
    visual = ShapeCollection([sWall1])
    collision = ShapeCollection([sWall1])
    sWall1_body = Body(name=PrefixedName("sWall1_body"), collision=collision, visual=visual)



    root_C_sWall1 = FixedConnection(parent=root, child=sWall1_body,
                                    parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=0, y=-2.01,
                                                                                                     z=1.50))

    with world.modify_world():
        world.add_body(root)
        world.add_body(sWall1_body)
        world.add_connection(root_C_sWall1)

        return world


def published(world: World):
    rclpy.init()
    node = rclpy.create_node("semantic_digital_twin")
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    viz = VizMarkerPublisher(world=world, node=node)


def test_loading_3d_model_environment():
    world = loading_environment()
    published(world)