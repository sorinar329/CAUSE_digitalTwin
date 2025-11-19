"""
3D model of Wind Turbine using Semantic Digital Twin framework.
"""

import threading, time
import rclpy
from semantic_digital_twin.adapters.viz_marker import VizMarkerPublisher
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.spatial_types.spatial_types import TransformationMatrix
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.geometry import Box, Scale, Color
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body

from semantic_annotations.semantic_annotations import Tower, TowerBase, Nacelle, RotorBlades, Hub


def main():
    """
    Entry point for constructing and visualizing the Wind Turbine.

    This function programmatically defines a hierarchical wind turbine model consisting of:
        - Tower base
        - Tower shaft
        - Nacelle
        - Three rotor blades
        - Central hub

    Each component is represented as a Body with associated geometric primitives, semantic
    annotations, and parent–child spatial relationships expressed through FixedConnections.

    The assembled world model is published to ROS 2 using visualization markers so that the
    full turbine structure can be inspected in RViz2.

    Visualization:
        Launch RViz2 and add a "Marker" display subscribed to:
            /viz_marker

    Purpose:
        Enables testing, debugging, and validation of semantic digital twin workflows,
        world modeling pipelines, and visualization toolchains for robotics and simulation.
    """


    world = World()

    # ----- Define colors -----
    red = Color(1, 0, 0)
    white = Color(1, 1, 1)
    blue = Color(0, 0, 1)
    brown = Color(1, 0.5, 0.25)
    green = Color(0, 1, 0)

    # ----- Root Body -----
    root = Body(name=PrefixedName("root"))

    # =====================================================================
    # Tower Base
    # =====================================================================
    body2 = Box(scale=Scale(1.0, 1.0, 0.2), color=brown)
    visual = ShapeCollection([body2])
    collision = ShapeCollection([body2])
    base_body = Body(name=PrefixedName("tower_base"), visual=visual, collision=collision)

    root_C_body2 = FixedConnection(
        parent=root,
        child=base_body,
        parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=0, y=-0, z=0.2)
    )
    base = TowerBase(base_body)

    # =====================================================================
    # Tower
    # =====================================================================
    body1 = Box(scale=Scale(0.2, 0.3, 3.0), color=red)
    visual = ShapeCollection([body1])
    collision = ShapeCollection([body1])
    tower_body = Body(name=PrefixedName("tower"), visual=visual, collision=collision)

    root_C_body1 = FixedConnection(
        parent=base_body,
        child=tower_body,
        parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=0, y=-0, z=1.6)
    )
    tower = Tower(tower_body)

    # =====================================================================
    # Nacelle
    # =====================================================================
    body3 = Box(scale=Scale(0.4, 0.3, 0.2), color=green)
    visual = ShapeCollection([body3])
    collision = ShapeCollection([body3])
    nacelle_body = Body(name=PrefixedName("nacelle_body"), visual=visual, collision=collision)

    root_C_body3 = FixedConnection(
        parent=tower_body,
        child=nacelle_body,
        parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=0.3, y=-0.0, z=1.4)
    )
    nacelle = Nacelle(nacelle_body)

    # =====================================================================
    # Rotor Blade 1
    # =====================================================================
    body4 = Box(scale=Scale(0.2, 0.2, 1.5), color=white)
    visual = ShapeCollection([body4])
    collision = ShapeCollection([body4])
    blade1 = Body(name=PrefixedName("rotor_blade1"), visual=visual, collision=collision)

    root_C_body4 = FixedConnection(
        parent=nacelle_body,
        child=blade1,
        parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(
            x=0.10, y=-0.78, z=0.50, roll=1.0, pitch=0.0, yaw=0.0
        )
    )
    rotorblade1 = RotorBlades(blade1)

    # =====================================================================
    # Rotor Blade 2
    # =====================================================================
    body5 = Box(scale=Scale(0.2, 0.2, 1.5), color=white)
    visual = ShapeCollection([body5])
    collision = ShapeCollection([body5])
    blade2 = Body(name=PrefixedName("rotor_blade2"), visual=visual, collision=collision)

    root_C_body5 = FixedConnection(
        parent=nacelle_body,
        child=blade2,
        parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(
            x=0.10, y=0.75, z=0.55, roll=2.2, pitch=0.0, yaw=0.0
        )
    )
    rotorblade2 = RotorBlades(blade2)

    # =====================================================================
    # Rotor Blade 3 (Bottom)
    # =====================================================================
    body6 = Box(scale=Scale(0.2, 0.2, 1.5), color=white)
    visual = ShapeCollection([body6])
    collision = ShapeCollection([body6])
    blade3 = Body(name=PrefixedName("rotor_blade3"), visual=visual, collision=collision)

    root_C_body6 = FixedConnection(
        parent=nacelle_body,
        child=blade3,
        parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(
            x=0.1, y=0.0, z=-0.85, roll=0.0, pitch=0.0, yaw=0.0
        )
    )
    rotorblade3 = RotorBlades(blade3)

    # =====================================================================
    # Hub
    # =====================================================================
    body7 = Box(scale=Scale(0.2, 0.3, 0.2), color=blue)
    visual = ShapeCollection([body7])
    collision = ShapeCollection([body7])
    hub_body = Body(name=PrefixedName("hub_body"), visual=visual, collision=collision)

    root_C_body7 = FixedConnection(
        parent=nacelle_body,
        child=hub_body,
        parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=0.30, y=-0.0, z=0.0)
    )
    hub = Hub(hub_body)

    # =====================================================================
    # Add Bodies & Connections to the World
    # =====================================================================
    with world.modify_world():
        world.add_body(root)

        world.add_body(tower_body)
        world.add_connection(root_C_body1)

        world.add_body(base_body)
        world.add_connection(root_C_body2)

        world.add_body(nacelle_body)
        world.add_connection(root_C_body3)

        world.add_body(blade1)
        world.add_connection(root_C_body4)

        world.add_body(blade2)
        world.add_connection(root_C_body5)

        world.add_body(blade3)
        world.add_connection(root_C_body6)

        world.add_body(hub_body)
        world.add_connection(root_C_body7)

    # =====================================================================
    # ROS2 Node and Visualization Publisher
    # =====================================================================
    rclpy.init()
    node = rclpy.create_node("semantic_digital_twin")

    # Spin ROS2 node in background thread
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    viz = VizMarkerPublisher(world=world, node=node)
    time.sleep(1)


if __name__ == "__main__":
    main()
