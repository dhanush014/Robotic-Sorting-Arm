#!/usr/bin/env python3

import random
import re
import sys
from pathlib import Path


OBJECTS = [
    "red_cube",
    "blue_cube",
    "red_cylinder",
    "blue_cylinder",
]

POSITIONS = [
    (-0.45, 0.20, 0.85),
    (-0.15, 0.20, 0.85),
    (0.15, 0.20, 0.85),
    (0.45, 0.20, 0.85),
]

OBSTACLES = [
    "obstacle_bottle",
    "obstacle_ball",
]

NUM_OBSTACLES = 5

OBSTACLE_POSITIONS = [
    (-0.6, 0.0, 0.775),
    (-0.3, 0.0, 0.775),
    (0.0, 0.0, 0.775),
    (0.3, 0.0, 0.775),
    (0.6, 0.0, 0.775),
    (-0.6, -0.2, 0.775),
    (-0.3, -0.2, 0.775),
    (0.0, -0.2, 0.775),
    (0.3, -0.2, 0.775),
    (0.6, -0.2, 0.775),
    (-0.6, 0.4, 0.775),
    (-0.3, 0.4, 0.775),
    (0.0, 0.4, 0.775),
    (0.3, 0.4, 0.775),
    (0.6, 0.4, 0.775),
]


def randomize_world(input_world, output_world):
    world_text = Path(input_world).read_text()

    shuffled_objects = OBJECTS.copy()
    random.shuffle(shuffled_objects)

    for object_type, position in zip(shuffled_objects, POSITIONS):
        x, y, z = position

        pattern = (
            rf'(<uri>model://{re.escape(object_type)}</uri>'
            rf'\s*<name>{re.escape(object_type)}_1</name>'
            rf'\s*<pose>)[^<]*(</pose>)'
        )

        replacement = rf'\g<1>{x} {y} {z} 0 0 0\g<2>'

        world_text, count = re.subn(
            pattern,
            replacement,
            world_text,
            count=1,
        )

        if count != 1:
            raise RuntimeError(
                f"Could not find include block for {object_type}"
            )

    world_text = re.sub(
        r'[ \t]*<include>\s*<uri>model://obstacle_[^<]*</uri>.*?</include>[ \t]*\n?',
        '',
        world_text,
        flags=re.DOTALL,
    )

    chosen_positions = random.sample(OBSTACLE_POSITIONS, NUM_OBSTACLES)
    type_counts = {}
    placed_obstacles = []
    include_blocks = ""

    for position in chosen_positions:
        obstacle_type = random.choice(OBSTACLES)
        type_counts[obstacle_type] = type_counts.get(obstacle_type, 0) + 1
        obstacle_name = f"{obstacle_type}_{type_counts[obstacle_type]}"
        x, y, z = position

        placed_obstacles.append((obstacle_name, position))
        include_blocks += (
            f"    <include>\n"
            f"        <uri>model://{obstacle_type}</uri>\n"
            f"        <name>{obstacle_name}</name>\n"
            f"        <pose>{x} {y} {z} 0 0 0</pose>\n"
            f"    </include>\n\n"
        )

    insert_at = world_text.rfind("</world>")
    if insert_at == -1:
        raise RuntimeError("Could not find </world> in input world")
    world_text = world_text[:insert_at] + include_blocks + world_text[insert_at:]

    Path(output_world).write_text(world_text)

    print("Randomized object placement:")
    for position, object_type in zip(POSITIONS, shuffled_objects):
        print(f"  {object_type}: {position}")

    print("\nRandomized obstacle placement:")
    for obstacle_type, position in placed_obstacles:
        print(f"  {obstacle_type}: {position}")

    print(f"\nGenerated world: {output_world}")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print(
            f"Usage: {sys.argv[0]} <input_world.sdf> <output_world.sdf>"
        )
        sys.exit(1)

    randomize_world(sys.argv[1], sys.argv[2])