from typing import Tuple
import numpy as np
import random


SDF_WORLD_PATH = "/home/roman/PX4-Autopilot/Tools/simulation/gz/worlds/"
SDF_WORLD_NAME = "default.sdf"
OBSTACLES = ["Pine Tree", "telephone_pole", "Metal Peg", "Wooden Peg"]
MY_OBSTACLES = OBSTACLES[:1]
NO_OBSTACLES = 17
OBSTACLES_AREA = ((5, 30), (-6.5, 6.5))
MIN_OBSTACLES_DISTANCE = 5


def obst_sdf_code(obstacle: str, pose: Tuple[float], rotation: float, name: str) -> str:
    code = f"    <include><uri>{obstacle}</uri><name>{name}</name><pose>{pose[0]} {pose[1]} 0 0 0 {rotation}</pose></include>\n"
    return code

def main():
    with open(SDF_WORLD_PATH + SDF_WORLD_NAME, "r") as f:
        code_lines = f.readlines()

    info = "    <!-- Randomly generated obstacles -->\n"
    info2 =f"    <!-- Number of obstacles = {NO_OBSTACLES} -->\n"
    if info in code_lines:
        del code_lines[-4-int(code_lines[code_lines.index(info) + 1][-7:-5]):-2]

    code_lines.insert(-2, info)
    code_lines.insert(-2, info2)

    positions = list()
    for i in range(NO_OBSTACLES):
        pose = (random.uniform(OBSTACLES_AREA[0][0], OBSTACLES_AREA[0][1]), random.uniform(OBSTACLES_AREA[1][0], OBSTACLES_AREA[1][1]))

        for prev_pose in positions:
            dist = np.linalg.norm(np.subtract(prev_pose, pose))
            if dist < MIN_OBSTACLES_DISTANCE:
                pose = (random.uniform(OBSTACLES_AREA[0][0], OBSTACLES_AREA[0][1]), random.uniform(OBSTACLES_AREA[1][0], OBSTACLES_AREA[1][1]))
            else:
                break

        rotation = random.uniform(-np.pi, np.pi)
        obstacle = random.choice(MY_OBSTACLES)

        code = obst_sdf_code(obstacle, pose, rotation, f"obstacle_{i+1}")

        code_lines.insert(-2, code)

    with open(f"{SDF_WORLD_PATH}{SDF_WORLD_NAME}", "w") as f:
        f.writelines(code_lines)


if __name__ == "__main__":
    main()