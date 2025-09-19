import sys
from dataclasses import dataclass
from typing import List, Tuple

@dataclass
class JointLimit:
    j_min: float
    j_max: float
    v_max: float
    a_max: float

@dataclass
class Operation:
    pick: Tuple[float, float, float]
    place: Tuple[float, float, float]
    deadline: float

@dataclass
class Problem:
    K: int
    N: int
    joints: List[JointLimit]
    tool_clearance: float
    safe_dist: float
    ops: List[Operation]

def parse_input(path: str) -> Problem:
    with open(path, "r") as f:
        lines = [l.strip() for l in f if l.strip()]

    idx = 0
    K, N = map(int, lines[idx].split()); idx += 1

    joints = []
    for _ in range(6):
        jmin, jmax, vmax, amax = map(float, lines[idx].split()); idx += 1
        joints.append(JointLimit(jmin, jmax, vmax, amax))

    tool_clear, safe_dist = map(float, lines[idx].split()); idx += 1

    ops = []
    for _ in range(N):
        px, py, pz, qx, qy, qz, t = map(float, lines[idx].split()); idx += 1
        ops.append(Operation((px,py,pz),(qx,qy,qz),t))

    return Problem(K,N,joints,tool_clear,safe_dist,ops)


def dump_output(makespan: float, schedules: dict, path: str):
    """
    schedules = { robot_id: [(time,x,y,z),...] }
    """
    with open(path,"w") as f:
        f.write(f"{int(makespan)}\n")
        for rid, frames in schedules.items():
            f.write(f"R{rid} {len(frames)}\n")
            for t,x,y,z in frames:
                f.write(f"{int(t)} {x:.3f} {y:.3f} {z:.3f}\n")
