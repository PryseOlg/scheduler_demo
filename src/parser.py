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
    robot_positions: List[Tuple[float, float, float]] 

def parse_input(path: str) -> Problem:
    with open(path, "r") as f:
        lines = [l.strip() for l in f if l.strip()]

    idx = 0
    
    # Более устойчивый парсинг первой строки
    if not lines:
        raise ValueError("Файл пуст или содержит только пустые строки")
    
    first_line_parts = lines[idx].split()
    if len(first_line_parts) == 2:
        K, N = map(int, first_line_parts)
    elif len(first_line_parts) == 1:
        # Если только одно значение, предполагаем что это количество роботов
        K = int(first_line_parts[0])
        N = 0  # Будем считать операции из файла
        print(f"⚠️ Предупреждение: в первой строке только одно значение. Предполагаем K={K}, N будет подсчитано из файла")
    else:
        raise ValueError(f"Неверный формат первой строки: '{lines[idx]}'. Ожидается формат 'K N' или 'K'")
    
    idx += 1

    robot_positions = []
    for _ in range(K):
        if idx < len(lines):
            coords = list(map(float, lines[idx].split()))
            if len(coords) == 3:  # Новый формат с позициями роботов
                robot_positions.append(tuple(coords))
                idx += 1
            else:  # Старый формат без позиций роботов
                robot_positions.append((0.0, 0.0, 0.0))
                # Не увеличиваем idx, так как эта строка содержит параметры суставов
        else:
            robot_positions.append((0.0, 0.0, 0.0))  # По умолчанию

    joints = []
    for _ in range(6):
        jmin, jmax, vmax, amax = map(float, lines[idx].split()); idx += 1
        joints.append(JointLimit(jmin, jmax, vmax, amax))

    tool_clear, safe_dist = map(float, lines[idx].split()); idx += 1

    ops = []
    if N == 0:
        # Подсчитываем операции из оставшихся строк
        remaining_lines = lines[idx:]
        N = len(remaining_lines)
        print(f"📊 Подсчитано {N} операций из файла")
    
    for _ in range(N):
        if idx >= len(lines):
            break
        px, py, pz, qx, qy, qz, t = map(float, lines[idx].split()); idx += 1
        ops.append(Operation((px,py,pz),(qx,qy,qz),t))

    return Problem(K, N, joints, tool_clear, safe_dist, ops, robot_positions)  # Добавили robot_positions


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
