#!/usr/bin/env python3
from ortools.sat.python import cp_model
from collision_checker import check_collision

# === Demo input ===
num_robots = 2
operations = [
    ((0.0, 0.0, 0.0), (0.5, 0.0, 0.0), 1000),  # pick -> place
    ((0.5, 0.5, 0.0), (0.0, 0.5, 0.0), 1000),
    ((1.0, 0.0, 0.0), (1.0, 0.5, 0.0), 1000),
]

safe_dist = 0.2
output_file = "schedule_output.txt"

# === Part A: CP-SAT assignment ===
model = cp_model.CpModel()
assignments = {}
for op_id in range(len(operations)):
    for r in range(num_robots):
        assignments[(op_id, r)] = model.NewBoolVar(f"op{op_id}_r{r}")

for op_id in range(len(operations)):
    model.Add(sum(assignments[(op_id, r)] for r in range(num_robots)) == 1)

makespan = model.NewIntVar(0, 100000, "makespan")
durations = [op[2] for op in operations]
for op_id in range(len(operations)):
    for r in range(num_robots):
        model.Add(makespan >= durations[op_id]).OnlyEnforceIf(assignments[(op_id, r)])

model.Minimize(makespan)
solver = cp_model.CpSolver()
status = solver.Solve(model)

if status != cp_model.OPTIMAL:
    print("❌ No solution found")
    exit(1)

final_makespan = solver.Value(makespan)
print(f"✅ Makespan = {final_makespan} ms")

# === Part B: Build robot waypoints ===
robot_paths = {r: [] for r in range(num_robots)}
for op_id, op in enumerate(operations):
    pick, place, t_i = op
    for r in range(num_robots):
        if solver.Value(assignments[(op_id, r)]):
            print(f"Operation {op_id} → Robot {r}")
            robot_paths[r].append((pick, t_i))
            robot_paths[r].append((place, t_i))

# === Part C: Check collisions ===
for r1 in range(num_robots):
    for r2 in range(r1 + 1, num_robots):
        collision = check_collision(
            [p for p, _ in robot_paths[r1]],
            [p for p, _ in robot_paths[r2]],
            safe_dist,
        )
        if collision:
            print(f"⚠️ Collision detected between Robot{r1} and Robot{r2}")
        else:
            print(f"✅ No collision between Robot{r1} and Robot{r2}")

# === Part D: Save to file ===
with open(output_file, "w") as f:
    f.write(f"{final_makespan}\n")
    for r in range(num_robots):
        waypoints = []
        time_acc = 0
        for point, duration in robot_paths[r]:
            waypoints.append((time_acc, point))
            time_acc += duration
        f.write(f"R{r} {len(waypoints)}\n")
        for t, (x, y, z) in waypoints:
            f.write(f"{t} {x:.3f} {y:.3f} {z:.3f}\n")

print(f"💾 Output written to {output_file}")
