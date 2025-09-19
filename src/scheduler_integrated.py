#!/usr/bin/env python3
"""
Интегрированный планировщик роботов с использованием парсера входных данных.
"""
import sys
from ortools.sat.python import cp_model
from collision_checker import check_collision
from parser import parse_input, dump_output, Problem, Operation
from inverse_kinematics import check_robot_reachability, print_reachability_report

def calculate_trajectory_time(pick_point, place_point, operation_time, joints):
    """
    Расчет времени движения между точками с учетом кинематических ограничений.
    Используем модель трапецеидального профиля скорости для каждого сустава.
    """
    # Расстояние между точками в 3D пространстве
    distance = ((place_point[0] - pick_point[0])**2 + 
                (place_point[1] - pick_point[1])**2 + 
                (place_point[2] - pick_point[2])**2)**0.5
    
    if distance < 1e-6:  # Если точки совпадают
        return int(operation_time)
    
    # Находим ограничивающий сустав (с наименьшим v_max)
    min_v_max = min(joint.v_max for joint in joints)
    min_a_max = min(joint.a_max for joint in joints)
    
    # Упрощенная модель: считаем, что TCP движется со скоростью, ограниченной
    # самым медленным суставом. Это приближение для 6-осевого робота.
    
    # Конвертируем угловые скорости в линейные (примерное соотношение)
    # Для промышленного робота обычно 1-2 м/с максимальная скорость TCP
    max_tcp_velocity = min_v_max * 0.01  # конвертация град/с -> м/с (приблизительно)
    max_tcp_acceleration = min_a_max * 0.01  # конвертация град/с² -> м/с²
    
    # Ограничиваем максимальную скорость TCP разумными пределами
    max_tcp_velocity = min(max_tcp_velocity, 2.0)  # максимум 2 м/с
    max_tcp_acceleration = min(max_tcp_acceleration, 5.0)  # максимум 5 м/с²
    
    # Расчет времени по трапецеидальному профилю скорости
    # Время разгона до максимальной скорости
    t_accel = max_tcp_velocity / max_tcp_acceleration
    
    # Расстояние, пройденное за время разгона
    dist_accel = 0.5 * max_tcp_acceleration * t_accel * t_accel
    
    if distance <= 2 * dist_accel:
        # Недостаточно места для достижения максимальной скорости
        # Треугольный профиль скорости
        movement_time = 2 * (distance / max_tcp_acceleration) ** 0.5
    else:
        # Трапецеидальный профиль скорости
        dist_constant = distance - 2 * dist_accel
        t_constant = dist_constant / max_tcp_velocity
        movement_time = 2 * t_accel + t_constant
    
    # Конвертируем в миллисекунды и добавляем небольшой запас
    movement_time_ms = int(movement_time * 1000 * 1.2)  # 20% запас на неточности модели
    
    # Общее время = время движения + время операции
    total_time = movement_time_ms + operation_time
    
    return total_time

def assign_operations_to_robots(problem: Problem):
    """
    Назначение операций роботам с использованием CP-SAT.
    """
    model = cp_model.CpModel()
    assignments = {}
    
    # Создаем переменные назначения: assignments[(op_id, robot_id)] = bool
    for op_id in range(problem.N):
        for r in range(problem.K):
            assignments[(op_id, r)] = model.NewBoolVar(f"op{op_id}_r{r}")
    
    # Каждая операция должна быть назначена ровно одному роботу
    for op_id in range(problem.N):
        model.Add(sum(assignments[(op_id, r)] for r in range(problem.K)) == 1)
    
    # Переменная для makespan
    makespan = model.NewIntVar(0, 1000000, "makespan")
    
    # Расчет времени выполнения каждой операции
    operation_times = []
    for op in problem.ops:
        time_for_op = calculate_trajectory_time(op.pick, op.place, op.deadline, problem.joints)
        operation_times.append(time_for_op)
    
    # Ограничения для makespan
    for op_id in range(problem.N):
        for r in range(problem.K):
            model.Add(makespan >= int(operation_times[op_id])).OnlyEnforceIf(assignments[(op_id, r)])
    
    # Минимизируем makespan
    model.Minimize(makespan)
    
    # Решаем задачу
    solver = cp_model.CpSolver()
    status = solver.Solve(model)
    
    if status != cp_model.OPTIMAL:
        return None, None, f"❌ Не удалось найти решение. Статус: {status}"
    
    final_makespan = solver.Value(makespan)
    
    # Извлекаем назначения
    robot_assignments = {r: [] for r in range(problem.K)}
    for op_id in range(problem.N):
        for r in range(problem.K):
            if solver.Value(assignments[(op_id, r)]):
                robot_assignments[r].append(op_id)
                print(f"Операция {op_id} → Робот {r}")
    
    return robot_assignments, final_makespan, None

def build_robot_paths(problem: Problem, robot_assignments: dict, makespan: int):
    """
    Построение траекторий для каждого робота.
    """
    robot_paths = {}
    
    for robot_id, operation_ids in robot_assignments.items():
        if not operation_ids:
            robot_paths[robot_id] = []
            continue
            
        waypoints = []
        current_time = 0
        
        for op_id in operation_ids:
            op = problem.ops[op_id]
            
            # Добавляем точку pick
            waypoints.append((current_time, op.pick))
            
            # Рассчитываем время движения к точке place
            movement_time = calculate_trajectory_time(op.pick, op.place, 0, problem.joints)
            current_time += movement_time
            
            # Добавляем точку place
            waypoints.append((current_time, op.place))
            
            # Добавляем время операции
            current_time += op.deadline
        
        robot_paths[robot_id] = waypoints
    
    return robot_paths

def check_all_collisions(robot_paths: dict, safe_dist: float):
    """
    Проверка коллизий между всеми парами роботов.
    """
    robot_ids = list(robot_paths.keys())
    collisions = []
    
    for i in range(len(robot_ids)):
        for j in range(i + 1, len(robot_ids)):
            r1, r2 = robot_ids[i], robot_ids[j]
            path1 = [point for time, point in robot_paths[r1]]
            path2 = [point for time, point in robot_paths[r2]]
            
            if path1 and path2:
                collision = check_collision(path1, path2, safe_dist)
                if collision:
                    collisions.append(f"⚠️ Коллизия между Роботом{r1} и Роботом{r2}")
                    print(f"⚠️ Коллизия между Роботом{r1} и Роботом{r2}")
                else:
                    print(f"✅ Нет коллизий между Роботом{r1} и Роботом{r2}")
    
    return collisions

def main():
    if len(sys.argv) != 3:
        print("Использование: python scheduler_integrated.py <входной_файл> <выходной_файл>")
        print("Пример: python scheduler_integrated.py scenarios/example1.txt output.txt")
        sys.exit(1)
    
    input_file = sys.argv[1]
    output_file = sys.argv[2]
    
    print(f"📖 Загружаем данные из {input_file}")
    
    try:
        # Парсим входные данные
        problem = parse_input(input_file)
        print(f"✅ Загружено: {problem.K} роботов, {problem.N} операций")
        print(f"   Tool clearance: {problem.tool_clearance}, Safe distance: {problem.safe_dist}")
        
        # Проверяем достижимость точек
        print("\n🔍 Проверяем достижимость точек...")
        reachability_results = check_robot_reachability(problem)
        print_reachability_report(reachability_results)
        
        # Проверяем, есть ли недостижимые точки
        unreachable_operations = []
        for robot_id, robot_results in reachability_results.items():
            for op_id, (pick_ok, place_ok) in enumerate(zip(robot_results['pick_points'], robot_results['place_points'])):
                if not (pick_ok and place_ok):
                    unreachable_operations.append((robot_id, op_id))
        
        if unreachable_operations:
            print(f"\n⚠️ Обнаружены недостижимые точки для {len(unreachable_operations)} операций")
            print("   Планировщик продолжит работу, но результаты могут быть некорректными.")
        
        # Назначаем операции роботам
        print("\n🤖 Назначаем операции роботам...")
        robot_assignments, makespan, error = assign_operations_to_robots(problem)
        
        if error:
            print(error)
            sys.exit(1)
        
        print(f"✅ Makespan = {makespan} мс")
        
        # Строим траектории
        print("\n🛤️ Строим траектории...")
        robot_paths = build_robot_paths(problem, robot_assignments, makespan)
        
        # Проверяем коллизии
        print("\n🔍 Проверяем коллизии...")
        collisions = check_all_collisions(robot_paths, problem.safe_dist)
        
        # Подготавливаем данные для сохранения
        schedules = {}
        for robot_id, waypoints in robot_paths.items():
            # Преобразуем (time, point) в (time, x, y, z)
            frames = []
            for time, point in waypoints:
                frames.append((time, point[0], point[1], point[2]))
            schedules[robot_id] = frames
        
        # Сохраняем результат
        print(f"\n💾 Сохраняем результат в {output_file}")
        dump_output(makespan, schedules, output_file)
        
        print("✅ Готово!")
        
        if collisions:
            print("\n⚠️ Обнаружены коллизии:")
            for collision in collisions:
                print(f"   {collision}")
        
    except FileNotFoundError:
        print(f"❌ Файл {input_file} не найден")
        sys.exit(1)
    except Exception as e:
        print(f"❌ Ошибка: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
