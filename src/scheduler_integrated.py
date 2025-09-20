#!/usr/bin/env python3
"""
Интегрированный планировщик роботов с использованием парсера входных данных.
"""
import sys
import os
import numpy as np
from ortools.sat.python import cp_model
from collision_checker import check_collision
from parser import parse_input, dump_output, Problem, Operation
from inverse_kinematics import check_robot_reachability, print_reachability_report

def calculate_trajectory_time(pick_point, place_point, operation_time, joints):
    """
    Расчет времени движения между точками с учетом кинематических ограничений.
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
    
    # Конвертируем угловые скорости в линейные (примерное соотношение)
    max_tcp_velocity = min_v_max * 0.01  # конвертация град/с -> м/с
    max_tcp_acceleration = min_a_max * 0.01  # конвертация град/с² -> м/с²
    
    # Ограничиваем максимальную скорость TCP разумными пределами
    max_tcp_velocity = min(max_tcp_velocity, 2.0)  # максимум 2 м/с
    max_tcp_acceleration = min(max_tcp_acceleration, 5.0)  # максимум 5 м/с²
    
    # Расчет времени по трапецеидальному профилю скорости
    t_accel = max_tcp_velocity / max_tcp_acceleration
    dist_accel = 0.5 * max_tcp_acceleration * t_accel * t_accel
    
    if distance <= 2 * dist_accel:
        # Недостаточно места для достижения максимальной скорости
        movement_time = 2 * (distance / max_tcp_acceleration) ** 0.5
    else:
        # Трапецеидальный профиль скорости
        dist_constant = distance - 2 * dist_accel
        t_constant = dist_constant / max_tcp_velocity
        movement_time = 2 * t_accel + t_constant
    
    # Конвертируем в миллисекунды и добавляем небольшой запас
    movement_time_ms = int(movement_time * 1000 * 1.2)  # 20% запас
    
    # Общее время = время движения + время операции
    total_time = movement_time_ms + operation_time
    
    print(f"Расстояние: {distance:.3f} м, Время движения: {movement_time_ms} мс, Общее время: {total_time} мс")    
    return total_time

def assign_operations_to_robots(problem: Problem, reachability_results: dict = None):
    """
    Назначение операций роботам с использованием CP-SAT.
    Учитывает достижимость точек для каждого робота.
    """
    model = cp_model.CpModel()
    assignments = {}
    
    # Создаем переменные назначения
    for op_id in range(problem.N):
        for r in range(problem.K):
            assignments[(op_id, r)] = model.NewBoolVar(f"op{op_id}_r{r}")
    
    # Собираем достижимые операции
    reachable_operations = set()
    if reachability_results:
        for op_id in range(problem.N):
            for r in range(problem.K):
                if r in reachability_results:
                    robot_results = reachability_results[r]
                    pick_reachable = robot_results['pick_points'][op_id]
                    place_reachable = robot_results['place_points'][op_id]
                    if pick_reachable and place_reachable:
                        reachable_operations.add(op_id)
                        break
    else:
        # Если нет результатов проверки достижимости, считаем все операции достижимыми
        reachable_operations = set(range(problem.N))
    
    # Каждая достижимая операция должна быть назначена ровно одному роботу
    for op_id in reachable_operations:
        model.Add(sum(assignments[(op_id, r)] for r in range(problem.K)) == 1)
    
    # Ограничения достижимости (если доступны результаты проверки)
    if reachability_results:
        # Сначала проверяем, есть ли хотя бы один робот, который может выполнить каждую операцию
        for op_id in range(problem.N):
            can_be_done = False
            for r in range(problem.K):
                if r in reachability_results:
                    robot_results = reachability_results[r]
                    pick_reachable = robot_results['pick_points'][op_id]
                    place_reachable = robot_results['place_points'][op_id]
                    if pick_reachable and place_reachable:
                        can_be_done = True
                        break
            
            if not can_be_done:
                print(f"⚠️ Операция {op_id} недостижима для всех роботов - пропускаем")
                continue
            
            # Если операция может быть выполнена, добавляем ограничения
            for r in range(problem.K):
                if r in reachability_results:
                    robot_results = reachability_results[r]
                    pick_reachable = robot_results['pick_points'][op_id]
                    place_reachable = robot_results['place_points'][op_id]
                    
                    if not (pick_reachable and place_reachable):
                        # Если точки недостижимы, запрещаем назначение
                        model.Add(assignments[(op_id, r)] == 0)
                        print(f"🚫 Операция {op_id} недостижима для робота {r}")
    
    # Переменная для makespan
    makespan = model.NewIntVar(0, 1000000, "makespan")
    
    # Расчет времени выполнения каждой достижимой операции
    operation_times = []
    for op_id, op in enumerate(problem.ops):
        if op_id in reachable_operations:
            time_for_op = calculate_trajectory_time(op.pick, op.place, op.deadline, problem.joints)
            operation_times.append(time_for_op)
            print(f"Операция {op}: время выполнения = {time_for_op} мс")
        else:
            operation_times.append(0)  # Недостижимые операции получают время 0
    
    # Ограничения для makespan - исправленная версия
    for op_id in range(problem.N):
        for r in range(problem.K):
            model.Add(makespan >= int(operation_times[op_id])).OnlyEnforceIf(assignments[(op_id, r)])
    
    # Добавляем переменные для времени работы каждого робота
    robot_times = []
    for r in range(problem.K):
        robot_time = model.NewIntVar(0, 1000000, f"robot_time_{r}")
        robot_times.append(robot_time)
        
        # Суммируем время всех операций, назначенных роботу r
        operation_times_for_robot = []
        for op_id in range(problem.N):
            operation_times_for_robot.append(assignments[(op_id, r)] * int(operation_times[op_id]))
        
        # robot_time = сумма времени всех операций робота
        model.Add(robot_time == sum(operation_times_for_robot))
    
    # Минимизируем makespan + штраф за дисбаланс
    # Добавляем переменную для максимального времени работы
    max_robot_time = model.NewIntVar(0, 1000000, "max_robot_time")
    for r in range(problem.K):
        model.Add(max_robot_time >= robot_times[r])
    
    # Минимизируем makespan + максимальное время работы робота
    model.Minimize(makespan + max_robot_time)
    
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
    Построение траекторий для каждого робота с учетом временной синхронизации.
    Роботы начинают из своих базовых позиций и идут к общей точке захвата.
    """
    robot_paths = {}
    
    # Собираем все операции с общими точками захвата
    pickup_points = {}
    for op_id, op in enumerate(problem.ops):
        pickup_key = (op.pick[0], op.pick[1], op.pick[2])
        if pickup_key not in pickup_points:
            pickup_points[pickup_key] = []
        pickup_points[pickup_key].append((op_id, op))
    
    # Для каждой общей точки захвата создаем очередь доступа
    pickup_schedules = {}
    for pickup_key, operations in pickup_points.items():
        if len(operations) > 1:  # Только для общих точек
            pickup_schedules[pickup_key] = []
    
    # Строим траектории с учетом очередей
    for robot_id, operation_ids in robot_assignments.items():
        if not operation_ids:
            robot_paths[robot_id] = []
            continue
            
        waypoints = []
        current_time = 0
        
        # Начинаем из базовой позиции робота
        robot_base = problem.robot_positions[robot_id]
        waypoints.append((current_time, robot_base))
        
        for op_id in operation_ids:
            op = problem.ops[op_id]
            pickup_key = (op.pick[0], op.pick[1], op.pick[2])
            
            # Проверяем, нужно ли ждать доступа к точке захвата
            if pickup_key in pickup_schedules:
                # Находим время, когда точка захвата будет свободна
                last_access_time = 0
                for scheduled_time, _ in pickup_schedules[pickup_key]:
                    # Время операции + время движения от точки захвата
                    operation_time = op.deadline
                    last_access_time = max(last_access_time, scheduled_time + operation_time)
                
                # Ждем, если необходимо
                if current_time < last_access_time:
                    current_time = last_access_time
                    print(f"🤖 Робот {robot_id} ждет доступа к точке захвата {pickup_key} до {current_time} мс")
                
                # Записываем время доступа
                pickup_schedules[pickup_key].append((current_time, robot_id))
            
            # Добавляем промежуточные waypoints для более интересных путей
            if len(waypoints) > 0:
                last_point = waypoints[-1][1]
                
                # Создаем промежуточную точку для обхода препятствий
                intermediate_point = create_intermediate_waypoint(last_point, op.pick, robot_id)
                if intermediate_point != op.pick:  # Если промежуточная точка отличается от цели
                    movement_time = calculate_trajectory_time(last_point, intermediate_point, 0, problem.joints)
                    current_time += movement_time
                    waypoints.append((current_time, intermediate_point))
            
            # Движение к точке захвата
            if len(waypoints) > 0:
                last_point = waypoints[-1][1]
                movement_time = calculate_trajectory_time(last_point, op.pick, 0, problem.joints)
                current_time += movement_time
            
            # Добавляем точку pick
            waypoints.append((current_time, op.pick))
            
            # Добавляем время операции захвата
            current_time += op.deadline
            
            # Движение к точке размещения
            movement_time = calculate_trajectory_time(op.pick, op.place, 0, problem.joints)
            current_time += movement_time
            
            # Добавляем точку place
            waypoints.append((current_time, op.place))
            
            # Добавляем время операции размещения
            current_time += op.deadline
        
        robot_paths[robot_id] = waypoints
    
    return robot_paths

def create_intermediate_waypoint(start_point, target_point, robot_id):
    """
    Создает промежуточную точку для более интересных путей.
    """
    start = np.array(start_point)
    target = np.array(target_point)
    
    # Вычисляем направление движения
    direction = target - start
    distance = np.linalg.norm(direction)
    
    if distance < 0.1:  # Если точки слишком близко
        return target_point
    
    # Нормализуем направление
    direction = direction / distance
    
    # Создаем промежуточную точку на 60% пути
    intermediate_distance = distance * 0.6
    intermediate_point = start + direction * intermediate_distance
    
    # Добавляем небольшое отклонение для каждого робота
    if robot_id == 0:
        # Робот 0 идет немного выше
        intermediate_point[2] += 0.05
    elif robot_id == 1:
        # Робот 1 идет немного ниже
        intermediate_point[2] -= 0.05
    else:
        # Робот 2 идет по прямой
        pass
    
    return tuple(intermediate_point)
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
        
        # Назначаем операции роботам с учетом достижимости
        print("\n🤖 Назначаем операции роботам...")
        robot_assignments, makespan, error = assign_operations_to_robots(problem, reachability_results)
        
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
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()
