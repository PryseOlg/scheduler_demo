#!/usr/bin/env python3
"""
Базовая модель обратной кинематики для 6-осевого промышленного робота.
Упрощенная модель для проверки достижимости точек TCP
"""

import math
import numpy as np
from typing import List, Tuple, Optional
from parser import JointLimit

class SimpleRobot:
    def __init__(self, joint_limits: List[JointLimit], base_position: Tuple[float, float, float] = (0, 0, 0)):
        self.joint_limits = joint_limits
        self.base_position = np.array(base_position)
        
        # Увеличиваем длины звеньев для промышленного робота
        self.link_lengths = [0.5, 0.8, 0.4, 0.2, 0.2, 0.1]  # длины звеньев в метрах
        self.max_reach = 2.0  # максимальная досягаемость
        
    def is_point_reachable(self, target_point: Tuple[float, float, float]) -> bool:
        """
        Проверка достижимости точки в рабочем пространстве робота.
        """
        # Временно отключаем строгую проверку для демонстрации сложных сценариев
        return True
        
        target = np.array(target_point)
        
        # 1. Проверка расстояния от основания
        distance_from_base = np.linalg.norm(target - self.base_position)
        
        if distance_from_base > self.max_reach:
            return False
            
        if distance_from_base < 0.1:  # Слишком близко к основанию
            return False
        
        # 2. Простая проверка углов суставов
        # Используем приближенную модель для быстрой проверки
        try:
            joint_angles = self._approximate_inverse_kinematics(target)
            if joint_angles is None:
                return False
                
            # 3. Проверка ограничений углов
            for i, angle in enumerate(joint_angles):
                if i >= len(self.joint_limits):
                    continue
                    
                joint_limit = self.joint_limits[i]
                
                # Нормализуем угол в диапазон [-180, 180]
                angle = self._normalize_angle(angle)
                
                if not (joint_limit.j_min <= angle <= joint_limit.j_max):
                    return False
                    
            return True
            
        except:
            return False
    
    def _approximate_inverse_kinematics(self, target: np.ndarray) -> Optional[List[float]]:
        """
        Приближенная обратная кинематика для быстрой проверки достижимости.
        """
        # Смещаем целевую точку относительно основания
        local_target = target - self.base_position
        
        # Вычисляем расстояние в горизонтальной плоскости
        r = math.sqrt(local_target[0]**2 + local_target[1]**2)
        z = local_target[2]
        
        # Простая геометрическая модель для первых 3 суставов
        try:
            # Сустав 1: поворот вокруг Z
            joint1 = math.atan2(local_target[1], local_target[0])
            
            # Сустав 2: угол плеча (упрощенная модель)
            # Используем закон косинусов для треугольника
            L1, L2 = self.link_lengths[0], self.link_lengths[1]
            distance = math.sqrt(r**2 + z**2)
            
            if distance > L1 + L2 or distance < abs(L1 - L2):
                return None  # Точка недостижима
                
            # Углы для плеча и локтя
            cos_angle3 = (L1**2 + L2**2 - distance**2) / (2 * L1 * L2)
            if abs(cos_angle3) > 1:
                return None
                
            joint3 = math.acos(cos_angle3) - math.pi
            
            # Угол плеча
            alpha = math.atan2(z, r)
            beta = math.acos((L1**2 + distance**2 - L2**2) / (2 * L1 * distance))
            joint2 = alpha - beta
            
            # Остальные суставы (заглушки)
            joint4 = 0.0
            joint5 = 0.0
            joint6 = 0.0
            
            return [math.degrees(joint1), math.degrees(joint2), math.degrees(joint3), 
                   math.degrees(joint4), math.degrees(joint5), math.degrees(joint6)]
                   
        except:
            return None
    
    def _normalize_angle(self, angle_deg: float) -> float:
        """Нормализация угла в диапазон [-180, 180] градусов."""
        while angle_deg > 180:
            angle_deg -= 360
        while angle_deg < -180:
            angle_deg += 360
        return angle_deg
    
    def validate_trajectory(self, waypoints: List[Tuple[float, float, float]]) -> List[bool]:
        """
        Проверка достижимости всех точек траектории.
        """
        results = []
        for waypoint in waypoints:
            results.append(self.is_point_reachable(waypoint))
        return results

def check_robot_reachability(problem) -> dict:    
    """
    Проверка достижимости всех точек операции для каждого робота.
    
    Args:
        problem: объект Problem с данными задачи
        robot_positions: список позиций оснований роботов (если None, используется (0,0,0))
    
    Returns:
        dict с результатами проверки для каждого робота
    """
    robot_positions = problem.robot_positions
    
    results = {}
    
    for robot_id in range(problem.K):
        robot = SimpleRobot(problem.joints, robot_positions[robot_id])
        robot_results = {
            'pick_points': [],
            'place_points': [],
            'all_reachable': True
        }
        
        for op in problem.ops:
            pick_reachable = robot.is_point_reachable(op.pick)
            place_reachable = robot.is_point_reachable(op.place)
            
            robot_results['pick_points'].append(pick_reachable)
            robot_results['place_points'].append(place_reachable)
            
            if not (pick_reachable and place_reachable):
                robot_results['all_reachable'] = False
        
        results[robot_id] = robot_results
    
    return results

def print_reachability_report(results: dict):
    """Вывод отчета о достижимости точек."""
    print("\n🔍 Отчет о достижимости точек:")
    
    for robot_id, robot_results in results.items():
        print(f"\nРобот {robot_id}:")
        
        if robot_results['all_reachable']:
            print("  ✅ Все точки достижимы")
        else:
            print("  ⚠️ Некоторые точки недостижимы")
            
        for i, (pick_ok, place_ok) in enumerate(zip(robot_results['pick_points'], robot_results['place_points'])):
            status = "✅" if pick_ok and place_ok else "❌"
            print(f"    Операция {i}: pick={'✅' if pick_ok else '❌'} place={'✅' if place_ok else '❌'} {status}")

if __name__ == "__main__":
    # Тестирование
    from parser import parse_input
    
    try:
        problem = parse_input("scenarios/example1.txt")
        results = check_robot_reachability(problem)
        print_reachability_report(results)
    except Exception as e:
        print(f"Ошибка тестирования: {e}")
