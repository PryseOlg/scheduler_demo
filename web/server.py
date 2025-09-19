#!/usr/bin/env python3
"""
Простой веб-сервер для интеграции планировщика роботов с визуализатором.
"""
import os
import sys
import json
import subprocess
import tempfile
from flask import Flask, request, jsonify, send_from_directory
from flask_cors import CORS

# Добавляем путь к модулям планировщика
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

app = Flask(__name__)
CORS(app)  # Разрешаем CORS для работы с фронтендом

@app.route('/')
def index():
    """Главная страница - возвращаем визуализатор."""
    return send_from_directory('.', 'visualizer.html')

@app.route('/api/run_scheduler', methods=['POST'])
def run_scheduler():
    """
    API endpoint для запуска планировщика.
    Принимает содержимое файла сценария и возвращает результат планирования.
    """
    try:
        data = request.get_json()
        scenario_content = data.get('scenario', '')
        
        if not scenario_content:
            return jsonify({'error': 'Не предоставлено содержимое сценария'}), 400
        
        # Создаем временный файл для входных данных
        with tempfile.NamedTemporaryFile(mode='w', suffix='.txt', delete=False) as input_file:
            input_file.write(scenario_content)
            input_file_path = input_file.name
        
        # Создаем временный файл для выходных данных
        with tempfile.NamedTemporaryFile(mode='w', suffix='.txt', delete=False) as output_file:
            output_file_path = output_file.name
        
        try:
            # Запускаем планировщик
            scheduler_path = os.path.join(os.path.dirname(__file__), '..', 'src', 'scheduler_integrated.py')
            result = subprocess.run([
                sys.executable, scheduler_path, input_file_path, output_file_path
            ], capture_output=True, text=True, timeout=30)
            
            if result.returncode != 0:
                return jsonify({
                    'error': f'Ошибка планировщика: {result.stderr}',
                    'stdout': result.stdout
                }), 500
            
            # Читаем результат
            with open(output_file_path, 'r') as f:
                schedule_content = f.read()
            
            # Парсим makespan из первой строки
            lines = schedule_content.strip().split('\n')
            makespan = int(lines[0]) if lines else 0
            
            # Подсчитываем количество роботов и операций
            num_robots = len([line for line in lines if line.startswith('R')])
            
            return jsonify({
                'success': True,
                'makespan': makespan,
                'schedule': schedule_content,
                'num_robots': num_robots,
                'output': result.stdout
            })
            
        finally:
            # Удаляем временные файлы
            try:
                os.unlink(input_file_path)
                os.unlink(output_file_path)
            except:
                pass
                
    except subprocess.TimeoutExpired:
        return jsonify({'error': 'Планировщик превысил время ожидания (30 сек)'}), 500
    except Exception as e:
        return jsonify({'error': f'Внутренняя ошибка сервера: {str(e)}'}), 500

@app.route('/api/scenarios')
def get_scenarios():
    """Возвращает список доступных сценариев."""
    scenarios_dir = os.path.join(os.path.dirname(__file__), '..', 'scenarios')
    scenarios = []
    
    if os.path.exists(scenarios_dir):
        for filename in os.listdir(scenarios_dir):
            if filename.endswith('.txt'):
                file_path = os.path.join(scenarios_dir, filename)
                try:
                    with open(file_path, 'r') as f:
                        content = f.read()
                    scenarios.append({
                        'name': filename,
                        'content': content
                    })
                except:
                    pass
    
    return jsonify({'scenarios': scenarios})

@app.route('/health')
def health():
    """Проверка состояния сервера."""
    return jsonify({'status': 'ok', 'message': 'Сервер планировщика роботов работает'})

if __name__ == '__main__':
    print("🚀 Запуск сервера планировщика роботов...")
    print("📱 Откройте http://localhost:5000 в браузере")
    print("🛑 Для остановки нажмите Ctrl+C")
    
    app.run(host='0.0.0.0', port=5000, debug=True)
