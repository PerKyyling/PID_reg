# PID регулятор

Проет для научной конфиренции: Инженеры будущего (2025)

## О проекте

Проект из себя представляет полностью готовый стенд, состоящий из:
- Физической части
- платы arduino (c++)
- камеры с моделью для отслеживания координат
- сервоприводов (далее серво)
- доп. питания
- макетной платы

### Схема подключения
flowchart LR
    Camera[Камера] --> PC[ПК/Ноутбук]
    PC -->|USB| Arduino[Arduino/Плата]
    Arduino --> breadboard[Макетная плата]
    servo[Сервопривода] --> breadboard[Макетная плата]
    power[Доп. питание] --> breadboard[Макетная плата]
    Ball[Шарик] -.->|отслеживание| Camera
    Platform[Платформа] --- Servo


## Быстрый старт

### Требования

- Python 3.9+
- Redis (установлен локально или через Docker)

### Установка

```bash
# 1. Клонировать репозиторий
git clone https://github.com/PerKyyling/AvailOlimp
cd AvailOlimp

# 2. Установить зависимости
pip install -r requirements.txt

# 3. Запустить Redis (в отдельном окне)
redis-server
# Или через Docker: docker run -d -p 6379:6379 redis

# 4. Запустить PvE сервис (одиночный режим)
python index.py

# 5. Запустить PvP сервис (в новом окне)
python server_for_game.py
