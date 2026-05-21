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
```mermaid
flowchart LR
    Camera[Камера] --> PC[ПК / Raspberry]
    
    PC -->|USB| Arduino[Arduino]
    
    subgraph Power[Питание]
        ExtPower[Доп. питание<br/>5В / 2А+]
    end
    
    subgraph ArduinoConnect[Подключение к Arduino]
        Arduino -->|5V| Breadboard
        Arduino -->|GND| Breadboard
        Arduino -->|PWM сигнал| Breadboard
    end
    
    subgraph ServoPower[Питание сервы]
        ExtPower -->|VCC| Breadboard
        ExtPower -->|GND| Breadboard
    end
    
    Breadboard --> Servo[Сервопривод]
    
    Ball[Шарик] -.->|отслеживание| Camera
    Platform[Платформа] --- Servo

### Архитектура
PID_camera_script.py - скрипт для камеры, для захвата объекта и отслеживания его положения
pid_regulator.ino - скрипт для arduino, отвечающий за обработку координат объекта, PID и поворот серво

## Как поднять

1) Подключить камеру к ноутбуку или rasberi
2) убедиться что налажена передача данных с камеры на ардуино
3) на ноутбуке запустить скрипт PID_camera_script.py
4) на arduino загрузить pid_regulator.ino

