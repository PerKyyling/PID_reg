#include <Servo.h>
#include <math.h>
#include "QueueList.h"


//Нейтральное положение сервоприводов (платформа горизонтальна)
#define CENTER_ANGLE_Y  90
#define CENTER_ANGLE_X  90
#define ANGLE_RANGE 5 // Максимальное отклонение от нейтрального угла


#define MIN_ANGLE_Y (CENTER_ANGLE_Y - ANGLE_RANGE)
#define MAX_ANGLE_Y (CENTER_ANGLE_Y + ANGLE_RANGE)
#define MIN_ANGLE_X (CENTER_ANGLE_X - ANGLE_RANGE)
#define MAX_ANGLE_X (CENTER_ANGLE_X + ANGLE_RANGE)


#define SERVO_X_PIN  5
#define SERVO_Y_PIN  6


Servo servo_x, servo_y;

class Reg
{
    // Коэффициенты ПИД
    float KP = 0.619;
    float KI = 0.00106;
    float KD = 0.016699;

    float DeathZone = 0.2;
    float Final = 0.0;

    float DT = 0.001;

    float prev_err = 0.0; // Предыдущая ошибка для дифференциальной части
    QueueList<float> integral; // Накопленный интеграл

public:
    Reg() : prev_err(0.0) {}; // GOIDA
    Reg(float kp, float ki, float kd, float dz, float final)
            : KP(kp), KI(ki), KD(kd), DeathZone(dz), Final(final), prev_err(0.0) // GOIDA
    {};

    // Основная функция ПИД-регулятора
    float upd(float curr){
        float err = Err(curr);

        if(fabs(err) <= DeathZone) {
            while(!integral.isEmpty()) {
                integral.pop();
            }
        }

        float pr = Pr(err);
        float in = In(err);
        float df = Df(err, prev_err);
        prev_err = err;

        return pr + in + df;
    };

private:
    // Отклонение шара от Final
    float Err(float curr){
        return Final - curr;
    };

    // Пропорциональная составляющая
    float Pr(float err){
        return KP * err;
    };

    // Интегральная составляющая
    float In(float err){
        if (integral.getCount() >= 5) {
            float time_per = integral.pop();
            float returned_integral = 0;
            for (int i = 0; i < 4; i ++){
                float timed_piece_integral = integral.pop();
                returned_integral = returned_integral + timed_piece_integral * DT;
                integral.push(timed_piece_integral);
            }
            returned_integral = returned_integral + err * DT;
            integral.push(err);
            return KI * returned_integral;
        }
        else {
            integral.push(err);
            return 0;
        }
    };

    // Дифференциальная составляющая
    float Df(float err, float prev_err){
        float dif = (err - prev_err) / DT;
        return KD * dif;
    };
};


float x, y; // Полученные координаты с камеры
float x_persp, y_persp; // Координаты с поправкой на перспективу
float out_x, out_y; // Выходные значения ПИД-регуляторов
int angle_x, angle_y; // Углы для сервоприводов

String data; // Данные, полученные с Serial
Reg reg_x, reg_y;

void setup() {
    Serial.begin(9600);
    Serial.flush();

    servo_x.attach(SERVO_X_PIN);
    servo_y.attach(SERVO_Y_PIN);

    servo_x.write(CENTER_ANGLE_X);
    servo_y.write(CENTER_ANGLE_Y);

    delay(2000);
}

void loop() {

    if (Serial.available() > 0) {
        data = Serial.readStringUntil('\n');

        // Извлечение координат
        x = data.substring(0, data.indexOf(',')).toFloat();
        y = data.substring(data.indexOf(',') + 1).toFloat();

        // Коррекция на перспективу
        x_persp = x / cos(radians(servo_x.read() - CENTER_ANGLE_X));
        y_persp = y / cos(radians(servo_y.read() - CENTER_ANGLE_Y));

        out_x = reg_x.upd(x_persp);
        out_y = reg_y.upd(y_persp);

        angle_x = constrain(CENTER_ANGLE_X + out_x, MIN_ANGLE_X, MAX_ANGLE_X);
        angle_y = constrain(CENTER_ANGLE_Y + out_y * -1, MIN_ANGLE_Y, MAX_ANGLE_Y);

        servo_x.write(angle_x);
        servo_y.write(angle_y);
    }
    delay(1);
}