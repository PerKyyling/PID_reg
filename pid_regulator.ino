#include <Servo.h>
#include <math.h>
#include "QueueList.h"

/*Следующие константы используются для режима сервоприводов в градусной мере
//====================//
//Нейтральное положение сервоприводов (платформа горизонтальна)
#define CENTER_ANGLE_Y  90
#define CENTER_ANGLE_X  90
#define ANGLE_RANGE 5 // Максимальное отклонение от нейтрального положения

#define MIN_ANGLE_Y (CENTER_ANGLE_Y - ANGLE_RANGE)
#define MAX_ANGLE_Y (CENTER_ANGLE_Y + ANGLE_RANGE)
#define MIN_ANGLE_X (CENTER_ANGLE_X - ANGLE_RANGE)
#define MAX_ANGLE_X (CENTER_ANGLE_X + ANGLE_RANGE)
//====================//
*/

/*Следующие константы используются для режима сервоприводов в микросекундной мере*/
//====================//
//Нейтральное положение сервоприводов (платформа горизонтальна)
#define CENTER_MS 1500 // У любого сервопривода нейтральное положение - 1500 микросекунд (но это не точно)
#define MS_RANGE 100  // Максимальное отклонение от нейтрального положения

#define MIN_MS (CENTER_MS - MS_RANGE)
#define MAX_MS (CENTER_MS + MS_RANGE)

//ТРЕБУЕТСЯ КАЛИБРОВКА//
#define MMIN_MS_Y 1000
#define MMAX_MS_Y 2000
#define MMIN_MS_X 1000
#define MMAX_MS_X 2000
//ТРЕБУЕТСЯ КАЛИБРОВКА//

#define MS_PER_DEGREE_X ((MMAX_MS_X - MMIN_MS_X) / 180.0)
#define MS_PER_DEGREE_Y ((MMAX_MS_Y - MMIN_MS_Y) / 180.0)
//====================//

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
int ms_x, ms_y; // Микросекунды для сервоприводов

String data; // Данные, полученные с Serial
Reg reg_x, reg_y;

float normalize_perspective_degrees(float coordinate, int current_degrees, int center_degrees){
  float angle_offset = current_degrees - center_degrees
  return coordinate / cos(radians(angle_offset));
}


float normalize_perspective_ms(float coordinate, int current_ms, int center_ms, float ms_per_degree) {
  float angle_offset = (current_ms - center_ms) / ms_per_degree;
  return coordinate / cos(radians(angle_offset));
}


void setup() {
    Serial.begin(9600);
    Serial.flush();

    servo_x.attach(SERVO_X_PIN);
    servo_y.attach(SERVO_Y_PIN);
    /*
    servo_x.write(CENTER_ANGLE_X);
    servo_y.write(CENTER_ANGLE_Y);
    */
    servo_x.writeMicroseconds(CENTER_MS);
    servo_y.writeMicroseconds(CENTER_MS);

    delay(2000);
}

void loop() {

    if (Serial.available() > 0) {
        data = Serial.readStringUntil('\n');

        // Извлечение координат
        x = data.substring(0, data.indexOf(',')).toFloat();
        y = data.substring(data.indexOf(',') + 1).toFloat();

        // Коррекция на перспективу
        /*Градусы
        x_persp = x / cos(radians(servo_x.read() - CENTER_ANGLE_X));
        y_persp = y / cos(radians(servo_y.read() - CENTER_ANGLE_Y));
        */

        /*Микросекунды*/
        x_persp = normalize_perspective_ms(x, servo_x.readMicroseconds(), CENTER_MS, MS_PER_DEGREE_X);
        y_persp = normalize_perspective_ms(y, servo_y.readMicroseconds(), CENTER_MS, MS_PER_DEGREE_Y);

        out_x = reg_x.upd(x_persp);
        out_y = reg_y.upd(y_persp);

        /*
        angle_x = constrain(CENTER_ANGLE_X + out_x, MIN_ANGLE_X, MAX_ANGLE_X);
        angle_y = constrain(CENTER_ANGLE_Y + out_y * -1, MIN_ANGLE_Y, MAX_ANGLE_Y);
  
        servo_x.write(angle_x);
        servo_y.write(angle_y);
        */

        ms_x = constrain(CENTER_MS + out_x, MIN_MS, MAX_MS);  
        ms_y = constrain(CENTER_MS + out_y * -1, MIN_MS, MAX_MS);

        servo_x.writeMicroseconds(ms_x);
        servo_y.writeMicroseconds(ms_y);
    }
    delay(1);
}
