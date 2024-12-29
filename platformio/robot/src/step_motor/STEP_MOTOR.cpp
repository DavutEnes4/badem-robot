#include "STEP_MOTOR.h"

step_motor::step_motor(int enable_pin, int dir_pin, int pwm_pin, int step_period, unsigned long period)
{
    this->enable_pin = enable_pin;
    this->dir_pin = dir_pin;
    this->pwm_pin = pwm_pin;
    this->step_period = step_period;
    this->period = period;
    this->previous_time = 0;
    this->forward = true;

    pinMode(enable_pin, OUTPUT);
    pinMode(dir_pin, OUTPUT);
    pinMode(pwm_pin, OUTPUT);

    digitalWrite(enable_pin, HIGH); // Motoru başlangıçta durdur
}

void step_motor::move()
{
    unsigned long current_time = micros();
    if ((current_time - previous_time) >= period)
    {
        digitalWrite(enable_pin, LOW);  // Motoru aktif et
        digitalWrite(pwm_pin, HIGH);   // PWM çıkışı ver
        delayMicroseconds(step_period);
        digitalWrite(pwm_pin, LOW);
        previous_time = current_time; // Zamanı güncelle
    }
}

void step_motor::stop()
{
    digitalWrite(pwm_pin, LOW);
    digitalWrite(enable_pin, HIGH); // Motoru devre dışı bırak
}

bool step_motor::is_moving()
{
    return digitalRead(enable_pin) == LOW;
}

void step_motor::set_speed(unsigned long new_period)
{
    this->period = new_period;
}

void step_motor::set_direction(bool direction)
{
    this->forward = direction;
    digitalWrite(dir_pin, direction ? HIGH : LOW);
}
