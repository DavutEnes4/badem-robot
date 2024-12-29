#include <Arduino.h>

class step_motor
{
private:
    int enable_pin;              
    int dir_pin;                 
    int pwm_pin;                 
    int step_period;             
    int period;                   
    long previous_time; 
    bool forward;

    step_motor(int enable_pin, int dir_pin, int pwm_pin, int step_period, int period)
    {
        this->enable_pin = enable_pin;
        this->dir_pin = dir_pin;
        this->pwm_pin = pwm_pin;
        this->period = period;
        this->step_period = step_period;
        this->previous_time = 0;
        this->forward = true;
    }
    void move()
    {
        unsigned long current_time = micros();
        if (current_time - this->previous_time > this->period)
        {
            digitalWrite(this->enable_pin, LOW);
            digitalWrite(this->pwm_pin, HIGH);
            delayMicroseconds(step_period);
            digitalWrite(this->pwm_pin, LOW);
            this->previous_time = current_time;
        }
    }
    void stop()
    {
        digitalWrite(this->pwm_pin, LOW);
        digitalWrite(this->enable_pin, HIGH);
    }
    bool is_moving()
    {
        return digitalRead(this->enable_pin) == LOW;
    }
    void set_speed(int new_period)
    {
        this->period = new_period;
    }
    void set_direction(bool direction)
    {
        this->forward = direction;
        digitalWrite(this->dir_pin, direction ? HIGH : LOW);
    }
};