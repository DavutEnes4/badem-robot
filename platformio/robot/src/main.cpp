#include <Arduino.h>
#include <STEP_MOTOR.h>

#define motor_right_enable 23
#define motor_right_dir_pin 25
#define motor_right_pwm 2
#define motor_left_enable 22
#define motor_left_dir_pin 24
#define motor_left_pwm 3

step_motor motor_right(motor_right_enable, motor_right_dir_pin, motor_right_pwm, 5, 20);
step_motor motor_left(motor_left_enable, motor_left_dir_pin, motor_left_pwm, 5, 20);

void setup()
{
  Serial.begin(115200);
}

void loop()
{
  fSerialRead();
  fMotorMove();
}

void fSerialRead()
{
  if (Serial.available())
  {
    char command = Serial.read();

    switch (command)
    {
    case 'w': // Düz ileri
      motor_right.set_direction(true);
      motor_left.set_direction(true);
      break;

    case 's': // Düz geri
      motor_right.set_direction(false);
      motor_left.set_direction(false);
      break;

    case 'a': // Sol (olduğu yerde dönme)
      motor_right.set_direction(true);
      motor_left.set_direction(false);
      break;

    case 'd': // Sağ (olduğu yerde dönme)
      motor_right.set_direction(false);
      motor_left.set_direction(true);
      break;

    case 'x': // Dur
      motor_right.stop();
      motor_left.stop();
      break;

    default:
      Serial.println("Geçersiz komut!");
      break;
    }
  }
}

void fMotorMove()
{
  if (motor_right.is_moving() || motor_left.is_moving())
  {
    motor_right.move();
    motor_left.move();
  }
}