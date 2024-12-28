#include <Arduino.h>

/**
 * @brief step_motor Sınıfı
 *
 *  Bu sınıf motor kontrollerini sağlar.
 */

class step_motor
{
private:
    int enable_pin;              // Enable pin
    int dir_pin;                 // Direction pin
    int pwm_pin;                 // PWM pin
    int step_period;             // Step period
    int period;                  // Period
    unsigned long previous_time; // Previous time
    bool forward;                // Forward
public:
    /**
     * @brief Yeni bir step_motor nesnesi oluşturur
     *
     * Bu fonksiyon, step_motor nesnesinin başlatılması için gerekli parametreleri alır ve motorun çalışabilmesi için gerekli pinleri ve ayarları yapılandırır.
     *
     * @param enable_pin Motorun aktif olup olmayacağını kontrol eden pin.
     * @param dir_pin Motorun yönünü belirleyen pin.
     * @param pwm_pin Motorun hızını ayarlayan PWM pin'i.
     * @param step_period Adım periyodu (her adım arasındaki süreyi belirler).
     * @param period Motorun çalışma periyodu (gerekli motor frekansını belirler).
     */
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
    /**
     * @brief Motoru çalıştırır
     *
     * Bu fonksiyon, motorun çalışmasını sağlar.
     */
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
    /**
     * @brief Motorun durmasını sağlar
     *
     * Bu fonksiyon, motorun durmasını sağlar.
     */
    void stop()
    {
        digitalWrite(this->pwm_pin, LOW);
        digitalWrite(this->enable_pin, HIGH);
    }
    /**
     * @brief Motorun çalışıp çalışmadığını kontrol eder
     *
     * Bu fonksiyon, motorun çalışıp çalışmadığını kontrol eder.
     *
     * @return Motor çalışıyorsa true, çalışmıyorsa false.
     */
    bool is_moving()
    {
        return digitalRead(this->enable_pin) == LOW;
    }
    /**
     * @brief Motorun hızını ayarlar
     *
     * Bu fonksiyon, motorun hızını ayarlar.
     *
     * @param period Motorun çalışma periyodu (gerekli motor frekansını belirler).
     */
    void set_speed(int new_period)
    {
        this->period = new_period;
    }
    /**
     * @brief Motorun ileri veya geri gitmesini sağlar
     *
     * Bu fonksiyon, motorun ileri veya geri gitmesini sağlar.
     *
     * @param direction Motorun ileri gitmesi durumunda true, geri gitmesi durumunda false.
     */
    void set_direction(bool direction)
    {
        this->forward = direction;
        digitalWrite(this->dir_pin, direction ? HIGH : LOW);
    }
};