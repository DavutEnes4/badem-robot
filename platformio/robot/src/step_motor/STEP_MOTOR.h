#ifndef STEP_MOTOR_H
#define STEP_MOTOR_H

#include <Arduino.h>

/**
 * @brief Motor class
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
    step_motor(int enable_pin, int dir_pin, int pwm_pin, int step_period, int period);
    
    /**
     * @brief Motoru çalıştırır
     *
     * Bu fonksiyon, motorun çalışmasını sağlar.
     */
    void move();

    /**
     * @brief Motorun durmasını sağlar
     *
     * Bu fonksiyon, motorun durmasını sağlar.
     */
    void stop();

    /**
     * @brief Motorun çalışıp çalışmadığını kontrol eder
     *
     * Bu fonksiyon, motorun çalışıp çalışmadığını kontrol eder.
     *
     * @return Motor çalışıyorsa true, çalışmıyorsa false.
     */
    bool is_moving();

    /**
     * @brief Motorun hızını ayarlar
     *
     * Bu fonksiyon, motorun hızını ayarlar.
     *
     * @param period Motorun çalışma periyodu (gerekli motor frekansını belirler).
     */
    void set_speed(int new_period);

    /**
     * @brief Motorun ileri veya geri gitmesini sağlar
     *
     * Bu fonksiyon, motorun ileri veya geri gitmesini sağlar.
     *
     * @param direction Motorun ileri gitmesi durumunda true, geri gitmesi durumunda false.
     */
    void set_direction(bool direction);
};

#endif
