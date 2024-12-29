#ifndef STEP_MOTOR_H
#define STEP_MOTOR_H

#include <Arduino.h>

/**
 * @class step_motor
 * @brief Motorun kontrolünü sağlayan sınıf.
 *
 * Bu sınıf motorun hareketi, hızı ve yönünü kontrol etmek için kullanılan fonksiyonları içerir.
 */
class step_motor
{
private:
    int enable_pin;   ///< Motorun aktif olup olmadığını kontrol eden pin
    int dir_pin;      ///< Motorun yönünü belirleyen pin
    int pwm_pin;      ///< Motorun hızını ayarlayan PWM pin'i
    int step_period;  ///< Adım periyodu, her adım arasında geçen süreyi belirler
    unsigned long period; ///< Motorun çalışma periyodu
    unsigned long previous_time; ///< Son işlem zamanını tutar
    bool forward;     ///< Motorun ileri mi yoksa geri mi gittiğini belirtir

public:
    /**
     * @brief Yeni bir step_motor nesnesi oluşturur
     * 
     * Motorun kontrolünü sağlamak için gerekli pinler ve parametrelerle motoru başlatır.
     * 
     * @param enable_pin Motorun aktif olup olmayacağını kontrol eden pin
     * @param dir_pin Motorun yönünü belirleyen pin
     * @param pwm_pin Motorun hızını ayarlayan PWM pin'i
     * @param step_period Adım periyodu (her adım arasındaki süreyi belirler)
     * @param period Motorun çalışma periyodu (gerekli motor frekansını belirler)
     */
    step_motor(int enable_pin, int dir_pin, int pwm_pin, int step_period, unsigned long period);

    /**
     * @brief Motoru çalıştırır
     * 
     * Bu fonksiyon motoru çalıştırarak adım adım hareket etmesini sağlar.
     */
    void move();

    /**
     * @brief Motoru durdurur
     * 
     * Motoru durduran fonksiyondur. PWM çıkışı durdurulur.
     */
    void stop();

    /**
     * @brief Motorun çalışıp çalışmadığını kontrol eder
     * 
     * Motor aktifse true döner, değilse false döner.
     * 
     * @return Motor çalışıyorsa true, çalışmıyorsa false döner
     */
    bool is_moving();

    /**
     * @brief Motorun hızını ayarlar
     * 
     * Motorun çalışma hızını belirleyen fonksiyondur.
     * 
     * @param new_period Yeni çalışma periyodu
     */
    void set_speed(unsigned long new_period);

    /**
     * @brief Motorun yönünü belirler
     * 
     * Motoru ileri veya geri yönlendirir.
     * 
     * @param direction Motorun yönü (true = ileri, false = geri)
     */
    void set_direction(bool direction);
};

#endif
