#include <Servo.h> // https://github.com/Dlloydev/ESP32-ESP32S2-AnalogWrite
#define NB_SERVOS 4

struct PersistentServo
{
    PersistentServo() = default;
    PersistentServo(uint16_t pin, int16_t angle, int16_t speed) 
    : pin(pin), angle(angle), speed(speed)
    {
        pinMode(pin, OUTPUT);
        Servo espServo = Servo();
        espServo.attach(pin);
        easing_constant = 0.8;
    }
    PersistentServo(uint16_t pin)
    {
      PersistentServo(pin, 100, 100);
    }

    uint16_t pin;
    int16_t angle;
    int16_t speed;
    Servo espServo;

    // When easing constant (ke) < 1.0, return value is normalized, when 1.0, returns pulse width (μs)
    // ke = 0.0 is linear, between 0.0 and 1.0 is tunable sigmoid, 1.0 is normal response
    // Normalized Tunable Sigmoid: https://www.desmos.com/calculator/ejkcwglzd1
    double easing_constant;
};

void updateServos(PersistentServo** myPersistentServos)
{
    for (int i = 0; i < NB_SERVOS; i++)
    {
        PersistentServo* current_servo = myPersistentServos[i];

        current_servo->espServo.write(current_servo->pin, current_servo->angle, current_servo->speed, current_servo->easing_constant);
    }
}