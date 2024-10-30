#include <Servo.h> // https://github.com/Dlloydev/ESP32-ESP32S2-AnalogWrite
#define NB_SERVOS 4

Servo servo_lib_handler = Servo();

struct PersistentServo
{

    PersistentServo() = default;
    PersistentServo(uint16_t pin, int16_t angle, int16_t speed, double easing_constant) 
    : pin(pin), angle(angle), speed(speed), easing_constant(easing_constant)
    {
        pinMode(pin, OUTPUT);
        servo_lib_handler.attach(pin);
    }
    PersistentServo(uint16_t pin): PersistentServo(pin, 100, 100, 0.8) {}

    uint16_t pin;
    int16_t angle;
    int16_t speed;

    // When easing constant (ke) < 1.0, return value is normalized, when 1.0, returns pulse width (μs)
    // ke = 0.0 is linear, between 0.0 and 1.0 is tunable sigmoid, 1.0 is normal response
    // Normalized Tunable Sigmoid: https://www.desmos.com/calculator/ejkcwglzd1
    double easing_constant;

    void update()
    {
        servo_lib_handler.write(pin, angle, speed, easing_constant);
    }
};


void updateServos(PersistentServo** myPersistentServos)
{
    for (int i = 0; i < NB_SERVOS; i++)
    {
        myPersistentServos[i]->update();
    }
}