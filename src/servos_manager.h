#include <Servo.h> // https://github.com/Dlloydev/ESP32-ESP32S2-AnalogWrite
#define NB_SERVOS 7

Servo servo_lib_handler = Servo();

struct PersistentServo
{

    PersistentServo() = default;
    PersistentServo(uint16_t pin, int16_t angle, int16_t speed, double easing_constant, int16_t min_angle, int16_t max_angle)
        : pin(pin), angle(angle), speed(speed), easing_constant(easing_constant), min_angle(min_angle), max_angle(max_angle)
    {
        pinMode(pin, OUTPUT);
        const int min_angle_microseconds = 500; // To use the servo to its full potential
        const int max_angle_microseconds = 2500;
        servo_lib_handler.attach(pin, min_angle_microseconds, max_angle_microseconds);
    }
    PersistentServo(uint16_t pin, int16_t angle, int16_t speed, double easing_constant) : PersistentServo(pin, angle, speed, easing_constant, 0, 65000) {}
    PersistentServo(uint16_t pin) : PersistentServo(pin, 100, 100, 0.8) {}

    uint16_t pin;
    int16_t angle;
    int16_t speed;
    int16_t min_angle;
    int16_t max_angle;

    // When easing constant (ke) < 1.0, return value is normalized, when 1.0, returns pulse width (μs)
    // ke = 0.0 is linear, between 0.0 and 1.0 is tunable sigmoid, 1.0 is normal response
    // Normalized Tunable Sigmoid: https://www.desmos.com/calculator/ejkcwglzd1
    double easing_constant;

    void update()
    {
        int16_t l_angle = max(angle, min_angle);
        l_angle = min(l_angle, max_angle);
        servo_lib_handler.write(pin, l_angle, speed, easing_constant);
    }
};

void updateServos(PersistentServo **myPersistentServos)
{
    for (int i = 0; i < NB_SERVOS; i++)
    {
        myPersistentServos[i]->update();
    }
}