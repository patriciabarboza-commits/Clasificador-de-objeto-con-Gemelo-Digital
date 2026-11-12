#pragma once
#include <Arduino.h>
#include <Servo.h>

// ======================================================
// SERVO CLASIFICADOR
//
// Servo.h usa el Timer1: se pierde el PWM de los pines
// 9 y 10. El attach va en begin(): init() reconfigura el
// Timer1 después de los constructores globales y
// desarmaría el servo.
// ======================================================

class ServoClassificador {
private:
    Servo servo;
    int pin;
    int angle_for_position_A;
    int angle_for_position_B;
    int angle_for_center;
public:
    // Constructor: pin, ángulo A, ángulo B, ángulo del
    // centro (0-180)
    ServoClassificador(int,int,int,int);

    // Constructor de copia
    ServoClassificador(const ServoClassificador&) = delete;

    // Constructor de movimiento
    ServoClassificador(ServoClassificador&&) noexcept = delete;

    // Operador de asignacion por copia
    ServoClassificador& operator=(const ServoClassificador&) = delete;

    // Operador de asignacion por movimiento
    ServoClassificador& operator=(ServoClassificador&&) noexcept = delete;

    // Destructor
    ~ServoClassificador() = default;

    // Engancha el servo y lo lleva al centro. Va en
    // setup()
    void begin();

    void goToCenter() noexcept;
    void goToPosA() noexcept;
    void goToPosB() noexcept;

};

// --- Implementaciones fuera de la clase ---
// Constructor
inline ServoClassificador::ServoClassificador(int pin,int anguloA,int anguloB,int anguloCentro)
    : pin(pin),
      angle_for_position_A(anguloA),
      angle_for_position_B(anguloB),
      angle_for_center(anguloCentro)
{
}

inline void ServoClassificador::begin()
{
    servo.attach(pin);

    goToCenter();
}

inline void ServoClassificador::goToCenter() noexcept
{
    servo.write(angle_for_center);
}

inline void ServoClassificador::goToPosA() noexcept
{
    servo.write(angle_for_position_A);
}

inline void ServoClassificador::goToPosB() noexcept
{
    servo.write(angle_for_position_B);
}
