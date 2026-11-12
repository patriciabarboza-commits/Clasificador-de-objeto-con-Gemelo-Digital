#pragma once
#include <Arduino.h>
#include <Servo.h>

// ======================================================
// SERVO DE RECHAZO
//
// Saca de la cinta los objetos rechazados. En reposo el
// brazo deja pasar; al rechazar se cruza en la cinta.
// Igual que ServoClassificador, el attach va en begin()
// ======================================================

class ServoRechazo {
private:
    Servo servo;
    int pin;
    int angle_reposo;
    int angle_rechazo;
public:
    // Constructor: pin, ángulo de reposo, ángulo de
    // rechazo (0-180)
    ServoRechazo(int,int,int);

    // Constructor de copia
    ServoRechazo(const ServoRechazo&) = delete;

    // Constructor de movimiento
    ServoRechazo(ServoRechazo&&) noexcept = delete;

    // Operador de asignacion por copia
    ServoRechazo& operator=(const ServoRechazo&) = delete;

    // Operador de asignacion por movimiento
    ServoRechazo& operator=(ServoRechazo&&) noexcept = delete;

    // Destructor
    ~ServoRechazo() = default;

    // Engancha el servo y lo deja en reposo. Va en
    // setup()
    void begin();

    void reposo() noexcept;
    void rechazar() noexcept;

};

// --- Implementaciones fuera de la clase ---
// Constructor
inline ServoRechazo::ServoRechazo(int pin,int anguloReposo,int anguloRechazo)
    : pin(pin),
      angle_reposo(anguloReposo),
      angle_rechazo(anguloRechazo)
{
}

inline void ServoRechazo::begin()
{
    servo.attach(pin);

    reposo();
}

inline void ServoRechazo::reposo() noexcept
{
    servo.write(angle_reposo);
}

inline void ServoRechazo::rechazar() noexcept
{
    servo.write(angle_rechazo);
}
