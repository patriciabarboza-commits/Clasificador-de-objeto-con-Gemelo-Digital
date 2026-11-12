#pragma once
#include <Arduino.h>
#include "interruptEnum.h"

// ======================================================
// ULTRASONIDO SIN BLOQUEAR
//
// - callAsyncRead() dispara el trigger y vuelve enseguida
//   (pulseIn bloqueaba hasta 25 ms).
// - El eco se mide por interrupción CHANGE en echo_pin:
//   la subida marca el inicio y la bajada el final.
// - getDistancia() pasa a cm la última duración medida.
//   -1 = sin lectura.
// - Un solo objeto de la clase: igual que en
//   SensorVelocidad, la ISR llama al objeto registrado
//   en begin().
// ======================================================

class SensorDistancia {
private:
    // Eco más largo que se acepta (~4.3 m, como el
    // timeout de pulseIn en cinta.ino)
    static const unsigned long TIMEOUT_ECO_US = 25000;

    interrupt interrupcion_utilizada;
    float distancia;
    // volatile: los cambia la ISR
    volatile bool is_async_reading = false;
    volatile bool lectura_nueva = false;
    volatile unsigned long duracion_eco_us = 0;
    volatile unsigned long fin_eco_us = 0;
    // Sólo lo usa la ISR
    unsigned long inicio_eco_us = 0;
    // micros() del eco que dio la distancia actual
    unsigned long momento_lectura_us = 0;
    int trigger_pin,echo_pin;

    static SensorDistancia* instancia;

    static void isr();
public:
    // Constructor por defecto
    SensorDistancia(int,int,interrupt);

    // Constructor de copia
    SensorDistancia(const SensorDistancia&) = delete;

    // Constructor de movimiento
    SensorDistancia(SensorDistancia&&) noexcept = delete;

    // Operador de asignacion por copia
    SensorDistancia& operator=(const SensorDistancia&) = delete;

    // Operador de asignacion por movimiento
    SensorDistancia& operator=(SensorDistancia&&) noexcept = delete;

    // Destructor
    ~SensorDistancia() = default;

    // Configura los pines y engancha la interrupción. Va
    // en setup(): los constructores globales corren antes
    // de que Arduino inicialice el hardware
    void begin();

    float getDistancia();

    // micros() en que terminó el eco de la última
    // distancia que devolvió getDistancia()
    unsigned long getMomentoLecturaUs() const;

    void callAsyncRead();

    void callbackISR();

};

// --- Implementaciones fuera de la clase ---
SensorDistancia* SensorDistancia::instancia = nullptr;

// Constructor por defecto
inline SensorDistancia::SensorDistancia(int trigger,int echo,interrupt interrupcion)
    : interrupcion_utilizada(interrupcion),
      distancia(-1),
      trigger_pin(trigger),
      echo_pin(echo)
{
}

inline void SensorDistancia::isr()
{
    instancia->callbackISR();
}

inline void SensorDistancia::begin()
{
    pinMode(trigger_pin, OUTPUT);
    digitalWrite(trigger_pin, LOW);

    pinMode(echo_pin, INPUT);

    instancia = this;

    attachInterrupt(interrupcion_utilizada, isr, CHANGE);
}

inline float SensorDistancia::getDistancia()
{
    if (lectura_nueva) {

        // Copia sin que la ISR cambie la duración a mitad
        // de camino (en el AVR son 4 bytes)
        noInterrupts();
        unsigned long duracion = duracion_eco_us;
        momento_lectura_us = fin_eco_us;
        lectura_nueva = false;
        interrupts();

        // Eco demasiado largo: no hay nada delante
        if (duracion > TIMEOUT_ECO_US) {
            distancia = -1;
        } else {
            distancia = duracion * 0.0343 / 2.0;
        }
    }

    return distancia;
};

inline unsigned long SensorDistancia::getMomentoLecturaUs() const
{
    return momento_lectura_us;
};

inline void SensorDistancia::callAsyncRead() {

    // La medición anterior no terminó: el sensor no
    // respondió (desconectado o eco perdido)
    if (is_async_reading) {
        distancia = -1;
    }

    is_async_reading = true;

    digitalWrite(trigger_pin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigger_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger_pin, LOW);
};

inline void SensorDistancia::callbackISR() {

    unsigned long ahora = micros();

    if (digitalRead(echo_pin) == HIGH) {

        // Flanco de subida: empieza el eco
        inicio_eco_us = ahora;

    } else if (is_async_reading) {

        // Flanco de bajada: termina el eco
        duracion_eco_us = ahora - inicio_eco_us;
        fin_eco_us = ahora;
        lectura_nueva = true;
        is_async_reading = false;
    }
};

/**
 *
 *
 * const int pinTrig = 9;
const int pinEcho = 2; // Pin con soporte de interrupción

// Variables compartidas con la interrupción deben ser volatile
volatile unsigned long tiempoInicio = 0;
volatile unsigned long duracionPulso = 0;
volatile bool nuevaLecturaDisponible = false;

// Función de interrupción ISR (debe ser muy corta y rápida)
void manejarInterrupcionEcho() {
  if (digitalRead(pinEcho) == HIGH) {
    tiempoInicio = micros(); // Flanco de subida: inicia el pulso
  } else {
    duracionPulso = micros() - tiempoInicio; // Flanco de bajada: finaliza el pulso
    nuevaLecturaDisponible = true;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(pinTrig, OUTPUT);
  pinMode(pinEcho, INPUT);

  // Configuramos la interrupción para detectar cualquier cambio (RISING y FALLING)
  attachInterrupt(digitalPinToInterrupt(pinEcho), manejarInterrupcionEcho, CHANGE);
}

void loop() {
  // Lanzar el trigger cada 100 ms de forma no bloqueante usando millis()
  static unsigned long ultimoDisparo = 0;
  if (millis() - ultimoDisparo >= 100) {
    ultimoDisparo = millis();

    digitalWrite(pinTrig, LOW);
    delayMicroseconds(2);
    digitalWrite(pinTrig, HIGH);
    delayMicroseconds(10);
    digitalWrite(pinTrig, LOW);
  }

  // Si la interrupción capturó un eco completo, procesamos el dato sin bloquear
  if (nuevaLecturaDisponible) {
    // Desactivamos temporalmente interrupciones para leer la variable de forma segura
    noInterrupts();
    unsigned long duracion = duracionPulso;
    nuevaLecturaDisponible = false;
    interrupts();

    float distancia = (duracion * 0.0343) / 2.0;

    Serial.print("Distancia asíncrona: ");
    Serial.print(distancia);
    Serial.println(" cm");
  }

  // Aquí puedes ejecutar otras tareas concurrentemente sin que el sensor detenga el programa
}
 */
