#pragma once
#include <Arduino.h>
#include "interruptEnum.h"

// ======================================================
// VELOCIDAD POR ENCODER
//
// - callbackISR() cuenta los flancos del encoder y anota
//   el micros() del último.
// - calcularVelocidad() mide el TIEMPO entre flancos, no
//   cuántos entraron en la ventana: con 20 ranuras a
//   30 RPM entran ~1 cada 100 ms, y contarlos daba saltos
//   de 30 RPM. Toma los flancos nuevos y el tiempo desde
//   el último flanco de la medición anterior hasta el
//   último de ésta: la resolución es la de micros().
// - Si en la ventana no llegó ningún flanco se mantiene
//   la velocidad anterior, salvo que el tiempo sin
//   flancos ya indique que es menor. Sin flancos durante
//   TIMEOUT_PARADO_US se considera parado.
// - El resultado se promedia con las 2 mediciones
//   anteriores, con más peso en la última.
// - attachInterrupt sólo acepta funciones sin
//   parámetros: begin() guarda el objeto en un puntero
//   estático y una función estática le pasa la
//   interrupción. Por eso hay un solo objeto de la clase
//   (el Uno/Nano tiene 2 interrupciones externas: una
//   para el encoder y otra para el ultrasonido).
// ======================================================

// Flancos que se cuentan: con Change cada ranura da 2
// (subida y bajada). Para medir tiempos conviene Rising:
// ranura y diente no miden lo mismo, y con Change los
// intervalos alternan entre corto y largo
enum TipoDeDeteccionEncoder{
    Rising,
    Change
};

class SensorVelocidad {
private:
    static const unsigned int RANURAS_POR_VUELTA = 20;

    // Flancos más juntos que esto son ruido del sensor
    // óptico: a 90 RPM con Rising hay uno cada 33 ms
    static const unsigned long MIN_INTERVALO_US = 2000;

    // Sin flancos durante esto: parado (por debajo de
    // ~6 RPM no se puede medir con 20 ranuras)
    static const unsigned long TIMEOUT_PARADO_US = 500000;

    // Pesos de la media: medición nueva, anterior, anteúltima
    static constexpr float PESO_NUEVA = 3.0;
    static constexpr float PESO_ANTERIOR = 2.0;
    static constexpr float PESO_ANTEULTIMA = 1.0;

    interrupt interrupcion_utilizada;
    TipoDeDeteccionEncoder  tipoflanco;
    // volatile: los cambia la ISR
    volatile long encoder_count;
    volatile unsigned long ultimo_flanco_us;
    int pin;

    // Último flanco de la medición anterior: la próxima
    // mide desde acá
    unsigned long referencia_us;
    // Parado o recién arrancado: el próximo flanco sólo
    // sirve de referencia
    bool sin_referencia;

    // Las 2 mediciones anteriores (sin promediar)
    float rpm_anterior;
    float rpm_anteultima;

    // Media ponderada (RPM)
    float velocidad;

    static SensorVelocidad* instancia;

    static void isr();
public:
    // Constructor por defecto
    SensorVelocidad(int,interrupt,TipoDeDeteccionEncoder);

    // Constructor de copia
    // (= delete: la ISR sólo llama al objeto registrado
    // en begin(), una copia no contaría flancos)
    SensorVelocidad(const SensorVelocidad&) = delete;

    // Constructor de movimiento
    SensorVelocidad(SensorVelocidad&&) noexcept = delete;

    // Operador de asignacion por copia
    SensorVelocidad& operator=(const SensorVelocidad&) = delete;

    // Operador de asignacion por movimiento
    SensorVelocidad& operator=(SensorVelocidad&&) noexcept = delete;

    // Destructor
    ~SensorVelocidad() = default;

    // Configura el pin y engancha la interrupción. Va en
    // setup(): los constructores globales corren antes de
    // que Arduino inicialice el hardware
    void begin();

    void calcularVelocidad();

    // RPM del último calcularVelocidad()
    float getVelocidad() const;

    // micros() del último flanco: después de una parada,
    // cuándo se movió el eje por última vez
    unsigned long getUltimoFlancoUs() const;

    void callbackISR();

};

// --- Implementaciones fuera de la clase ---
SensorVelocidad* SensorVelocidad::instancia = nullptr;

// Constructor por defecto
inline SensorVelocidad::SensorVelocidad(int pin,interrupt interrupcion,TipoDeDeteccionEncoder tipo)
    : interrupcion_utilizada(interrupcion),
      tipoflanco(tipo),
      encoder_count(0),
      ultimo_flanco_us(0),
      pin(pin),
      referencia_us(0),
      sin_referencia(true),
      rpm_anterior(0),
      rpm_anteultima(0),
      velocidad(0)
{
}

inline void SensorVelocidad::isr()
{
    instancia->callbackISR();
}

inline void SensorVelocidad::begin()
{
    pinMode(pin, INPUT_PULLUP);

    encoder_count = 0;
    ultimo_flanco_us = micros();
    sin_referencia = true;

    instancia = this;

    attachInterrupt(
        interrupcion_utilizada,
        isr,
        tipoflanco == Change ? CHANGE : RISING
    );
}

inline void SensorVelocidad::calcularVelocidad()
{
    // Copia y reinicio sin que la ISR cambie los valores
    // a mitad de camino (en el AVR son 4 bytes)
    noInterrupts();
    long flancos = encoder_count;
    encoder_count = 0;
    unsigned long ultimo = ultimo_flanco_us;
    interrupts();

    unsigned long ahora = micros();

    float flancos_por_vuelta =
        RANURAS_POR_VUELTA * (tipoflanco == Change ? 2 : 1);

    float rpm_nueva;

    if (flancos > 0 && !sin_referencia) {

        // `flancos` intervalos entre la referencia y el
        // último flanco
        rpm_nueva =
            flancos * 60.0e6
            / (flancos_por_vuelta * (float)(ultimo - referencia_us));

        referencia_us = ultimo;

    } else if (flancos > 0) {

        // Primer flanco después de estar parado: todavía
        // no hay intervalo que medir
        rpm_nueva = 0;
        referencia_us = ultimo;
        sin_referencia = false;

    } else if (sin_referencia || ahora - referencia_us > TIMEOUT_PARADO_US) {

        rpm_nueva = 0;
        sin_referencia = true;

    } else {

        // El próximo flanco todavía no llegó: la velocidad
        // es, como mucho, la que daría si llegara ahora
        float cota =
            60.0e6 / (flancos_por_vuelta * (float)(ahora - referencia_us));

        rpm_nueva = min(rpm_anterior, cota);
    }

    velocidad =
        (
            PESO_NUEVA * rpm_nueva
            + PESO_ANTERIOR * rpm_anterior
            + PESO_ANTEULTIMA * rpm_anteultima
        )
        / (PESO_NUEVA + PESO_ANTERIOR + PESO_ANTEULTIMA);

    rpm_anteultima = rpm_anterior;
    rpm_anterior = rpm_nueva;
};

inline float SensorVelocidad::getVelocidad() const
{
    return velocidad;
};

inline unsigned long SensorVelocidad::getUltimoFlancoUs() const
{
    // 4 bytes que cambia la ISR
    noInterrupts();
    unsigned long ultimo = ultimo_flanco_us;
    interrupts();

    return ultimo;
};

inline void SensorVelocidad::callbackISR()
{
    unsigned long ahora = micros();

    if (ahora - ultimo_flanco_us < MIN_INTERVALO_US) {
        return;
    }

    ultimo_flanco_us = ahora;
    encoder_count++;
};
