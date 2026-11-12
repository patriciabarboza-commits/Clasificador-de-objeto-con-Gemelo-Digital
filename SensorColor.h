#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>

// ======================================================
// SENSOR DE COLOR TCS34725 (librería Adafruit TCS34725)
//
// Va por I2C, así que no hay pines que definir: en el
// Uno/Nano SDA = A4 y SCL = A5. El LED del módulo
// ilumina el objeto.
// ======================================================

struct RGB {
    // uint16_t: cada canal llega hasta 65535 con
    // integraciones largas (no entra en un int del AVR)
    uint16_t r;
    uint16_t g;
    uint16_t b;
};

class SensorColor {
private:
    Adafruit_TCS34725 tcs;
    // false si begin() no encontró el sensor
    bool detectado;
public:
    // Constructor por defecto
    SensorColor();

    // Constructor de copia
    SensorColor(const SensorColor&) = delete;

    // Constructor de movimiento
    SensorColor(SensorColor&&) noexcept = delete;

    // Operador de asignacion por copia
    SensorColor& operator=(const SensorColor&) = delete;

    // Operador de asignacion por movimiento
    SensorColor& operator=(SensorColor&&) noexcept = delete;

    // Destructor
    ~SensorColor() = default;

    // Inicia el I2C y el sensor: false si no responde.
    // Va en setup(): Wire no está listo durante los
    // constructores globales
    bool begin();

    // Lectura cruda de los canales (0,0,0 sin sensor).
    // Sin bloquear: devuelve la última integración (cada
    // 50 ms) en ~1 ms de I2C
    RGB getRGB();

};

// --- Implementaciones fuera de la clase ---
// Constructor por defecto
inline SensorColor::SensorColor()
    : tcs(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X),
      detectado(false)
{
}

inline bool SensorColor::begin()
{
    detectado = tcs.begin();

    return detectado;
}

inline RGB SensorColor::getRGB()
{
    RGB color = {0, 0, 0};

    // Sin sensor no se llama a la librería: reintentaría
    // el begin() en cada lectura
    if (!detectado) {
        return color;
    }

    // getRawData() lee los registros y después espera
    // 51 ms sin hacer nada: frenaba el loop (y el PID)
    // justo al pasar cada objeto. Después de begin() el
    // sensor integra solo y sin parar, así que los
    // registros siempre tienen la última medición
    color.r = tcs.read16(TCS34725_RDATAL);
    color.g = tcs.read16(TCS34725_GDATAL);
    color.b = tcs.read16(TCS34725_BDATAL);

    return color;
};

/*

Codigo de ejemplo
#include <Wire.h>
#include "Adafruit_TCS34725.h"


 * Inicialización con tiempo de integración y ganancia:
 * TCS34725_INTEGRATIONTIME_50MS: 2.4ms, 24ms, 50ms, 101ms, 154ms, 700ms
 * TCS34725_GAIN_1X: 1X, 4X, 16X, 60X

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void setup() {
  Serial.begin(115200);

  if (tcs.begin()) {
    Serial.println("Sensor TCS34725 detectado correctamente.");
  } else {
    Serial.println("Error: No se encontró el sensor TCS34725. Revisa las conexiones.");
    while (1);
  }
}

void loop() {
  uint16_t r, g, b, c;
  uint16_t colorTemp, lux;

  // Lectura de canales RAW (Red, Green, Blue, Clear)
  tcs.getRawData(&r, &g, &b, &c);

  // Cálculo opcional de temperatura de color y nivel de iluminancia
  colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  lux = tcs.calculateLux(r, g, b);

  Serial.print("R: "); Serial.print(r);
  Serial.print(" | G: "); Serial.print(g);
  Serial.print(" | B: "); Serial.print(b);
  Serial.print(" | Clear: "); Serial.print(c);
  Serial.print(" | Temp (K): "); Serial.print(colorTemp);
  Serial.print(" | Lux: "); Serial.println(lux);

  delay(500);
}

*/
