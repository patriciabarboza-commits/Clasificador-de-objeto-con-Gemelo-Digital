#pragma once

#include <Arduino.h>


// ======================================================
// PID CON FEEDFORWARD
//
// salida = feedforward + P + I + D
//
// - El feedforward es el PWM estimado para el setpoint
//   (lo calcula el sketch con el modelo del motor).
// - El PID sólo corrige la diferencia, así no tiene que
//   "cargar" la integral desde cero en cada arranque.
// - El derivativo se calcula sobre la medición para
//   evitar el salto al cambiar el setpoint.
// - La integral se guarda ya multiplicada por Ki para
//   que cambiar Ki en marcha no produzca saltos.
// ======================================================

class PID {

public:

  enum class Mode {

    MANUAL,

    AUTOMATIC
  };


  enum class Direction {

    DIRECT,

    REVERSE
  };


private:

  double* input;

  double* output;

  double* setpoint;


  double kp;

  double ki;

  double kd;


  double terminoI;

  double inputAnterior;

  double feedforward;


  double outputMin;

  double outputMax;


  Mode mode;

  Direction direction;


  bool primerCalculo;


public:


  PID(
    double* input,
    double* output,
    double* setpoint,
    double kp,
    double ki,
    double kd,
    Direction direction = Direction::DIRECT
  )
    :

      input(input),

      output(output),

      setpoint(setpoint),

      kp(kp),

      ki(ki),

      kd(kd),

      terminoI(0.0),

      inputAnterior(0.0),

      feedforward(0.0),

      outputMin(0.0),

      outputMax(255.0),

      mode(Mode::MANUAL),

      direction(direction),

      primerCalculo(true)

  {

  }


  // ==================================================
  // COMPUTE
  //
  // dt: tiempo real entre muestras (s). Lo mide el
  // sketch, así el primer cálculo después de un reset
  // ya produce una salida válida.
  // ==================================================

  void Compute(
    double dt
  ) {

    if (
      !input
      ||
      !output
      ||
      !setpoint
    ) {

      return;
    }


    if (
      mode
      ==
      Mode::MANUAL
    ) {

      return;
    }


    if (
      dt <= 0
    ) {

      return;
    }


    *output =
      calcular(
        *setpoint,
        *input,
        dt
      );
  }


  // ==================================================
  // PID
  // ==================================================

  double calcular(
    double referencia,
    double valorActual,
    double dt
  ) {

    double signo =
      (direction == Direction::REVERSE) ? -1.0 : 1.0;


    // ==========================================
    // ERROR
    // ==========================================

    double error =
      signo * (referencia - valorActual);


    // ==========================================
    // PROPORCIONAL
    // ==========================================

    double P =
      kp * error;


    // ==========================================
    // DERIVATIVO (sobre la medición)
    // ==========================================

    double D = 0;

    if (
      !primerCalculo
    ) {

      D =
        -signo * kd * (valorActual - inputAnterior) / dt;
    }


    // ==========================================
    // INTEGRAL CON ANTI WINDUP
    // ==========================================

    double terminoINuevo =
      terminoI + ki * error * dt;


    double salidaProvisional =
      feedforward + P + terminoINuevo + D;


    if (

      (
        salidaProvisional <= outputMax
        &&
        salidaProvisional >= outputMin
      )

      ||

      (
        salidaProvisional > outputMax
        &&
        error < 0
      )

      ||

      (
        salidaProvisional < outputMin
        &&
        error > 0
      )

    ) {

      terminoI =
        terminoINuevo;
    }


    // ==========================================
    // SALIDA PID
    // ==========================================

    double salida =
      feedforward + P + terminoI + D;


    salida =
      constrain(
        salida,
        outputMin,
        outputMax
      );


    inputAnterior =
      valorActual;

    primerCalculo =
      false;


    return salida;
  }


  // ==================================================
  // RESET
  // ==================================================

  void reset() {

    terminoI =
      0;

    inputAnterior =
      0;

    primerCalculo =
      true;
  }


  // ==================================================
  // MODO
  // ==================================================

  void SetMode(
    Mode nuevoModo
  ) {


    if (
      nuevoModo
      ==
      mode
    ) {

      return;
    }


    if (
      nuevoModo
      ==
      Mode::AUTOMATIC
    ) {

      reset();
    }


    mode =
      nuevoModo;
  }


  // ==================================================
  // LÍMITES
  // ==================================================

  void SetOutputLimits(
    double minimo,
    double maximo
  ) {


    if (
      minimo
      >=
      maximo
    ) {

      return;
    }


    outputMin =
      minimo;

    outputMax =
      maximo;
  }


  // ==================================================
  // FEEDFORWARD
  // ==================================================

  void SetFeedforward(
    double valor
  ) {

    feedforward =
      valor;
  }


  // ==================================================
  // GANANCIAS
  // ==================================================

  void setKp(
    double valor
  ) {

    kp =
      valor;
  }


  void setKi(
    double valor
  ) {

    ki =
      valor;
  }


  void setKd(
    double valor
  ) {

    kd =
      valor;
  }
};