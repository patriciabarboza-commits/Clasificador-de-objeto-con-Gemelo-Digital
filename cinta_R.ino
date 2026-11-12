#include <Arduino.h>
#include "SensorDistancia.h"
#include "SensorColor.h"
#include "SensorVelocidad.h"
#include "ServoClasificador.h"
#include "ServoRechazo.h"
#include "PID.h"

#define TRIGGER_PIN 11
#define ECHO_PIN 2

#define SPEED_PIN 3

// Puente H
#define PIN_IN1 4
#define PIN_IN2 5
#define PIN_ENA 6   // PWM

#define SERVO_PIN 9
// Servo.h ya le quita el PWM al pin 10 (Timer1): usarlo
// para el segundo servo no cuesta nada
#define SERVO_RECHAZO_PIN 10

// Pulsador de emergencia entre este pin y GND (pull-up
// interno). Es sólo el aviso al firmware: la parada de
// emergencia de verdad tiene que cortar la alimentación
// del motor por hardware, sin pasar por el Arduino.
// Sin pulsador conectado el pin queda en HIGH = normal
#define PIN_EMERGENCIA 7
#define EMERGENCIA_ACTIVA LOW

// Alarma local: se prende en PARADA_SEGURA aunque el
// gemelo no esté conectado
#define PIN_ALARMA LED_BUILTIN


#define TAMANO_MAXIMO_CINTA 40
#define DISTANCIA_MEDICION 15

// Debounce de presencia: el ultrasonido da lecturas
// sueltas falsas (-1 o el fondo, >100 cm) en medio del
// paso de un objeto, y valores sueltos dentro de la cinta
// estando vacía. Se cuenta una muestra por disparo:
//   objeto nuevo: MUESTRAS_PRESENCIA válidas seguidas
//   objeto que se fue: MUESTRAS_AUSENCIA inválidas seguidas
#define MUESTRAS_PRESENCIA 2
#define MUESTRAS_AUSENCIA 3

// Clasificación por color dominante. Ganancia por canal
// para que un objeto blanco dé R = G = B: el TCS34725 no
// tiene los tres canales igual de sensibles. Calibrar con
// TCS34725ColorSensorTest
#define GANANCIA_R 1.0
#define GANANCIA_G 1.0
#define GANANCIA_B 1.0
// Con menos luz reflejada (R+G+B) no se decide: DUDOSO.
// Medido en los registros: cinta vacía ~53, el objeto
// más oscuro ~110
#define BRILLO_MINIMO 80
// El canal dominante tiene que superar al segundo en esta
// fracción de R+G+B; si no, DUDOSO
#define MARGEN_COLOR 0.08

// Debounce de la clasificación (Anexo A, conteo de
// muestras consecutivas): la clase se confirma con
// N_DEBOUNCE lecturas seguidas iguales. Si en
// MAX_MUESTRAS_COLOR lecturas no se confirma: DUDOSO.
// Cada lectura es una integración nueva del sensor
// (50 ms), así que Δt = PERIODO_MEDICION_COLOR:
//   T_deb = (N_DEBOUNCE - 1) * Δt = 120 ms
//   peor caso (DUDOSO) = (MAX - 1) * Δt = 240 ms
#define N_DEBOUNCE 3
#define MAX_MUESTRAS_COLOR 5

#define SERVO_A_POSITION 45+20
#define SERVO_B_POSITION 135-20
#define SERVO_CENTER_POSITION 90

// Medir en la cinta con ServoTest. Lejos de 0 y 180: los
// servos baratos no llegan y quedan forzando
#define SERVO_RECHAZO_REPOSO 90
#define SERVO_RECHAZO_ACTIVO 0


// Deadline de la letra (Cuadro 5): muestreo de sensores
// y actualización del PID cada <= 100 ms. El loop puede
// atrasar cada tarea unos ms (delay(5), serial), por eso
// los períodos quedan por debajo.
// El HC-SR04 necesita >= 60 ms entre disparos
#define PERIODO_MEDICION_DISTANCIA 90
// Velocidad + PID
#define PERIODO_MEDICION_VELOCIDAD 50
// Mayor que la integración del TCS34725 (50 ms): cada
// lectura es una medición nueva
#define PERIODO_MEDICION_COLOR 60
// gemelo.py grafica 200 tramas = 10 s
#define PERIODO_RESPUESTA_SERIAL 50

// Invertir con el motor girando es un golpe para el motor
// y la cinta: después de invertir queda sin tensión
#define PAUSA_INVERSION_MS 1000

// Atasco: con este PWM la banda se mueve seguro; si
// igual la velocidad no pasa de VELOCIDAD_ATASCO durante
// TIEMPO_ATASCO_MS, está trabada. Calibrar: el PWM con
// el que arranca la banda vacía, con margen
#define PWM_ATASCO 150
#define VELOCIDAD_ATASCO 1.0
#define TIEMPO_ATASCO_MS 1000

// Lecturas seguidas en 0,0,0: el sensor de color no
// responde (con el LED encendido nunca da 0). 5 * 60 ms
#define LECTURAS_NULAS_FALLA 5

// Iguales que en gemelo.py
#define BAUDRATE 115200
#define RPM_MAX 90.0   // 100 % del comando V


// Valores que gemelo.py espera en ESTADO
enum EstadoMotor {
    PARADO = 0,
    ADELANTE = 1,
    REVERSA = 2
};

EstadoMotor estadoMotor = PARADO;


// ======================================================
// MÁQUINA DE ESTADOS (AUTOMATA en la trama)
//
//   IDLE --F/R--> RUN --objeto--> OBJETO_DETECTADO
//     --zona de medición--> CLASIFICANDO --debounce-->
//       DESVIAR_A (verde)  | DESVIAR_B (azul)
//       RECHAZAR (rojo)    | RECHAZAR_DUDOSO
//     --el objeto sale de la cinta--> RUN
//
//   S desde cualquiera de RUN...RECHAZAR_DUDOSO -> IDLE.
//   Al volver a arrancar sigue donde estaba el objeto
//   (estado con historia: tipoObjeto no se borra).
//
//   Falla desde cualquier estado -> PARADA_SEGURA:
//   motor frenado, servos quietos, alarma. Sólo sale con
//   el comando X (rearme) si la causa desapareció, y
//   vuelve a IDLE: hay que arrancar a mano.
//
// La parada segura es determinista y no depende del
// gemelo: las fallas las detecta el propio Arduino.
// ======================================================

enum EstadoSistema {
    IDLE = 0,
    RUN = 1,
    OBJETO_DETECTADO = 2,
    CLASIFICANDO = 3,
    DESVIAR_A = 4,
    DESVIAR_B = 5,
    RECHAZAR = 6,
    RECHAZAR_DUDOSO = 7,
    PARADA_SEGURA = 8
};

EstadoSistema estadoSistema = IDLE;

// Causa de la PARADA_SEGURA (FALLA en la trama)
enum Falla {
    SIN_FALLA = 0,
    FALLA_EMERGENCIA = 1,     // pulsador
    FALLA_HMI = 2,            // botón del gemelo (comando E)
    FALLA_ATASCO = 3,
    FALLA_SENSOR_COLOR = 4
};

Falla fallaActual = SIN_FALLA;

// Fallas provocadas a propósito para probar la parada
// segura (comando I<n>, INYECCION en la trama)
enum Inyeccion {
    SIN_INYECCION = 0,
    INYECTAR_SENSOR_COLOR = 1,   // el sensor lee 0,0,0
    INYECTAR_ATASCO = 2          // el encoder no ve giro
};

Inyeccion inyeccion = SIN_INYECCION;

// micros() en que se detectó la falla
unsigned long momentoFallaUs = 0;
// Desde la falla hasta el último flanco del encoder:
// cuánto tardó el motor en quedar quieto (deadline 200 ms)
unsigned long tiempoParadaMs = 0;

bool posibleAtasco = false;
unsigned long inicioAtasco = 0;
unsigned int lecturasNulas = 0;


// Velocidad pedida con V (RPM): se conserva al detener
double rpmElegida = 30.0;

// double: PID.h los lee y escribe por puntero.
// El setpoint vale 0 mientras el motor está parado
double setpoint = 0.0;
double salidaPID = 0.0;
double velocidad = 0.0;

// Iguales a KP_INICIAL, KI_INICIAL y KD_INICIAL de
// gemelo.py, que los cambia con K. Ajustados a mano en
// la cinta: con Kp 0.5 y Ki 1.5 no andaba bien
const double KP = 1.0;
const double KI = 0.4;
const double KD = 0.0;

const int PWM_MAX = 255;

int pwm = 0;


PID pid(
    &velocidad,
    &salidaPID,
    &setpoint,
    KP,
    KI,
    KD,
    PID::Direction::DIRECT);


// Valores que se mandan en OBJETO.
// VERDE va al lado A y AZUL al B (servo clasificador).
// ROJO y DUDOSO los saca el servo de rechazo: ante la duda
// se rechaza, para no contaminar los depósitos buenos.
// Mientras se clasifica sigue en ANTES_DE_MEDICION (el
// AUTOMATA dice CLASIFICANDO)
enum TipoObjeto {
    SIN_OBJETO = 0,
    ANTES_DE_MEDICION = 1,
    VERDE = 2,
    AZUL = 3,
    ROJO = 4,
    DUDOSO = 5
};

TipoObjeto tipoObjeto = SIN_OBJETO;

// Debounce de presencia (muestras del ultrasonido)
unsigned int muestrasPresente = 0;
unsigned int muestrasAusente = 0;

// Debounce en curso
bool clasificando = false;
TipoObjeto claseCandidata = SIN_OBJETO;
unsigned int muestrasIguales = 0;
unsigned int muestrasTotales = 0;
// Eco del ultrasonido que vio el objeto en la zona
unsigned long momentoDeteccionUs = 0;

// Última lectura cruda del sensor de color: se lee cada
// PERIODO_MEDICION_COLOR haya objeto o no, así gemelo.py
// registra la señal completa (Anexo A)
RGB colorUltimo = {0, 0, 0};
// gemelo.py lo usa para saber que hubo una decisión nueva
unsigned int cantidadClasificaciones = 0;
// Desde el eco que vio el objeto en la zona de medición
// hasta la orden al servo (incluye el debounce)
unsigned long latenciaDecisionMs = 0;
// Período más largo entre dos cálculos del PID desde la
// trama anterior: gemelo.py lo compara con el deadline
unsigned long periodoPidMaxMs = 0;

SensorDistancia sensorDistancia{TRIGGER_PIN,ECHO_PIN,INT_0};
SensorColor sensorColor{};
// Rising: se mide el tiempo entre flancos iguales (ver
// SensorVelocidad.h)
SensorVelocidad sensorVelocidad{SPEED_PIN,INT_1,Rising};
ServoClassificador servoClassificador{
    SERVO_PIN,
    SERVO_A_POSITION,
    SERVO_B_POSITION,
    SERVO_CENTER_POSITION};
ServoRechazo servoRechazo{
    SERVO_RECHAZO_PIN,
    SERVO_RECHAZO_REPOSO,
    SERVO_RECHAZO_ACTIVO};

unsigned long ultimaLecturaDistancia = 0;
unsigned long ultimaLecturaVelocidad = 0;
unsigned long ultimaLecturaColor = 0;
unsigned long ultimaRespuestaSerial = 0;
unsigned long ultimaInversion = 0;

// "K1.000,0.200,0.000" ya ocupa 18 caracteres
char bufferSerial[40];
unsigned int largoBuffer = 0;

unsigned long timePasedUntilLastDistanceRead(unsigned long);
unsigned long timePasedUntilLastSpeedRead(unsigned long);
unsigned long timePasedUntilLastSerialOut(unsigned long);
void callHCSR04AsyncRead();
void calcularVelocidad();
void leerVelocidad();
void controlarMotor(float);
void verificarAtasco();
void telemetriaSerial();
void leerComando();
void interpretarComando(char*);
void cambiarSP(double);
void cambiarKpKiKd(double,double,double);
void switchDirection(bool);
void detener();
void apagarMotor();
void frenarMotor();
void aplicarPuente(EstadoMotor);
void actualizarSetpoint();
float getDistancia();
void setServo();
void seguimientoObjeto(float);
void empezarClasificacion();
void tomarMuestraColor();
void decidir(TipoObjeto);
void verificarEmergencia();
void entrarParadaSegura(Falla);
void rearmar();
void actualizarTiempoParada();
EstadoSistema calcularEstado();


// Clase de UNA lectura (sin debounce)
TipoObjeto claseDeColor(RGB color) {
    // unsigned long: la suma de 3 canales no entra en 16 bits
    unsigned long brillo = (unsigned long)color.r + color.g + color.b;
    if (brillo < BRILLO_MINIMO){
        return DUDOSO;
    }

    float r = color.r * GANANCIA_R;
    float g = color.g * GANANCIA_G;
    float b = color.b * GANANCIA_B;

    TipoObjeto clase;
    float mayor;
    float segundo;
    if (r >= g && r >= b){
        clase = ROJO;
        mayor = r;
        segundo = max(g, b);
    }else if (g >= b){
        clase = VERDE;
        mayor = g;
        segundo = max(r, b);
    }else{
        clase = AZUL;
        mayor = b;
        segundo = max(r, g);
    }

    if ((mayor - segundo) / (r + g + b) < MARGEN_COLOR){
        return DUDOSO;
    }
    return clase;
}


void setup(){
    Serial.begin(BAUDRATE);
    pinMode(PIN_IN1, OUTPUT);
    pinMode(PIN_IN2, OUTPUT);
    pinMode(PIN_ENA, OUTPUT);
    pinMode(PIN_EMERGENCIA, INPUT_PULLUP);
    pinMode(PIN_ALARMA, OUTPUT);
    digitalWrite(PIN_ALARMA, LOW);
    pid.SetOutputLimits(0, PWM_MAX);
    detener();

    // Sin sensor las lecturas dan 0,0,0 y a las
    // LECTURAS_NULAS_FALLA se entra en PARADA_SEGURA
    if (!sensorColor.begin()){
        Serial.println("No se encontro el sensor TCS34725");
    }
    sensorDistancia.begin();
    sensorVelocidad.begin();
    servoClassificador.begin();
    servoRechazo.begin();

    // El setup tarda (begin() del color espera la primera
    // integración): sin esto el primer período del PID
    // contaría ese tiempo
    ultimaLecturaVelocidad = millis();
}



void loop(){
    auto actual = millis();

    leerComando();

    verificarEmergencia();

    if(timePasedUntilLastDistanceRead(actual) >= PERIODO_MEDICION_DISTANCIA){
        // Una muestra por disparo: el resultado del
        // disparo anterior, antes de lanzar el nuevo
        if (fallaActual == SIN_FALLA){
            seguimientoObjeto(getDistancia());
        }
        callHCSR04AsyncRead();
    }

    if(actual - ultimaLecturaColor >= PERIODO_MEDICION_COLOR){
        tomarMuestraColor();
    }

    if(timePasedUntilLastSpeedRead(actual) >= PERIODO_MEDICION_VELOCIDAD){
        unsigned long periodo = timePasedUntilLastSpeedRead(actual);
        periodoPidMaxMs = max(periodoPidMaxMs, periodo);
        float dt = periodo / 1000.0;
        calcularVelocidad();
        leerVelocidad();
        controlarMotor(dt);
        verificarAtasco();
    }

    if (fallaActual == SIN_FALLA){
        setServo();
    }else{
        // Servos quietos: no se mueve nada con la
        // máquina en falla
        actualizarTiempoParada();
    }

    estadoSistema = calcularEstado();

    if(timePasedUntilLastSerialOut(actual) > PERIODO_RESPUESTA_SERIAL){
        telemetriaSerial();
    }

    delay(5);
}

unsigned long timePasedUntilLastDistanceRead(unsigned long actual)
{
    return actual - ultimaLecturaDistancia;
}

unsigned long timePasedUntilLastSpeedRead(unsigned long actual)
{
    return actual - ultimaLecturaVelocidad;
}

unsigned long timePasedUntilLastSerialOut(unsigned long actual)
{
    return actual - ultimaRespuestaSerial;
}

void callHCSR04AsyncRead()
{
    ultimaLecturaDistancia = millis();
    sensorDistancia.callAsyncRead();
}

void calcularVelocidad()
{
    ultimaLecturaVelocidad = millis();
    sensorVelocidad.calcularVelocidad();
}

void leerVelocidad()
{
    velocidad = sensorVelocidad.getVelocidad();

    // Falla inyectada: la banda "no gira" para el PID y
    // para la detección de atasco
    if (inyeccion == INYECTAR_ATASCO){
        velocidad = 0;
    }
}

void controlarMotor(float dt)
{
    // En PARADA_SEGURA el motor queda frenado: no se toca
    if (fallaActual != SIN_FALLA){
        return;
    }
    // Parado, recién invertido o con velocidad 0: motor
    // apagado y el PID no acumula integral
    if (setpoint <= 0 || millis() - ultimaInversion < PAUSA_INVERSION_MS){
        apagarMotor();
        return;
    }
    // Al pasar de MANUAL a AUTOMATIC el PID se reinicia
    pid.SetMode(PID::Mode::AUTOMATIC);
    pid.Compute(dt);
    pwm = (int)(salidaPID + 0.5);
    analogWrite(PIN_ENA, pwm);
}

void verificarAtasco()
{
    if (fallaActual != SIN_FALLA){
        posibleAtasco = false;
        return;
    }

    bool atascado = pwm >= PWM_ATASCO && velocidad < VELOCIDAD_ATASCO;

    if (!atascado){
        posibleAtasco = false;
        return;
    }

    if (!posibleAtasco){
        posibleAtasco = true;
        inicioAtasco = millis();
    }else if (millis() - inicioAtasco >= TIEMPO_ATASCO_MS){
        entrarParadaSegura(FALLA_ATASCO);
    }
}

void verificarEmergencia()
{
    if (digitalRead(PIN_EMERGENCIA) == EMERGENCIA_ACTIVA){
        entrarParadaSegura(FALLA_EMERGENCIA);
    }
}

void entrarParadaSegura(Falla falla)
{
    // Ya en parada: queda la primera causa
    if (fallaActual != SIN_FALLA){
        return;
    }
    fallaActual = falla;
    momentoFallaUs = micros();
    tiempoParadaMs = 0;

    estadoMotor = PARADO;
    actualizarSetpoint();
    frenarMotor();
    digitalWrite(PIN_ALARMA, HIGH);

    clasificando = false;
}

void rearmar()
{
    if (fallaActual == SIN_FALLA){
        return;
    }
    // Con el pulsador todavía apretado no se rearma
    if (digitalRead(PIN_EMERGENCIA) == EMERGENCIA_ACTIVA){
        return;
    }
    fallaActual = SIN_FALLA;
    posibleAtasco = false;
    lecturasNulas = 0;
    digitalWrite(PIN_ALARMA, LOW);

    // Se suelta el freno; queda en IDLE hasta F/R
    detener();

    // Lo que esté en la cinta se vuelve a detectar. Si ya
    // pasó la zona de medición se rechaza (DUDOSO)
    tipoObjeto = SIN_OBJETO;
    muestrasPresente = 0;
    muestrasAusente = 0;
}

void actualizarTiempoParada()
{
    // Último movimiento del eje después de la falla. Si el
    // motor ya estaba quieto queda en 0
    unsigned long ultimo = sensorVelocidad.getUltimoFlancoUs();
    if ((long)(ultimo - momentoFallaUs) > 0){
        tiempoParadaMs = (ultimo - momentoFallaUs) / 1000;
    }
}

EstadoSistema calcularEstado()
{
    if (fallaActual != SIN_FALLA){
        return PARADA_SEGURA;
    }
    if (estadoMotor == PARADO){
        return IDLE;
    }
    if (clasificando){
        return CLASIFICANDO;
    }
    switch (tipoObjeto){
        case ANTES_DE_MEDICION: return OBJETO_DETECTADO;
        case VERDE: return DESVIAR_A;
        case AZUL: return DESVIAR_B;
        case ROJO: return RECHAZAR;
        case DUDOSO: return RECHAZAR_DUDOSO;
        default: return RUN;
    }
}

void seguimientoObjeto(float distancia)
{
    // El objeto se aleja del sensor: arranca cerca y se
    // clasifica al llegar a DISTANCIA_MEDICION.
    // -1 = sin eco: no hay nada delante
    bool valida = distancia >= 0 && distancia <= TAMANO_MAXIMO_CINTA;

    if (valida){
        muestrasAusente = 0;
        if (muestrasPresente < MUESTRAS_PRESENCIA){
            muestrasPresente++;
        }
    }else{
        muestrasPresente = 0;
        if (muestrasAusente < MUESTRAS_AUSENCIA){
            muestrasAusente++;
        }
    }

    if (tipoObjeto == SIN_OBJETO){
        // Un valor suelto con la cinta vacía no es un objeto
        if (muestrasPresente >= MUESTRAS_PRESENCIA){
            if (distancia < DISTANCIA_MEDICION){
                tipoObjeto = ANTES_DE_MEDICION;
            }else{
                // Apareció pasada la zona de medición: no se
                // le midió el color, se rechaza
                tipoObjeto = DUDOSO;
            }
        }
        return;
    }

    if (muestrasAusente >= MUESTRAS_AUSENCIA){
        // Salió de la cinta (o se fue durante el debounce)
        tipoObjeto = SIN_OBJETO;
        clasificando = false;
        return;
    }

    // Lectura falsa suelta: el objeto sigue donde estaba
    if (!valida){
        return;
    }

    if (tipoObjeto == ANTES_DE_MEDICION
        && !clasificando
        && distancia >= DISTANCIA_MEDICION){
        empezarClasificacion();
    }
}

void empezarClasificacion()
{
    clasificando = true;
    claseCandidata = SIN_OBJETO;
    muestrasIguales = 0;
    muestrasTotales = 0;
    momentoDeteccionUs = sensorDistancia.getMomentoLecturaUs();

    // La primera muestra ya, sin esperar el período
    tomarMuestraColor();
}

void tomarMuestraColor()
{
    ultimaLecturaColor = millis();

    RGB color = sensorColor.getRGB();
    if (inyeccion == INYECTAR_SENSOR_COLOR){
        color = RGB{0, 0, 0};
    }
    colorUltimo = color;

    // Sensor que no responde
    if (color.r == 0 && color.g == 0 && color.b == 0){
        lecturasNulas++;
        if (lecturasNulas >= LECTURAS_NULAS_FALLA){
            entrarParadaSegura(FALLA_SENSOR_COLOR);
        }
    }else{
        lecturasNulas = 0;
    }

    if (!clasificando || fallaActual != SIN_FALLA){
        return;
    }

    // Debounce por conteo de muestras consecutivas
    TipoObjeto clase = claseDeColor(color);
    muestrasTotales++;
    if (clase == claseCandidata){
        muestrasIguales++;
    }else{
        claseCandidata = clase;
        muestrasIguales = 1;
    }

    if (muestrasIguales >= N_DEBOUNCE){
        decidir(claseCandidata);
    }else if (muestrasTotales >= MAX_MUESTRAS_COLOR){
        // Lecturas que alternan: contacto dudoso
        decidir(DUDOSO);
    }
}

void decidir(TipoObjeto clase)
{
    clasificando = false;
    tipoObjeto = clase;
    cantidadClasificaciones++;
    setServo();
    latenciaDecisionMs = (micros() - momentoDeteccionUs) / 1000;
}

// Trama que lee gemelo.py:
// RPM,SETPOINT,DISTANCIA,PWM,ESTADO,OBJETO,R,G,B,CLASIFICACIONES,
// LATENCIA_MS,PERIODO_PID_MS,AUTOMATA,FALLA,T_PARADA_MS,INYECCION
// (descarta las que no tienen exactamente 16 campos)
void telemetriaSerial()
{
    ultimaRespuestaSerial = millis();
    Serial.print(velocidad, 2);
    Serial.print(',');
    Serial.print(setpoint, 2);
    Serial.print(',');
    Serial.print(getDistancia(), 2);
    Serial.print(',');
    Serial.print(pwm);
    Serial.print(',');
    Serial.print((int)estadoMotor);
    Serial.print(',');
    Serial.print((int)tipoObjeto);
    Serial.print(',');
    Serial.print(colorUltimo.r);
    Serial.print(',');
    Serial.print(colorUltimo.g);
    Serial.print(',');
    Serial.print(colorUltimo.b);
    Serial.print(',');
    Serial.print(cantidadClasificaciones);
    Serial.print(',');
    Serial.print(latenciaDecisionMs);
    Serial.print(',');
    Serial.print(periodoPidMaxMs);
    Serial.print(',');
    Serial.print((int)estadoSistema);
    Serial.print(',');
    Serial.print((int)fallaActual);
    Serial.print(',');
    Serial.print(tiempoParadaMs);
    Serial.print(',');
    Serial.println((int)inyeccion);
    periodoPidMaxMs = 0;
}

void leerComando()
{
    // Junta caracteres sin bloquear y procesa cada línea completa
    while (Serial.available() > 0){
        char c = Serial.read();
        if (c == '\n' || c == '\r'){
            if (largoBuffer > 0){
                bufferSerial[largoBuffer] = '\0';
                largoBuffer = 0;
                interpretarComando(bufferSerial);
            }
        }else if (largoBuffer < sizeof(bufferSerial) - 1){
            bufferSerial[largoBuffer++] = c;
        }
    }
}

// Comandos de gemelo.py: V<0-100>, F, R, S, K<kp>,<ki>,<kd>,
// E (parada de emergencia), X (rearme), I<0-2> (inyectar falla)
void interpretarComando(char* comando)
{
    char* p = comando + 1;
    switch (comando[0]){
        case 'V': {
            char* fin;
            double porcentaje = strtod(p, &fin);
            // Sin número se ignora (no se frena por error)
            if (fin != p){
                cambiarSP(porcentaje);
            }
            break;
        }
        case 'F':
            switchDirection(true);
            break;
        case 'R':
            switchDirection(false);
            break;
        case 'S':
            detener();
            break;
        case 'K': {
            double kp = strtod(p, &p);
            if (*p != ',') break;
            double ki = strtod(p + 1, &p);
            if (*p != ',') break;
            double kd = strtod(p + 1, &p);
            cambiarKpKiKd(kp, ki, kd);
            break;
        }
        case 'E':
            entrarParadaSegura(FALLA_HMI);
            break;
        case 'X':
            rearmar();
            break;
        case 'I': {
            int n = atoi(p);
            if (n >= SIN_INYECCION && n <= INYECTAR_ATASCO){
                inyeccion = (Inyeccion)n;
            }
            break;
        }
    }
}

void cambiarSP(double porcentaje)
{
    rpmElegida = constrain(porcentaje, 0, 100) / 100.0 * RPM_MAX;
    // En marcha el PID toma el valor nuevo en el próximo cálculo
    actualizarSetpoint();
}

void cambiarKpKiKd(double kp, double ki, double kd)
{
    if (kp < 0 || ki < 0 || kd < 0){
        return;
    }
    pid.setKp(kp);
    pid.setKi(ki);
    pid.setKd(kd);
}

void switchDirection(bool adelante)
{
    // En PARADA_SEGURA no arranca: primero el rearme (X)
    if (fallaActual != SIN_FALLA){
        return;
    }
    EstadoMotor sentido = adelante ? ADELANTE : REVERSA;
    // Ya va para ese lado: gemelo.py manda V y F/R en cada botón
    if (sentido == estadoMotor){
        return;
    }
    if (estadoMotor != PARADO){
        ultimaInversion = millis();
    }
    estadoMotor = sentido;
    actualizarSetpoint();
    // Con ENA en 0 el motor queda suelto: se puede cambiar el
    // sentido del puente y el PID lo arranca en su próximo cálculo
    apagarMotor();
    aplicarPuente(sentido);
}

void detener()
{
    // En PARADA_SEGURA el motor sigue frenado
    if (fallaActual != SIN_FALLA){
        return;
    }
    estadoMotor = PARADO;
    actualizarSetpoint();
    aplicarPuente(PARADO);
    apagarMotor();
}

void apagarMotor()
{
    pid.SetMode(PID::Mode::MANUAL);
    salidaPID = 0;
    pwm = 0;
    analogWrite(PIN_ENA, 0);
}

void frenarMotor()
{
    // Freno dinámico: las dos salidas del puente al mismo
    // nivel con ENA en alto cortocircuitan el motor y la
    // banda para mucho antes que dejándola suelta (L298N:
    // "fast motor stop"; TB6612: "short brake")
    pid.SetMode(PID::Mode::MANUAL);
    salidaPID = 0;
    pwm = 0;
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, HIGH);
    analogWrite(PIN_ENA, PWM_MAX);
}

void aplicarPuente(EstadoMotor sentido)
{
    digitalWrite(PIN_IN1, sentido == ADELANTE ? HIGH : LOW);
    digitalWrite(PIN_IN2, sentido == REVERSA ? HIGH : LOW);
}

void actualizarSetpoint()
{
    setpoint = (estadoMotor == PARADO) ? 0.0 : rpmElegida;
}

float getDistancia()
{
    return sensorDistancia.getDistancia();
}

void setServo()
{
    // El servo de rechazo está antes del clasificador: lo
    // rechazado no llega al clasificador, que queda donde
    // estaba. Sin clasificar, el clasificador queda en la
    // última posición; sólo va al centro al arrancar
    switch (tipoObjeto){
        case VERDE:
            servoRechazo.reposo();
            servoClassificador.goToPosA();
            break;
        case AZUL:
            servoRechazo.reposo();
            servoClassificador.goToPosB();
            break;
        case ROJO:
        case DUDOSO:
            servoRechazo.rechazar();
            break;
        case SIN_OBJETO:
            // El objeto rechazado salió de la cinta
            servoRechazo.reposo();
            break;
        default:
            break;
    }
}
