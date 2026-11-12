import csv
import math
import os
import serial
import statistics
import time
import threading

from collections import deque

from dash import Dash, dcc, html, Input, Output, State, ctx

import plotly.graph_objects as go


# ==================================================
# PUERTO SERIAL
# ==================================================

PUERTO = "COM4"
BAUDRATE = 115200


try:

    ser = serial.Serial(
        PUERTO,
        BAUDRATE,
        timeout=0.2
    )

except serial.SerialException as e:

    from serial.tools import list_ports

    disponibles = [p.device for p in list_ports.comports()]

    raise SystemExit(
        f"No se pudo abrir {PUERTO}: {e}\n"
        f"Puertos disponibles: {disponibles or 'ninguno'}\n"
        "Conectá el Arduino, cerrá el Monitor Serie y "
        "revisá PUERTO."
    )


# Al abrir el puerto el Arduino se reinicia
time.sleep(2)

ser.reset_input_buffer()


serial_lock = threading.Lock()

# Protege el historial: lo escribe el hilo serial y
# lo lee la página
datos_lock = threading.Lock()


# ==================================================
# GANANCIAS PID INICIALES (iguales a cinta_R.ino)
# ==================================================

KP_INICIAL = 1.0
KI_INICIAL = 0.4
KD_INICIAL = 0.0


# ==================================================
# ESCALA DE VELOCIDAD
# ==================================================

# El Arduino recibe V<0-100> como % de su RPM_MAX
# (cinta.ino). 100 % del slider = 90 RPM
RPM_MAX_ARDUINO = 90.0
RPM_MAX_SETPOINT = 90.0


def porcentaje_arduino(velocidad):

    # Se manda un entero (pasos de 0.9 RPM). Con
    # RPM_MAX_SETPOINT = RPM_MAX_ARDUINO el % del slider
    # pasa tal cual
    return int(
        round(
            velocidad * RPM_MAX_SETPOINT / RPM_MAX_ARDUINO
        )
    )


# ==================================================
# VARIABLES ACTUALES DEL SISTEMA
# ==================================================

rpm_actual = 0.0
setpoint_actual = 0.0
distancia_actual = -1.0
pwm_actual = 0.0
estado_actual = 0
objeto_actual = 0
# (R, G, B) de la última clasificación
color_actual = (0, 0, 0)
clasificaciones_actual = 0
latencia_actual = 0
periodo_pid_actual = 0
automata_actual = 0
falla_actual = 0
t_parada_actual = 0
inyeccion_actual = 0

ultima_trama = "Esperando datos del Arduino..."


# ==================================================
# DIVERGENCIA
# ==================================================

divergencia_activa = False

inicio_divergencia = None

ultimo_setpoint_divergencia = 0.0

ultimo_estado_divergencia = 0


# El error debe mantenerse durante 2 segundos
TIEMPO_DIVERGENCIA = 2.0


# Permitimos 10 % de diferencia
TOLERANCIA_PORCENTAJE = 0.10


# Pero como mínimo permitimos 0.75 RPM
TOLERANCIA_MINIMA = 0.75


# ==================================================
# HISTORIAL PARA GRÁFICAS
# ==================================================

# El Arduino envía una trama cada 50 ms: 200 puntos = 10 s
MAX_PUNTOS = 200

tiempos = deque(maxlen=MAX_PUNTOS)

rpm_hist = deque(maxlen=MAX_PUNTOS)

setpoint_hist = deque(maxlen=MAX_PUNTOS)

# Posición del objeto (None = sin objeto: corta la línea)
pos_real_hist = deque(maxlen=MAX_PUNTOS)

pos_sim_hist = deque(maxlen=MAX_PUNTOS)


# ==================================================
# DIVERGENCIAS DE CLASIFICACIÓN Y BANDA
# ==================================================

# Sólo se detectan y se muestran: la acción de cada una
# queda a cargo del operador

# Distancias en cm desde el ultrasonido, iguales a
# cinta_R.ino. El objeto se aleja del sensor
DISTANCIA_MEDICION = 15.0
TAMANO_MAXIMO_CINTA = 40.0

# Dónde barre el brazo de cada servo: medirlo en la
# cinta. El de rechazo va antes del clasificador A/B.
# Cada posición + MARGEN_DESVIADOR_CM tiene que quedar
# por debajo de TAMANO_MAXIMO_CINTA, o un objeto que no
# se desvió no se vería pasar
POSICION_RECHAZO_CM = 20.0
POSICION_DESVIADOR_CM = 25.0

# Con la clase decidida, un objeto que sigue en la cinta
# más allá de esto no fue desviado
MARGEN_DESVIADOR_CM = 5.0

# Con la banda hacia adelante el objeto sólo se aleja:
# si vuelve hacia el sensor más que esto, rebotó
MARGEN_REBOTE_CM = 4.0

# Si según el encoder la banda avanzó este múltiplo de lo
# que le faltaba al objeto para pasar el desviador y
# sigue en la cinta, quedó trabado
FACTOR_RECORRIDO_DESVIO = 2.0

# Mediana de las últimas 12 tramas (6 lecturas del
# ultrasonido): un eco suelto no la mueve
TRAMAS_MEDIANA_DISTANCIA = 12

# Suma R+G+B esperada (integración de 50 ms, ganancia
# 4X). Igual a BRILLO_MINIMO de cinta_R.ino: medido en
# los registros, cinta vacía ~53 y el objeto más oscuro
# ~110. 0 = el sensor no respondió
BRILLO_MIN_VALIDO = 80
BRILLO_MAX_VALIDO = 20000

# La tasa de dudosos se calcula sobre las últimas
# clasificaciones
VENTANA_DUDOSOS = 10
MIN_CLASIFICACIONES_TASA = 5
UMBRAL_TASA_DUDOSOS = 0.30

# Deadlines de la letra (Cuadro 5), ajustados a la
# estrategia como permite la letra.
# Decisión de clase: desde el eco que ve el objeto en la
# zona de medición hasta la orden al servo. El debounce
# toma N_DEBOUNCE = 3 lecturas de color separadas por la
# integración del sensor (60 ms): 120 ms si las lecturas
# coinciden y hasta 240 ms si termina en DUDOSO. Lo que
# de verdad limita es la Ec. 1 (llegar antes al servo):
# se verifica abajo con MARGEN_DESVIADOR
DEADLINE_DECISION_MS = 250

# Muestreo de sensores y actualización del PID
DEADLINE_PID_MS = 100

# Parada segura: desde la falla hasta que el motor queda
# quieto (último flanco del encoder)
DEADLINE_PARADA_MS = 200

# Sincronización del desviador (Ec. 1 de la letra):
# t_servo + latencia de decisión + margen tiene que ser
# menor que el vuelo d / v_banda desde la zona de
# medición hasta cada servo. Medir el tiempo de giro de
# los servos (filmando a cámara lenta, por ejemplo)
T_SERVO_MS = 150
T_MARGEN_MS = 50

# El Arduino manda una trama cada 50 ms
DEADLINE_TRAMA_MS = 250

# Avance de la banda por vuelta del encoder: medir el
# diámetro del rodillo motriz
DIAMETRO_RODILLO_CM = 3.0
AVANCE_CM_POR_VUELTA = math.pi * DIAMETRO_RODILLO_CM

TOLERANCIA_POSICION_CM = 3.0

# Cuánto tiene que sostenerse una falla para confirmarla
# (el ultrasonido mide cada 100 ms)
TIEMPO_CONFIRMACION_DESVIO = 0.5
TIEMPO_CONFIRMACION_POSICION = 1.0


# TipoObjeto en cinta_R.ino
OBJETO_DUDOSO = 5

NOMBRES_OBJETO = {
    0: "Sin objeto",
    1: "Antes de medición",
    2: "VERDE → A",
    3: "AZUL → B",
    4: "ROJO → rechazo",
    5: "DUDOSO → rechazo"
}

# Clases decididas: (nombre, dónde las saca su servo)
CLASES = {
    2: ("A (VERDE)", POSICION_DESVIADOR_CM),
    3: ("B (AZUL)", POSICION_DESVIADOR_CM),
    4: ("RECHAZO (ROJO)", POSICION_RECHAZO_CM),
    5: ("RECHAZO (DUDOSO)", POSICION_RECHAZO_CM)
}

# Contador de cada clase
CLAVE_CONTADOR = {
    2: "A",
    3: "B",
    4: "rojo",
    5: "dudoso"
}


# EstadoSistema en cinta_R.ino (AUTOMATA en la trama)
AUTOMATA_PARADA_SEGURA = 8

NOMBRES_AUTOMATA = {
    0: "IDLE",
    1: "RUN",
    2: "OBJETO_DETECTADO",
    3: "CLASIFICANDO",
    4: "DESVIAR_A",
    5: "DESVIAR_B",
    6: "RECHAZAR",
    7: "RECHAZAR_DUDOSO",
    8: "PARADA_SEGURA"
}

# Falla en cinta_R.ino
NOMBRES_FALLA = {
    0: "Sin falla",
    1: "Pulsador de emergencia",
    2: "Parada de emergencia desde el gemelo",
    3: "Atasco de banda",
    4: "Sensor de color sin respuesta"
}

# Inyeccion en cinta_R.ino (comando I<n>)
NOMBRES_INYECCION = {
    0: "ninguna",
    1: "sensor de color",
    2: "atasco"
}


# Fallas virtuales: el gemelo altera los datos que entran
# a los detectores, sin tocar el Arduino, para probar que
# cada divergencia se detecta
FALLAS_VIRTUALES = {
    "servo": "Servo no acciona (el objeto sigue de largo)",
    "sensor_sucio": "Sensor sucio (lectura de color baja)",
    "latencia": "Latencia de decisión +300 ms",
    "deslizamiento": "Deslizamiento de banda (encoder +50 %)",
    "dudosos": "Todas las lecturas dudosas",
}

# Registro de cada trama en CSV (mínimos/máximos, cuello
# de botella, señal cruda del color para el Anexo A)
CARPETA_REGISTROS = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "registros"
)


DIVERGENCIAS = {

    "desviador": {
        "nombre": "Clase decidida pero desviador no acciona",
        "causa": "Fallo de servo, objeto rebotó",
        "accion": "Alarma + revisión",
        "espera": "Esperando un objeto clasificado"
    },

    "rango": {
        "nombre": "Lectura de clasificación fuera de rango",
        "causa": "Sensor sucio / descalibrado",
        "accion": "Mantenimiento",
        "espera": "Esperando una lectura de color"
    },

    "latencia": {
        "nombre": "Latencia de decisión > deadline",
        "causa": "Carga de CPU o comunicación",
        "accion": "Optimizar",
        "espera": "Esperando tramas"
    },

    "posicion": {
        "nombre": "Posición simulada ≠ posición real",
        "causa": "Deslizamiento de banda",
        "accion": "Recalibrar encoder",
        "espera": "Esperando un objeto en la cinta"
    },

    "dudosos": {
        "nombre": "Tasa de dudosos > umbral",
        "causa": "Humedad, deformación",
        "accion": "Ajustar",
        "espera": "Esperando clasificaciones"
    },
}


# Lo que muestra la página: SIN DATOS, OK, VERIFICANDO o
# ACTIVA. Lo escribe el hilo serial (con datos_lock)
estado_divergencias = {

    clave: {
        "estado": "SIN DATOS",
        "detalle": info["espera"]
    }

    for clave, info in DIVERGENCIAS.items()
}

# Últimas activaciones: (hora, clave, detalle)
registro_divergencias = deque(maxlen=15)


# Seguimiento del objeto: sólo lo usa el hilo serial
seguimiento = {

    # Valores de la trama anterior. El Arduino se
    # reinicia al abrir el puerto: el contador arranca
    # en 0
    "objeto": 0,
    "clasificaciones": 0,
    "t_trama": None,

    # De la última clasificación por color
    "latencia_ms": None,

    # Desviador: distancias desde la decisión y avance
    # de la banda según el encoder
    "dist_decision": None,
    "dist_max": None,
    "recorrido": 0.0,
    "falla_desvio": False,

    # Posición simulada (cm desde el ultrasonido)
    "pos_sim": None,
    "falla_posicion": False,
}

distancias_objeto = deque(maxlen=TRAMAS_MEDIANA_DISTANCIA)

# True = lectura dudosa
lecturas_dudosas = deque(maxlen=VENTANA_DUDOSOS)

# Segundos entre tramas (último segundo)
intervalos_trama = deque(maxlen=20)

# Período más largo del PID que informó cada trama
# (último segundo). 0 = no hubo cálculo en esa trama
periodos_pid = deque(maxlen=20)


# ==================================================
# AUTÓMATA, CONTADORES Y EVENTOS (con datos_lock)
# ==================================================

contadores = {
    "A": 0,
    "B": 0,
    "rojo": 0,
    "dudoso": 0,
    "fallas": 0
}

# Momento de cada objeto clasificado (tasa de objetos/min)
momentos_objetos = deque(maxlen=200)

# Cambios de estado del autómata: (hora, texto)
registro_eventos = deque(maxlen=15)

ultimo_evento = "Sin eventos"

# Valores de la trama anterior para detectar cambios
anterior = {
    "automata": None,
    "objeto": 0,
    "falla": 0,
    "t_parada": 0
}

# Mínimos y máximos operativos medidos con la banda en
# marcha (velocidad estable) y peor tiempo de parada
operativos = {
    "rpm_min": None,
    "rpm_max": None,
    "periodo_pid_max": 0,
    "latencia_max": 0,
    "t_parada_max": 0
}


# ==================================================
# FALLAS VIRTUALES
# ==================================================

# Las cambia la página, las lee el hilo serial
fallas_virtuales = {clave: False for clave in FALLAS_VIRTUALES}

# Servo virtual que no acciona: la clase y la posición
# simulada del objeto que "sigue de largo"
inyeccion_servo = {
    "clase": None,
    "pos": 0.0,
    "objeto_anterior": 0
}


# ==================================================
# REGISTRO CSV
# ==================================================

COLUMNAS_CSV = [
    "tiempo_s", "hora", "rpm", "setpoint", "distancia",
    "pwm", "estado_motor", "objeto", "r", "g", "b",
    "clasificaciones", "latencia_ms", "periodo_pid_ms",
    "automata", "falla", "t_parada_ms", "inyeccion",
    "fallas_virtuales"
]

registro_csv = {
    "archivo": None,
    "escritor": None,
    "nombre": "",
    "ultimo_flush": 0.0
}

registro_lock = threading.Lock()


def nuevo_registro():

    # Cierra el archivo actual y abre uno nuevo (uno por
    # experimento: ciclo completo, mínimos, fallas...)

    os.makedirs(CARPETA_REGISTROS, exist_ok=True)

    nombre = time.strftime("registro_%Y%m%d_%H%M%S.csv")

    with registro_lock:

        if registro_csv["archivo"] is not None:

            registro_csv["archivo"].close()

        archivo = open(
            os.path.join(CARPETA_REGISTROS, nombre),
            "w",
            newline="",
            encoding="utf-8"
        )

        escritor = csv.writer(archivo)

        escritor.writerow(COLUMNAS_CSV)

        registro_csv["archivo"] = archivo

        registro_csv["escritor"] = escritor

        registro_csv["nombre"] = nombre

    return nombre


def registrar_trama(ahora, valores):

    activas = "+".join(
        clave for clave, activa in fallas_virtuales.items()
        if activa
    )

    with registro_lock:

        escritor = registro_csv["escritor"]

        if escritor is None:
            return

        escritor.writerow(
            [f"{ahora:.3f}", time.strftime("%H:%M:%S")]
            + list(valores)
            + [activas]
        )

        # Guardar en disco cada 1 s, no en cada trama
        if ahora - registro_csv["ultimo_flush"] >= 1.0:

            registro_csv["archivo"].flush()

            registro_csv["ultimo_flush"] = ahora


class Confirmacion:

    # La condición pasa a ACTIVA si se sostiene
    # `segundos`; mientras tanto queda VERIFICANDO

    def __init__(self, segundos):

        self.segundos = segundos

        self.inicio = None


    def actualizar(self, condicion, ahora):

        if not condicion:

            self.inicio = None

            return "OK"

        if self.inicio is None:

            self.inicio = ahora

        if ahora - self.inicio >= self.segundos:

            return "ACTIVA"

        return "VERIFICANDO"


    def reiniciar(self):

        self.inicio = None


confirmacion_desvio = Confirmacion(TIEMPO_CONFIRMACION_DESVIO)

confirmacion_posicion = Confirmacion(TIEMPO_CONFIRMACION_POSICION)


def tolerancia_rpm(setpoint):

    # 10 % del setpoint, con un mínimo de 0.75 RPM
    return max(
        TOLERANCIA_MINIMA,
        setpoint * TOLERANCIA_PORCENTAJE
    )


def marcar(clave, estado, detalle):

    with datos_lock:

        actual = estado_divergencias[clave]

        # Se registra al activarse, no en cada trama
        if (
            estado == "ACTIVA"
            and
            actual["estado"] != "ACTIVA"
        ):

            registro_divergencias.appendleft(
                (time.strftime("%H:%M:%S"), clave, detalle)
            )

        actual["estado"] = estado

        actual["detalle"] = detalle


def evaluar_lectura_color(color, objeto):

    # ------------------------------------------
    # Lectura fuera de rango
    # ------------------------------------------

    brillo = sum(color)

    rango = f"{BRILLO_MIN_VALIDO}–{BRILLO_MAX_VALIDO}"

    fuera = (
        brillo < BRILLO_MIN_VALIDO
        or
        brillo > BRILLO_MAX_VALIDO
    )

    if brillo == 0:

        marcar(
            "rango", "ACTIVA",
            "Brillo 0: el sensor de color no respondió"
        )

    elif fuera:

        marcar(
            "rango", "ACTIVA",
            f"Brillo {brillo} fuera del rango {rango}"
        )

    else:

        r, g, b = color

        marcar(
            "rango", "OK",
            f"Último R={r} G={g} B={b}, brillo {brillo} "
            f"(rango {rango})"
        )


    # ------------------------------------------
    # Tasa de dudosos (el Arduino decide DUDOSO
    # sin color dominante o con poca luz)
    # ------------------------------------------

    lecturas_dudosas.append(objeto == OBJETO_DUDOSO)

    total = len(lecturas_dudosas)

    dudosas = sum(lecturas_dudosas)

    detalle = (
        f"{dudosas} de las últimas {total} lecturas sin "
        "color dominante"
    )

    if total < MIN_CLASIFICACIONES_TASA:

        marcar(
            "dudosos", "SIN DATOS",
            f"{detalle} (se evalúa desde "
            f"{MIN_CLASIFICACIONES_TASA})"
        )

        return

    tasa = dudosas / total

    marcar(
        "dudosos",
        "ACTIVA" if tasa > UMBRAL_TASA_DUDOSOS else "OK",
        f"{tasa:.0%} dudosas (umbral "
        f"{UMBRAL_TASA_DUDOSOS:.0%}): {detalle}"
    )


def evaluar_latencia(ahora):

    t_trama = seguimiento["t_trama"]

    if t_trama is None:
        return

    # El hueco actual cuenta aunque todavía no haya
    # llegado la trama siguiente
    peor_trama_ms = 1000 * max(
        max(intervalos_trama, default=0.0),
        ahora - t_trama
    )

    latencia = seguimiento["latencia_ms"]

    lenta_cpu = (
        latencia is not None
        and
        latencia > DEADLINE_DECISION_MS
    )

    lenta_com = peor_trama_ms > DEADLINE_TRAMA_MS

    peor_pid_ms = max(periodos_pid, default=0)

    lento_pid = peor_pid_ms > DEADLINE_PID_MS

    if latencia is None:

        decision = "Decisión: sin clasificaciones"

    else:

        decision = (
            f"Decisión: {latencia} ms "
            f"(deadline {DEADLINE_DECISION_MS} ms)"
            + (" ⚠" if lenta_cpu else "")
        )

    tramas = (
        f"Tramas: hasta {peor_trama_ms:.0f} ms entre "
        f"tramas (deadline {DEADLINE_TRAMA_MS} ms)"
        + (" ⚠" if lenta_com else "")
    )

    pid = (
        f"PID: período hasta {peor_pid_ms} ms "
        f"(deadline {DEADLINE_PID_MS} ms)"
        + (" ⚠" if lento_pid else "")
    )

    sincronia, tarde_servo = sincronizacion_desviador(
        rpm_actual, estado_actual, latencia
    )

    marcar(
        "latencia",
        "ACTIVA"
        if lenta_cpu or lenta_com or lento_pid or tarde_servo
        else "OK",
        f"{decision} · {pid} · {tramas} · {sincronia}"
    )


def sincronizacion_desviador(rpm, estado, latencia):

    # Ec. 1: el servo tiene que estar en posición antes de
    # que llegue el objeto. El más exigente es el servo
    # más cercano a la zona de medición.
    # Devuelve (texto, True si no llega a tiempo)

    distancia_cm = (
        min(POSICION_RECHAZO_CM, POSICION_DESVIADOR_CM)
        - DISTANCIA_MEDICION
    )

    # Sin decisiones todavía: el peor caso permitido
    if latencia is None:
        latencia = DEADLINE_DECISION_MS

    necesario_ms = latencia + T_SERVO_MS + T_MARGEN_MS

    v_max = distancia_cm / (necesario_ms / 1000)

    rpm_max = v_max * 60 / AVANCE_CM_POR_VUELTA

    v = abs(rpm) / 60 * AVANCE_CM_POR_VUELTA

    if estado != 1 or v <= 0:

        return (
            f"Desviador: velocidad máxima {rpm_max:.0f} RPM",
            False
        )

    vuelo_ms = 1000 * distancia_cm / v

    margen_ms = vuelo_ms - necesario_ms

    return (
        f"Desviador: vuelo {vuelo_ms:.0f} ms, margen "
        f"{margen_ms:+.0f} ms (máx. {rpm_max:.0f} RPM)"
        + (" ⚠ velocidad excesiva" if margen_ms < 0 else ""),
        margen_ms < 0
    )


def evaluar_desvio(ahora, dt, rpm, estado, objeto, objeto_anterior):

    clase = CLASES.get(objeto)


    # Sin clase decidida. Si recién salió de la cinta sin
    # alarma, el desviador lo sacó
    if clase is None:

        clase_anterior = CLASES.get(objeto_anterior)

        if (
            clase_anterior is not None
            and
            not seguimiento["falla_desvio"]
        ):

            marcar(
                "desviador", "OK",
                f"Último objeto clase {clase_anterior[0]} "
                "desviado"
            )

        confirmacion_desvio.reiniciar()

        return


    clase, posicion_servo = clase


    # Clase recién decidida: el servo ya tendría que
    # estar en posición. Una alarma anterior queda hasta
    # acá
    if objeto != objeto_anterior:

        seguimiento["dist_decision"] = None

        seguimiento["dist_max"] = None

        seguimiento["recorrido"] = 0.0

        seguimiento["falla_desvio"] = False

        confirmacion_desvio.reiniciar()


    if seguimiento["falla_desvio"] or not distancias_objeto:
        return


    distancia = statistics.median(distancias_objeto)


    # Parada o en reversa no avanza hacia el desviador:
    # se vuelve a medir desde donde quede
    if estado != 1:

        seguimiento["dist_decision"] = None

        seguimiento["dist_max"] = None

        seguimiento["recorrido"] = 0.0

        confirmacion_desvio.reiniciar()

        marcar(
            "desviador", "OK",
            f"Clase {clase} decidida, banda sin avanzar: "
            "no se evalúa"
        )

        return


    if seguimiento["dist_decision"] is None:

        seguimiento["dist_decision"] = distancia

        seguimiento["dist_max"] = distancia

    seguimiento["dist_max"] = max(
        seguimiento["dist_max"],
        distancia
    )

    seguimiento["recorrido"] += (
        abs(rpm) / 60.0
        * AVANCE_CM_POR_VUELTA
        * dt
    )

    dist_max = seguimiento["dist_max"]

    limite = posicion_servo + MARGEN_DESVIADOR_CM

    faltante = max(
        limite - seguimiento["dist_decision"],
        0.0
    )

    rebote = dist_max - distancia > MARGEN_REBOTE_CM


    if distancia > limite:

        falla = True

        detalle = (
            f"Clase {clase} decidida y el objeto siguió "
            f"hasta {distancia:.1f} cm (servo en "
            f"{posicion_servo:.0f} cm)"
        )

    elif rebote:

        falla = True

        detalle = (
            f"Clase {clase} decidida y el objeto volvió "
            f"de {dist_max:.1f} a {distancia:.1f} cm: "
            "rebotó"
        )

    elif (
        seguimiento["recorrido"]
        >
        FACTOR_RECORRIDO_DESVIO * faltante
    ):

        falla = True

        detalle = (
            f"Clase {clase} decidida, la banda avanzó "
            f"{seguimiento['recorrido']:.0f} cm y el "
            f"objeto sigue en la cinta a {distancia:.1f} "
            "cm: quedó trabado"
        )

    else:

        falla = False

        detalle = (
            f"Clase {clase} decidida, objeto a "
            f"{distancia:.1f} cm (servo en "
            f"{posicion_servo:.0f} cm)"
        )


    estado_div = confirmacion_desvio.actualizar(falla, ahora)

    # La mediana ya descarta los ecos sueltos y el
    # rebote dura poco: no espera la confirmación
    if rebote:

        estado_div = "ACTIVA"

    if estado_div == "ACTIVA":

        seguimiento["falla_desvio"] = True

    marcar("desviador", estado_div, detalle)


def evaluar_posicion(
    ahora, dt, rpm, setpoint, distancia, estado, objeto
):

    # Devuelve la posición simulada para la gráfica

    if objeto == 0:

        # Recién salió de la cinta
        if (
            seguimiento["pos_sim"] is not None
            and
            not seguimiento["falla_posicion"]
        ):

            marcar("posicion", "OK", "Sin objeto en la cinta")

        seguimiento["pos_sim"] = None

        confirmacion_posicion.reiniciar()

        return None


    valida = 0 <= distancia <= TAMANO_MAXIMO_CINTA


    # Objeto nuevo: la simulación arranca donde lo ve el
    # ultrasonido
    if seguimiento["pos_sim"] is None:

        if not valida:
            return None

        seguimiento["pos_sim"] = distancia

        seguimiento["falla_posicion"] = False

        confirmacion_posicion.reiniciar()

        marcar(
            "posicion", "OK",
            f"Objeto nuevo a {distancia:.1f} cm: "
            "simulación iniciada"
        )

        return distancia


    estable = (
        estado != 0
        and
        setpoint > 0
        and
        abs(abs(rpm) - setpoint) <= tolerancia_rpm(setpoint)
    )


    # Parada, arranque o cambio de velocidad: la RPM
    # filtrada va atrasada. Se vuelve a sincronizar con
    # el ultrasonido
    if not estable:

        if valida:

            seguimiento["pos_sim"] = distancia

        confirmacion_posicion.reiniciar()

        if not seguimiento["falla_posicion"]:

            marcar(
                "posicion", "OK",
                "Banda parada o en transitorio: simulación "
                "sincronizada con el ultrasonido"
            )

        return seguimiento["pos_sim"]


    # Avance según el encoder
    sentido = 1 if estado == 1 else -1

    seguimiento["pos_sim"] += (
        sentido
        * abs(rpm) / 60.0
        * AVANCE_CM_POR_VUELTA
        * dt
    )

    pos_sim = seguimiento["pos_sim"]


    # Ya hay alarma para este objeto, o trama sin eco
    if seguimiento["falla_posicion"] or not valida:
        return pos_sim


    # En su servo el objeto sale de costado: el
    # ultrasonido deja de medir su avance. Sin clase
    # todavía, el primero que puede sacarlo es el de
    # rechazo
    clase = CLASES.get(objeto)

    posicion_servo = (
        clase[1] if clase is not None
        else POSICION_RECHAZO_CM
    )

    if pos_sim > posicion_servo:

        confirmacion_posicion.reiniciar()

        marcar(
            "posicion", "OK",
            "Objeto en el servo: no se compara"
        )

        return pos_sim


    diferencia = pos_sim - distancia

    estado_div = confirmacion_posicion.actualizar(
        abs(diferencia) > TOLERANCIA_POSICION_CM,
        ahora
    )

    if estado_div == "ACTIVA":

        seguimiento["falla_posicion"] = True

    marcar(
        "posicion", estado_div,
        f"Simulada {pos_sim:.1f} cm · real "
        f"{distancia:.1f} cm · diferencia "
        f"{diferencia:+.1f} cm (tolerancia "
        f"±{TOLERANCIA_POSICION_CM:.0f} cm)"
    )

    return pos_sim


def evaluar_divergencias(
    ahora, rpm, setpoint, distancia, estado, objeto,
    color, clasificaciones, latencia_ms, periodo_pid_ms
):

    # Devuelve (posición simulada, posición real) para
    # la gráfica

    dt = 0.0

    if seguimiento["t_trama"] is not None:

        dt = ahora - seguimiento["t_trama"]

        intervalos_trama.append(dt)

    seguimiento["t_trama"] = ahora

    periodos_pid.append(periodo_pid_ms)


    objeto_anterior = seguimiento["objeto"]

    seguimiento["objeto"] = objeto


    en_cinta = (
        objeto != 0
        and
        0 <= distancia <= TAMANO_MAXIMO_CINTA
    )

    if objeto_anterior == 0 and objeto != 0:

        distancias_objeto.clear()

    if en_cinta:

        distancias_objeto.append(distancia)


    # Clasificación por color nueva (0 = el Arduino se
    # acaba de reiniciar)
    if (
        clasificaciones != seguimiento["clasificaciones"]
        and
        clasificaciones != 0
    ):

        seguimiento["latencia_ms"] = latencia_ms

        evaluar_lectura_color(
            color,
            OBJETO_DUDOSO if fallas_virtuales["dudosos"]
            else objeto
        )

    seguimiento["clasificaciones"] = clasificaciones


    evaluar_latencia(ahora)

    evaluar_desvio(
        ahora, dt, rpm, estado, objeto, objeto_anterior
    )

    pos_sim = evaluar_posicion(
        ahora, dt, rpm, setpoint, distancia, estado, objeto
    )

    return pos_sim, (distancia if en_cinta else None)


# ==================================================
# FALLAS VIRTUALES
# ==================================================

def aplicar_fallas_virtuales(
    dt, rpm, distancia, estado, objeto, color, latencia
):

    # Devuelve los datos alterados que ven los detectores.
    # La página y el CSV siguen mostrando los reales

    # Servo que no acciona: desde que se decide la clase,
    # el objeto "sigue de largo" por la cinta según el
    # encoder, hasta caerse por el final
    iny = inyeccion_servo

    objeto_real = objeto

    if not fallas_virtuales["servo"]:

        iny["clase"] = None

    else:

        clase_nueva = (
            objeto in CLASES
            and
            iny["objeto_anterior"] not in CLASES
        )

        if (
            iny["clase"] is None
            and
            clase_nueva
            and
            0 <= distancia <= TAMANO_MAXIMO_CINTA
        ):

            iny["clase"] = objeto

            iny["pos"] = distancia

        if iny["clase"] is not None:

            if estado == 1:

                iny["pos"] += (
                    abs(rpm) / 60.0
                    * AVANCE_CM_POR_VUELTA
                    * dt
                )

            if iny["pos"] > TAMANO_MAXIMO_CINTA:

                iny["clase"] = None

            else:

                objeto = iny["clase"]

                distancia = iny["pos"]

    iny["objeto_anterior"] = objeto_real


    if fallas_virtuales["sensor_sucio"]:

        color = tuple(c // 20 for c in color)

    if fallas_virtuales["latencia"]:

        latencia += 300

    if fallas_virtuales["deslizamiento"]:

        rpm *= 1.5

    return rpm, distancia, objeto, color, latencia


# ==================================================
# AUTÓMATA, CONTADORES Y EVENTOS
# ==================================================

def actualizar_automata(
    ahora, rpm, setpoint, estado, objeto, automata,
    falla, t_parada, periodo_pid, latencia,
    clasificaciones_nuevas
):

    global ultimo_evento

    with datos_lock:

        # Cambio de estado del autómata = evento
        if automata != anterior["automata"]:

            previo = anterior["automata"]

            texto = (
                NOMBRES_AUTOMATA.get(automata, str(automata))
                if previo is None
                else
                f"{NOMBRES_AUTOMATA.get(previo, previo)} → "
                f"{NOMBRES_AUTOMATA.get(automata, automata)}"
            )

            if automata == AUTOMATA_PARADA_SEGURA:

                texto += (
                    f" ({NOMBRES_FALLA.get(falla, falla)})"
                )

                contadores["fallas"] += 1

            # Al rearmar: cuánto tardó en quedar quieto
            if previo == AUTOMATA_PARADA_SEGURA:

                texto += (
                    f" · motor quieto a los "
                    f"{anterior['t_parada']} ms de la falla"
                )

            ultimo_evento = texto

            registro_eventos.appendleft(
                (time.strftime("%H:%M:%S"), texto)
            )

        # Objeto con clase nueva (también los que no se
        # midieron y se rechazan como DUDOSO)
        if (
            objeto in CLAVE_CONTADOR
            and
            anterior["objeto"] not in CLAVE_CONTADOR
        ):

            contadores[CLAVE_CONTADOR[objeto]] += 1

            momentos_objetos.append(ahora)


        # Mínimos y máximos operativos
        estable = (
            estado != 0
            and
            setpoint > 0
            and
            abs(abs(rpm) - setpoint) <= tolerancia_rpm(setpoint)
        )

        if estable:

            rpm_abs = abs(rpm)

            operativos["rpm_min"] = (
                rpm_abs if operativos["rpm_min"] is None
                else min(operativos["rpm_min"], rpm_abs)
            )

            operativos["rpm_max"] = (
                rpm_abs if operativos["rpm_max"] is None
                else max(operativos["rpm_max"], rpm_abs)
            )

        operativos["periodo_pid_max"] = max(
            operativos["periodo_pid_max"], periodo_pid
        )

        if clasificaciones_nuevas:

            operativos["latencia_max"] = max(
                operativos["latencia_max"], latencia
            )

        operativos["t_parada_max"] = max(
            operativos["t_parada_max"], t_parada
        )

        anterior["automata"] = automata

        anterior["objeto"] = objeto

        anterior["falla"] = falla

        anterior["t_parada"] = t_parada


def objetos_por_minuto(ahora):

    return sum(
        1 for t in momentos_objetos
        if ahora - t <= 60
    )


# ==================================================
# RECONEXIÓN
# ==================================================

def reconectar():

    # El puerto quedó inválido (Arduino desconectado o
    # reiniciado por una caída de tensión): Windows no lo
    # recupera solo. Se cierra y se reintenta abrir cada
    # 1 s hasta que vuelva

    with serial_lock:

        ser.close()

    print(
        f"Puerto {PUERTO} perdido. Reintentando... "
        "(revisá el USB y la alimentación de los servos)"
    )

    while True:

        time.sleep(1)

        # Sin tramas: la página muestra la latencia de
        # comunicación creciendo
        evaluar_latencia(time.time())

        try:

            with serial_lock:

                ser.open()

                # Al abrir el puerto el Arduino se reinicia
                time.sleep(2)

                ser.reset_input_buffer()

        except serial.SerialException:

            with serial_lock:

                ser.close()

            continue

        print(f"Reconectado a {PUERTO}")

        return


# ==================================================
# LEER DATOS DEL ARDUINO
# ==================================================

def leer_serial():

    global rpm_actual
    global setpoint_actual
    global distancia_actual
    global pwm_actual
    global estado_actual
    global objeto_actual
    global color_actual
    global clasificaciones_actual
    global latencia_actual
    global periodo_pid_actual
    global automata_actual
    global falla_actual
    global t_parada_actual
    global inyeccion_actual
    global ultima_trama

    global divergencia_activa
    global inicio_divergencia
    global ultimo_setpoint_divergencia
    global ultimo_estado_divergencia


    while True:

        try:

            # readline espera la línea completa (o el
            # timeout), sin demoras extra entre tramas
            linea = (
                ser.readline()
                .decode(
                    "utf-8",
                    errors="ignore"
                )
                .strip()
            )

        except serial.SerialException as e:

            print(
                "Error leyendo Arduino:",
                e
            )

            reconectar()

            continue


        if not linea:

            # Sin tramas: la latencia de comunicación
            # sigue creciendo
            evaluar_latencia(time.time())

            continue


        try:

                # Guardar la trama recibida
                ultima_trama = linea


                datos = linea.split(",")


                # RPM,SETPOINT,DISTANCIA,PWM,ESTADO,OBJETO,
                # R,G,B,CLASIFICACIONES,LATENCIA_MS,
                # PERIODO_PID_MS,AUTOMATA,FALLA,T_PARADA_MS,
                # INYECCION

                if len(datos) != 16:
                    continue


                rpm = float(datos[0])

                setpoint = float(datos[1])

                distancia = float(datos[2])

                pwm = float(datos[3])

                estado = int(datos[4])

                objeto = int(datos[5])

                color = (
                    int(datos[6]),
                    int(datos[7]),
                    int(datos[8])
                )

                clasificaciones = int(datos[9])

                latencia = int(datos[10])

                periodo_pid = int(datos[11])

                automata = int(datos[12])

                falla = int(datos[13])

                t_parada = int(datos[14])

                inyeccion = int(datos[15])


                rpm_actual = rpm

                setpoint_actual = setpoint

                distancia_actual = distancia

                pwm_actual = pwm

                estado_actual = estado

                objeto_actual = objeto

                color_actual = color

                clasificaciones_actual = clasificaciones

                latencia_actual = latencia

                if periodo_pid > 0:

                    periodo_pid_actual = periodo_pid

                automata_actual = automata

                falla_actual = falla

                t_parada_actual = t_parada

                inyeccion_actual = inyeccion


                ahora = time.time()

                valores_trama = datos


                # ==================================================
                # AUTÓMATA, CONTADORES Y REGISTRO
                # ==================================================

                clasificaciones_nuevas = (
                    clasificaciones
                    != seguimiento["clasificaciones"]
                    and
                    clasificaciones != 0
                )

                actualizar_automata(
                    ahora, rpm, setpoint, estado, objeto,
                    automata, falla, t_parada, periodo_pid,
                    latencia, clasificaciones_nuevas
                )

                registrar_trama(ahora, valores_trama)


                # ==================================================
                # DIVERGENCIAS DE CLASIFICACIÓN Y BANDA
                # ==================================================

                dt_trama = (
                    ahora - seguimiento["t_trama"]
                    if seguimiento["t_trama"] is not None
                    else 0.0
                )

                (
                    rpm_det, distancia_det, objeto_det,
                    color_det, latencia_det
                ) = aplicar_fallas_virtuales(
                    dt_trama, rpm, distancia, estado, objeto,
                    color, latencia
                )

                pos_sim, pos_real = evaluar_divergencias(
                    ahora,
                    rpm_det, setpoint, distancia_det, estado,
                    objeto_det, color_det, clasificaciones,
                    latencia_det, periodo_pid
                )


                # ==================================================
                # GUARDAR DATOS HISTÓRICOS
                # ==================================================

                with datos_lock:

                    tiempos.append(
                        time.time()
                    )

                    rpm_hist.append(
                        abs(rpm_actual)
                    )

                    setpoint_hist.append(
                        setpoint_actual
                    )

                    pos_real_hist.append(pos_real)

                    pos_sim_hist.append(pos_sim)


                # ==================================================
                # DETECCIÓN DE DIVERGENCIA
                # ==================================================

                ahora = time.time()

                # Orden nueva (velocidad o sentido): el
                # transitorio no cuenta como divergencia
                if (
                    abs(
                        setpoint_actual
                        -
                        ultimo_setpoint_divergencia
                    ) > 0.01
                    or
                    estado_actual != ultimo_estado_divergencia
                ):

                    ultimo_setpoint_divergencia = (
                        setpoint_actual
                    )

                    ultimo_estado_divergencia = (
                        estado_actual
                    )

                    inicio_divergencia = None

                    divergencia_activa = False


                if (
                    setpoint_actual > 0
                    and
                    estado_actual != 0
                ):


                    error = abs(
                        abs(rpm_actual)
                        -
                        setpoint_actual
                    )


                    tolerancia = tolerancia_rpm(
                        setpoint_actual
                    )


                    # ==========================================
                    # FUERA DE TOLERANCIA
                    # ==========================================

                    if error > tolerancia:


                        if inicio_divergencia is None:

                            inicio_divergencia = ahora


                        elif (
                            ahora
                            -
                            inicio_divergencia
                            >=
                            TIEMPO_DIVERGENCIA
                        ):

                            divergencia_activa = True


                    # ==========================================
                    # VOLVIÓ A LA NORMALIDAD
                    # ==========================================

                    else:

                        inicio_divergencia = None

                        divergencia_activa = False


                # ==================================================
                # MOTOR DETENIDO
                # ==================================================

                else:

                    inicio_divergencia = None

                    divergencia_activa = False


        except ValueError:

            # Trama cortada o con ruido: se descarta
            continue


# ==================================================
# HILO PARA LECTURA SERIAL
# ==================================================

print(
    "Registrando tramas en",
    os.path.join(CARPETA_REGISTROS, nuevo_registro())
)

hilo = threading.Thread(
    target=leer_serial,
    daemon=True
)

hilo.start()


# ==================================================
# ENVIAR COMANDOS AL ARDUINO
# ==================================================

def enviar(comando):

    with serial_lock:

        try:

            ser.write(
                (comando + "\n").encode()
            )

        except serial.SerialException as e:

            # Arduino desconectado: se avisa en consola en
            # lugar de romper el callback de la página
            print(
                "Error enviando al Arduino:",
                e
            )


# ==================================================
# CREAR APLICACIÓN DASH
# ==================================================

app = Dash(__name__)


# ==================================================
# DISEÑO DE LA PÁGINA
# ==================================================

app.layout = html.Div(

    style={

        "fontFamily": "Arial",

        "maxWidth": "1100px",

        "margin": "auto",

        "padding": "20px"
    },

    children=[


        # ==========================================
        # TÍTULO
        # ==========================================

        html.H1(

            "Gemelo Digital - Banda Transportadora",

            style={
                "textAlign": "center"
            }
        ),


        # ==========================================
        # CONTROL
        # ==========================================

        html.H2(
            "Control"
        ),


        html.Label(
            f"Velocidad deseada "
            f"(% de {RPM_MAX_SETPOINT:.0f} RPM)"
        ),


        dcc.Slider(

            id="velocidad",

            min=0,

            max=100,

            value=30,

            step=1,

            marks={

                0: "0%",

                25: "25%",

                50: "50%",

                75: "75%",

                100: "100%"

            }
        ),


        html.Br(),


        # ==========================================
        # BOTONES
        # ==========================================

        html.Button(

            "Adelante",

            id="adelante",

            n_clicks=0
        ),


        html.Button(

            "Reversa",

            id="reversa",

            n_clicks=0,

            style={
                "marginLeft": "10px"
            }
        ),


        html.Button(

            "STOP",

            id="stop",

            n_clicks=0,

            style={
                "marginLeft": "10px"
            }
        ),


        html.Div(

            id="mensaje",

            style={
                "marginTop": "15px"
            }
        ),


        # ==========================================
        # GANANCIAS PID
        # ==========================================

        html.H3(
            "Ganancias PID"
        ),


        html.Div(

            style={
                "display": "flex",
                "gap": "15px",
                "alignItems": "center",
                "flexWrap": "wrap"
            },

            children=[

                html.Label("Kp"),

                dcc.Input(
                    id="kp",
                    type="number",
                    value=KP_INICIAL,
                    min=0,
                    step=0.1,
                    style={"width": "80px"}
                ),

                html.Label("Ki"),

                dcc.Input(
                    id="ki",
                    type="number",
                    value=KI_INICIAL,
                    min=0,
                    step=0.1,
                    style={"width": "80px"}
                ),

                html.Label("Kd"),

                dcc.Input(
                    id="kd",
                    type="number",
                    value=KD_INICIAL,
                    min=0,
                    step=0.01,
                    style={"width": "80px"}
                ),

                html.Button(
                    "Aplicar PID",
                    id="aplicar-pid",
                    n_clicks=0
                ),
            ]
        ),


        # ==========================================
        # SEGURIDAD
        # ==========================================

        html.H3(
            "Seguridad"
        ),


        html.Button(

            "⚠ PARADA DE EMERGENCIA",

            id="parada-emergencia",

            n_clicks=0,

            style={
                "backgroundColor": "#c62828",
                "color": "white",
                "fontWeight": "bold",
                "padding": "8px 16px",
                "border": "none",
                "borderRadius": "4px"
            }
        ),


        html.Button(

            "Rearmar",

            id="rearmar",

            n_clicks=0,

            style={
                "marginLeft": "10px"
            }
        ),


        html.Div(
            "La parada desde el gemelo se suma al pulsador "
            "físico: no lo reemplaza. Rearmar sólo funciona "
            "si la causa de la falla desapareció.",
            style={
                "color": "gray",
                "fontSize": "13px",
                "marginTop": "6px"
            }
        ),


        # ==========================================
        # INYECCIÓN DE FALLAS
        # ==========================================

        html.H3(
            "Inyección de fallas"
        ),


        html.Div(

            style={
                "display": "flex",
                "gap": "40px",
                "flexWrap": "wrap"
            },

            children=[

                html.Div([

                    html.B(
                        "En el Arduino (prueban la parada "
                        "segura)"
                    ),

                    dcc.RadioItems(

                        id="inyeccion-arduino",

                        options=[
                            {
                                "label": f" {nombre}",
                                "value": valor
                            }
                            for valor, nombre
                            in NOMBRES_INYECCION.items()
                        ],

                        value=0,

                        labelStyle={"display": "block"}
                    ),
                ]),


                html.Div([

                    html.B(
                        "Virtuales en el gemelo (prueban "
                        "las divergencias)"
                    ),

                    dcc.Checklist(

                        id="fallas-virtuales",

                        options=[
                            {
                                "label": f" {nombre}",
                                "value": clave
                            }
                            for clave, nombre
                            in FALLAS_VIRTUALES.items()
                        ],

                        value=[],

                        labelStyle={"display": "block"}
                    ),
                ]),
            ]
        ),


        html.Div(

            id="mensaje-fallas",

            style={
                "marginTop": "10px"
            }
        ),


        # ==========================================
        # REGISTRO CSV
        # ==========================================

        html.H3(
            "Registro"
        ),


        html.Button(

            "Nuevo archivo de registro",

            id="nuevo-registro",

            n_clicks=0
        ),


        html.Span(

            id="registro-nombre",

            style={
                "marginLeft": "12px",
                "fontFamily": "monospace"
            }
        ),


        html.Hr(),


        # ==========================================
        # INDICADORES
        # ==========================================

        html.Div(

            style={

                "display": "flex",

                "justifyContent": "space-around",

                "textAlign": "center"
            },

            children=[


                html.Div([

                    html.H3(
                        "RPM"
                    ),

                    html.H2(
                        id="rpm-texto"
                    )

                ]),


                html.Div([

                    html.H3(
                        "Setpoint"
                    ),

                    html.H2(
                        id="setpoint-texto"
                    )

                ]),


                html.Div([

                    html.H3(
                        "PWM"
                    ),

                    html.H2(
                        id="pwm-texto"
                    )

                ]),


                html.Div([

                    html.H3(
                        "Distancia"
                    ),

                    html.H2(
                        id="distancia-texto"
                    )

                ]),


                html.Div([

                    html.H3(
                        "Objeto"
                    ),

                    html.H2(
                        id="objeto-texto"
                    )

                ]),

            ]
        ),


        # ==========================================
        # ESTADO
        # ==========================================

        html.H2(
            "Estado del sistema"
        ),


        html.Div(

            id="estado-texto",

            style={

                "fontSize": "25px",

                "fontWeight": "bold",

                "marginBottom": "20px"
            }
        ),


        # ==========================================
        # MÁQUINA DE ESTADOS
        # ==========================================

        html.H2(
            "Máquina de estados"
        ),


        html.Div(

            id="automata-texto",

            style={

                "fontSize": "25px",

                "fontWeight": "bold"
            }
        ),


        html.Div(

            id="automata-detalle",

            style={

                "whiteSpace": "pre-line",

                "fontSize": "16px",

                "margin": "10px 0 20px 0"
            }
        ),


        html.H3(
            "Contadores"
        ),


        html.Div(

            id="contadores",

            style={

                "display": "flex",

                "justifyContent": "space-around",

                "textAlign": "center",

                "marginBottom": "20px"
            }
        ),


        html.H3(
            "Mínimos y máximos operativos (desde que se "
            "abrió el gemelo)"
        ),


        html.Div(

            id="operativos",

            style={

                "whiteSpace": "pre-line",

                "fontSize": "15px",

                "marginBottom": "20px"
            }
        ),


        html.H3(
            "Registro de eventos"
        ),


        html.Div(

            id="registro-eventos",

            style={

                "backgroundColor": "#eeeeee",

                "padding": "15px",

                "borderRadius": "8px",

                "fontFamily": "monospace",

                "fontSize": "14px",

                "marginBottom": "30px"
            }
        ),


        # ==========================================
        # DIVERGENCIA
        # ==========================================

        html.H2(
            "Divergencia"
        ),


        html.Div(

            id="divergencia-texto",

            style={

                "fontSize": "20px",

                "fontWeight": "bold",

                "marginBottom": "25px"
            }
        ),


        # ==========================================
        # DIVERGENCIAS DE CLASIFICACIÓN Y BANDA
        # ==========================================

        html.H2(
            "Divergencias de clasificación y banda"
        ),


        html.Table(

            id="tabla-divergencias",

            style={

                "width": "100%",

                "borderCollapse": "collapse",

                "fontSize": "15px",

                "marginBottom": "15px"
            }
        ),


        html.H3(
            "Registro de divergencias"
        ),


        html.Div(

            id="registro-divergencias",

            style={

                "backgroundColor": "#eeeeee",

                "padding": "15px",

                "borderRadius": "8px",

                "fontFamily": "monospace",

                "fontSize": "14px",

                "marginBottom": "30px"
            }
        ),


        # ==========================================
        # DATOS SERIALES
        # ==========================================

        html.H2(
            "Datos seriales"
        ),


        html.Div(

            id="datos-seriales",

            style={

                "backgroundColor": "#eeeeee",

                "padding": "15px",

                "borderRadius": "8px",

                "fontFamily": "monospace",

                "fontSize": "17px",

                "whiteSpace": "pre-line",

                "marginBottom": "30px"
            }
        ),


        # ==========================================
        # GRÁFICA
        # ==========================================

        html.H2(
            "Velocidad"
        ),


        dcc.Graph(
            id="grafica"
        ),


        html.H2(
            "Posición del objeto"
        ),


        dcc.Graph(
            id="grafica-posicion"
        ),


        # ==========================================
        # ACTUALIZACIÓN DE PÁGINA
        # ==========================================

        dcc.Interval(

            id="actualizar",

            interval=200,

            n_intervals=0
        )

    ]
)


# ==================================================
# CONTROL DESDE LA PÁGINA
# ==================================================

@app.callback(

    Output(
        "mensaje",
        "children"
    ),

    Input(
        "velocidad",
        "value"
    ),

    Input(
        "adelante",
        "n_clicks"
    ),

    Input(
        "reversa",
        "n_clicks"
    ),

    Input(
        "stop",
        "n_clicks"
    ),

    Input(
        "aplicar-pid",
        "n_clicks"
    ),

    Input(
        "parada-emergencia",
        "n_clicks"
    ),

    Input(
        "rearmar",
        "n_clicks"
    ),

    State(
        "kp",
        "value"
    ),

    State(
        "ki",
        "value"
    ),

    State(
        "kd",
        "value"
    ),

    prevent_initial_call=True
)

def controlar(
    velocidad,
    adelante,
    reversa,
    stop,
    aplicar_pid,
    parada_emergencia,
    rearmar,
    kp,
    ki,
    kd
):

    boton = ctx.triggered_id


    # ==========================================
    # SEGURIDAD (antes de validar la velocidad: la
    # emergencia no puede depender del slider)
    # ==========================================

    if boton == "parada-emergencia":

        enviar("E")

        return "⚠ Parada de emergencia enviada"

    if boton == "rearmar":

        enviar("X")

        return (
            "Rearme enviado: si la causa desapareció queda "
            "en IDLE (arrancar con Adelante/Reversa)"
        )


    # En Dash 4 el slider tiene una caja para escribir el
    # valor: si se borra llega None y el Arduino lo leería
    # como 0 %
    if velocidad is None:

        return "Ingresá una velocidad entre 0 y 100 %"

    velocidad = int(
        round(
            min(max(velocidad, 0), 100)
        )
    )

    comando_velocidad = (
        f"V{porcentaje_arduino(velocidad)}"
    )


    # ==========================================
    # CAMBIAR VELOCIDAD
    # ==========================================

    if boton == "velocidad":

        enviar(
            comando_velocidad
        )

        rpm = (
            porcentaje_arduino(velocidad)
            / 100
            * RPM_MAX_ARDUINO
        )

        return (
            f"Velocidad seleccionada: "
            f"{velocidad}% ({rpm:.1f} RPM)"
        )


    # ==========================================
    # ADELANTE
    # ==========================================

    # Se manda la velocidad antes del sentido: el slider
    # arranca en 30 % pero ese valor nunca llegaba al
    # Arduino hasta que se movía el slider

    if boton == "adelante":

        enviar(comando_velocidad)

        enviar("F")

        return "Motor hacia adelante"


    # ==========================================
    # REVERSA
    # ==========================================

    if boton == "reversa":

        enviar(comando_velocidad)

        enviar("R")

        return "Motor en reversa"


    # ==========================================
    # GANANCIAS PID
    # ==========================================

    if boton == "aplicar-pid":

        if (
            kp is None or kp < 0
            or
            ki is None or ki < 0
            or
            kd is None or kd < 0
        ):

            return "Ganancias inválidas: deben ser números ≥ 0"

        enviar(f"K{kp:.3f},{ki:.3f},{kd:.3f}")

        return (
            f"PID aplicado: Kp={kp}  Ki={ki}  Kd={kd}"
        )


    # ==========================================
    # STOP
    # ==========================================

    if boton == "stop":

        enviar("S")

        return "Motor detenido"


    return ""


# ==================================================
# INYECCIÓN DE FALLAS
# ==================================================

@app.callback(

    Output(
        "mensaje-fallas",
        "children"
    ),

    Input(
        "inyeccion-arduino",
        "value"
    ),

    Input(
        "fallas-virtuales",
        "value"
    ),

    prevent_initial_call=True
)

def inyectar_fallas(inyeccion, virtuales):

    if ctx.triggered_id == "inyeccion-arduino":

        enviar(f"I{inyeccion}")

        return (
            "Falla en el Arduino: "
            f"{NOMBRES_INYECCION.get(inyeccion, inyeccion)}"
        )

    activas = set(virtuales or [])

    for clave in fallas_virtuales:

        fallas_virtuales[clave] = clave in activas

    if not activas:

        return "Sin fallas virtuales"

    return "Fallas virtuales activas: " + ", ".join(
        FALLAS_VIRTUALES[clave] for clave in activas
    )


# ==================================================
# REGISTRO CSV
# ==================================================

@app.callback(

    Output(
        "registro-nombre",
        "children"
    ),

    Input(
        "nuevo-registro",
        "n_clicks"
    )
)

def cambiar_registro(n):

    # Al cargar la página sólo muestra el archivo actual
    nombre = (
        nuevo_registro() if n
        else registro_csv["nombre"]
    )

    return f"registros/{nombre}"


# ==================================================
# TABLA Y REGISTRO DE DIVERGENCIAS
# ==================================================

COLOR_ESTADO = {
    "SIN DATOS": "gray",
    "OK": "green",
    "VERIFICANDO": "orange",
    "ACTIVA": "red"
}

TEXTO_ESTADO = {
    "SIN DATOS": "Sin datos",
    "OK": "✓ OK",
    "VERIFICANDO": "Verificando...",
    "ACTIVA": "⚠ ACTIVA"
}

CELDA = {
    "border": "1px solid #cccccc",
    "padding": "6px 8px",
    "textAlign": "left",
    "verticalAlign": "top"
}


def tabla_divergencias(estados):

    encabezado = html.Tr([

        html.Th(
            titulo,
            style={**CELDA, "backgroundColor": "#eeeeee"}
        )

        for titulo in (
            "Divergencia",
            "Estado",
            "Medición",
            "Causa probable",
            "Acción"
        )
    ])

    filas = []

    for clave, info in DIVERGENCIAS.items():

        estado = estados[clave]["estado"]

        activa = estado == "ACTIVA"

        filas.append(
            html.Tr(

                style={
                    "backgroundColor":
                        "#ffecec" if activa else "white"
                },

                children=[

                    html.Td(
                        info["nombre"],
                        style={**CELDA, "fontWeight": "bold"}
                    ),

                    html.Td(
                        TEXTO_ESTADO[estado],
                        style={
                            **CELDA,
                            "color": COLOR_ESTADO[estado],
                            "fontWeight": "bold",
                            "whiteSpace": "nowrap"
                        }
                    ),

                    html.Td(
                        estados[clave]["detalle"],
                        style=CELDA
                    ),

                    html.Td(
                        info["causa"],
                        style=CELDA
                    ),

                    # Sólo se indica: no se ejecuta nada
                    html.Td(
                        info["accion"],
                        style={
                            **CELDA,
                            "color": "red" if activa else "gray",
                            "fontWeight":
                                "bold" if activa else "normal"
                        }
                    ),
                ]
            )
        )

    return [
        html.Thead(encabezado),
        html.Tbody(filas)
    ]


def registro_html(registro):

    if not registro:

        return "Sin divergencias registradas"

    return [

        html.Div(
            f"{hora}  {DIVERGENCIAS[clave]['nombre']}: "
            f"{detalle}  →  {DIVERGENCIAS[clave]['accion']}"
        )

        for hora, clave, detalle in registro
    ]


# ==================================================
# ACTUALIZAR INFORMACIÓN DE LA PÁGINA
# ==================================================

@app.callback(

    Output(
        "rpm-texto",
        "children"
    ),

    Output(
        "setpoint-texto",
        "children"
    ),

    Output(
        "pwm-texto",
        "children"
    ),

    Output(
        "distancia-texto",
        "children"
    ),

    Output(
        "objeto-texto",
        "children"
    ),

    Output(
        "estado-texto",
        "children"
    ),

    Output(
        "divergencia-texto",
        "children"
    ),

    Output(
        "divergencia-texto",
        "style"
    ),

    Output(
        "datos-seriales",
        "children"
    ),

    Output(
        "grafica",
        "figure"
    ),

    Output(
        "tabla-divergencias",
        "children"
    ),

    Output(
        "registro-divergencias",
        "children"
    ),

    Output(
        "grafica-posicion",
        "figure"
    ),

    Output(
        "automata-texto",
        "children"
    ),

    Output(
        "automata-texto",
        "style"
    ),

    Output(
        "automata-detalle",
        "children"
    ),

    Output(
        "contadores",
        "children"
    ),

    Output(
        "operativos",
        "children"
    ),

    Output(
        "registro-eventos",
        "children"
    ),

    Input(
        "actualizar",
        "n_intervals"
    )
)

def actualizar_pagina(n):


    # ==========================================
    # ESTADO DEL MOTOR
    # ==========================================

    if estado_actual == 1:

        estado = "ADELANTE"

    elif estado_actual == 2:

        estado = "REVERSA"

    else:

        estado = "PARADO"


    # ==========================================
    # DISTANCIA
    # ==========================================

    if distancia_actual < 0:

        distancia = "Sin lectura"

    else:

        distancia = (
            f"{distancia_actual:.1f} cm"
        )


    # ==========================================
    # TIPO DE OBJETO (TipoObjeto en cinta_R.ino)
    # ==========================================

    objeto = NOMBRES_OBJETO.get(objeto_actual, "Sin objeto")


    # ==========================================
    # DIVERGENCIA
    # ==========================================

    error = abs(
        abs(rpm_actual)
        -
        setpoint_actual
    )


    # Motor detenido

    if (
        estado_actual == 0
        or
        setpoint_actual <= 0
    ):

        texto_divergencia = (
            "Sin evaluación - motor detenido"
        )

        estilo_divergencia = {

            "fontSize": "20px",

            "fontWeight": "bold",

            "marginBottom": "25px",

            "color": "gray"
        }


    # Divergencia confirmada

    elif divergencia_activa:

        texto_divergencia = (

            "⚠ DIVERGENCIA DE VELOCIDAD  |  "

            f"Error: {error:.2f} RPM"
        )

        estilo_divergencia = {

            "fontSize": "20px",

            "fontWeight": "bold",

            "marginBottom": "25px",

            "color": "red"
        }


    # Todavía está verificando

    elif inicio_divergencia is not None:

        texto_divergencia = (

            "Verificando posible divergencia...  |  "

            f"Error: {error:.2f} RPM"
        )

        estilo_divergencia = {

            "fontSize": "20px",

            "fontWeight": "bold",

            "marginBottom": "25px",

            "color": "orange"
        }


    # Funcionamiento normal

    else:

        texto_divergencia = (

            "✓ SIN DIVERGENCIA  |  "

            f"Error: {error:.2f} RPM"
        )

        estilo_divergencia = {

            "fontSize": "20px",

            "fontWeight": "bold",

            "marginBottom": "25px",

            "color": "green"
        }


    # ==========================================
    # DATOS SERIALES
    # ==========================================

    datos_seriales = (

        f"RPM real:      {rpm_actual:.2f} RPM\n"

        f"Setpoint:      {setpoint_actual:.2f} RPM\n"

        f"Distancia:     {distancia_actual:.2f} cm\n"

        f"PWM:           {pwm_actual:.0f}\n"

        f"Estado:        {estado}\n"

        f"Objeto:        {objeto}\n"

        f"Color R,G,B:   {color_actual[0]}, "
        f"{color_actual[1]}, {color_actual[2]} "
        f"(brillo {sum(color_actual)})\n"

        f"Clasificaciones: {clasificaciones_actual}\n"

        f"Latencia:      {latencia_actual} ms\n"

        f"Período PID:   {periodo_pid_actual} ms\n"

        f"Autómata:      "
        f"{NOMBRES_AUTOMATA.get(automata_actual, automata_actual)}"
        f" · falla {falla_actual} · t_parada "
        f"{t_parada_actual} ms · inyección "
        f"{inyeccion_actual}\n"

        f"\n"

        f"Trama recibida desde Arduino:\n"

        f"{ultima_trama}"
    )


    # ==========================================
    # CREAR GRÁFICA
    # ==========================================

    figura = go.Figure()


    # Copia consistente del historial (el hilo serial
    # lo sigue modificando mientras se dibuja)
    with datos_lock:

        tiempos_copia = list(tiempos)

        rpm_copia = list(rpm_hist)

        setpoint_copia = list(setpoint_hist)

        pos_real_copia = list(pos_real_hist)

        pos_sim_copia = list(pos_sim_hist)

        estados_copia = {
            clave: dict(valor)
            for clave, valor in estado_divergencias.items()
        }

        registro_copia = list(registro_divergencias)


    tiempo_relativo = [

        t - tiempos_copia[0]

        for t in tiempos_copia
    ]


    if len(tiempos_copia) > 0:

        # RPM reales

        figura.add_trace(

            go.Scatter(

                x=tiempo_relativo,

                y=rpm_copia,

                mode="lines",

                name="RPM reales"
            )
        )


        # Setpoint

        figura.add_trace(

            go.Scatter(

                x=tiempo_relativo,

                y=setpoint_copia,

                mode="lines",

                name="Setpoint"
            )
        )


    figura.update_layout(

        title="Respuesta del controlador PID",

        xaxis_title="Tiempo (s)",

        yaxis_title="Velocidad (RPM)",

        # Mantiene zoom/leyenda entre actualizaciones
        uirevision="fija"
    )


    # ==========================================
    # GRÁFICA DE POSICIÓN
    # ==========================================

    figura_posicion = go.Figure()


    if len(tiempos_copia) > 0:

        figura_posicion.add_trace(

            go.Scatter(

                x=tiempo_relativo,

                y=pos_real_copia,

                mode="lines",

                name="Real (ultrasonido)"
            )
        )


        figura_posicion.add_trace(

            go.Scatter(

                x=tiempo_relativo,

                y=pos_sim_copia,

                mode="lines",

                line={"dash": "dash"},

                name="Simulada (encoder)"
            )
        )


    figura_posicion.add_hline(
        y=DISTANCIA_MEDICION,
        line_dash="dot",
        line_color="gray",
        annotation_text="Medición"
    )

    figura_posicion.add_hline(
        y=POSICION_RECHAZO_CM,
        line_dash="dot",
        line_color="orange",
        annotation_text="Rechazo"
    )

    figura_posicion.add_hline(
        y=POSICION_DESVIADOR_CM,
        line_dash="dot",
        line_color="red",
        annotation_text="Desviador A/B"
    )


    figura_posicion.update_layout(

        title="Posición simulada vs real",

        xaxis_title="Tiempo (s)",

        yaxis_title="Distancia al ultrasonido (cm)",

        yaxis_range=[0, TAMANO_MAXIMO_CINTA],

        uirevision="fija"
    )


    # ==========================================
    # ACTUALIZAR COMPONENTES
    # ==========================================

    return (

        f"{abs(rpm_actual):.1f}",

        f"{setpoint_actual:.1f} RPM",

        f"{pwm_actual:.0f}",

        distancia,

        objeto,

        estado,

        texto_divergencia,

        estilo_divergencia,

        datos_seriales,

        figura,

        tabla_divergencias(estados_copia),

        registro_html(registro_copia),

        figura_posicion,

        *panel_automata()
    )


# ==================================================
# PANEL DE LA MÁQUINA DE ESTADOS
# ==================================================

def panel_automata():

    # Devuelve (texto, estilo, detalle, contadores,
    # operativos, registro de eventos)

    ahora = time.time()

    with datos_lock:

        cuentas = dict(contadores)

        eventos = list(registro_eventos)

        evento = ultimo_evento

        ops = dict(operativos)

        por_minuto = objetos_por_minuto(ahora)


    # ------------------------------------------
    # Estado actual
    # ------------------------------------------

    en_falla = automata_actual == AUTOMATA_PARADA_SEGURA

    texto = NOMBRES_AUTOMATA.get(
        automata_actual, str(automata_actual)
    )

    if en_falla:

        texto = "⚠ " + texto

    estilo = {
        "fontSize": "25px",
        "fontWeight": "bold",
        "color": (
            "red" if en_falla
            else "gray" if automata_actual == 0
            else "green"
        )
    }

    lineas = []

    if en_falla:

        lineas.append(
            "Falla: "
            + NOMBRES_FALLA.get(falla_actual, str(falla_actual))
        )

        lineas.append(
            f"Motor quieto a los {t_parada_actual} ms de la "
            f"falla (deadline {DEADLINE_PARADA_MS} ms)"
            + (
                " ⚠" if t_parada_actual > DEADLINE_PARADA_MS
                else " ✓"
            )
        )

        lineas.append(
            "Para seguir: quitar la causa y apretar Rearmar"
        )

    if inyeccion_actual:

        lineas.append(
            "Falla inyectada en el Arduino: "
            + NOMBRES_INYECCION.get(
                inyeccion_actual, str(inyeccion_actual)
            )
        )

    virtuales = [
        FALLAS_VIRTUALES[clave]
        for clave, activa in fallas_virtuales.items()
        if activa
    ]

    if virtuales:

        lineas.append(
            "Fallas virtuales: " + ", ".join(virtuales)
        )

    lineas.append(f"Último evento: {evento}")


    # Predicción de llegada al servo que le toca
    pos = seguimiento["pos_sim"]

    v = abs(rpm_actual) / 60 * AVANCE_CM_POR_VUELTA

    if pos is not None and estado_actual == 1 and v > 0:

        clase = CLASES.get(objeto_actual)

        if clase is None:

            # Sin clase todavía: el primero del camino
            nombre_servo = "rechazo (el primero del camino)"

            posicion_servo = POSICION_RECHAZO_CM

        elif clase[1] == POSICION_RECHAZO_CM:

            nombre_servo = "rechazo"

            posicion_servo = POSICION_RECHAZO_CM

        else:

            nombre_servo = "desviador A/B"

            posicion_servo = POSICION_DESVIADOR_CM

        if pos < posicion_servo:

            lineas.append(
                f"Predicción: llega al servo de "
                f"{nombre_servo} en "
                f"{(posicion_servo - pos) / v:.1f} s"
            )


    # ------------------------------------------
    # Contadores
    # ------------------------------------------

    total = (
        cuentas["A"] + cuentas["B"]
        + cuentas["rojo"] + cuentas["dudoso"]
    )

    casillas = [
        ("A (verde)", cuentas["A"]),
        ("B (azul)", cuentas["B"]),
        ("Rechazo (rojo)", cuentas["rojo"]),
        ("Dudosos", cuentas["dudoso"]),
        ("Fallas", cuentas["fallas"]),
        ("Objetos/min", por_minuto),
    ]

    contadores_html = [

        html.Div([
            html.Div(nombre, style={"color": "gray"}),
            html.H2(str(valor), style={"margin": "4px"})
        ])

        for nombre, valor in casillas
    ]


    # ------------------------------------------
    # Mínimos y máximos operativos
    # ------------------------------------------

    if ops["rpm_min"] is None:

        velocidades = "sin datos con la banda estable"

    else:

        velocidades = (
            f"{ops['rpm_min']:.1f} – {ops['rpm_max']:.1f} RPM "
            f"({ops['rpm_min'] / 60 * AVANCE_CM_POR_VUELTA:.1f}"
            f" – {ops['rpm_max'] / 60 * AVANCE_CM_POR_VUELTA:.1f}"
            " cm/s)"
        )

    tasa_dudosos = (
        f"{cuentas['dudoso'] / total:.0%}" if total
        else "sin objetos"
    )

    sincronia, _ = sincronizacion_desviador(
        0, 0, ops["latencia_max"] or None
    )

    operativos_texto = (
        f"Velocidad de banda: {velocidades}\n"
        f"Período PID máximo: {ops['periodo_pid_max']} ms "
        f"(deadline {DEADLINE_PID_MS} ms)\n"
        f"Latencia de decisión máxima: "
        f"{ops['latencia_max']} ms "
        f"(deadline {DEADLINE_DECISION_MS} ms)\n"
        f"Peor tiempo de parada: {ops['t_parada_max']} ms "
        f"(deadline {DEADLINE_PARADA_MS} ms)\n"
        f"Objetos: {total} · tasa de dudosos {tasa_dudosos}"
        f" · {por_minuto} objetos en el último minuto\n"
        f"{sincronia} (con la peor latencia medida)"
    )


    # ------------------------------------------
    # Registro de eventos
    # ------------------------------------------

    if eventos:

        eventos_html = [
            html.Div(f"{hora}  {texto_evento}")
            for hora, texto_evento in eventos
        ]

    else:

        eventos_html = "Sin eventos"


    return (
        texto,
        estilo,
        "\n".join(lineas),
        contadores_html,
        operativos_texto,
        eventos_html
    )


# ==================================================
# EJECUTAR SERVIDOR
# ==================================================

if __name__ == "__main__":

    app.run(
        debug=False,
        port=8050
    )