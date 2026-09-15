# =============================================================
# Videoregal Firmware
# Raspberry Pi Pico (MicroPython)
# Kommunikation mit dem Pi4 ueber UART (JSON, 115200 Baud)
# Siehe FWSpec.md
# =============================================================

# --- Imports -----------------------------------------------------

import time
import math
import json
from machine import Pin, PWM, UART, I2C, ADC

# --- Konfiguration ---------------------------------------------

DEBUG = True  # Debug-Ausgaben ueber USB print() einschalten

# UART zum Pi4
UART_ID = 0
UART_BAUD = 115200
UART_TX_PIN = 0    # GP0
UART_RX_PIN = 1    # GP1

# I2C fuer die INA219 Sensoren
I2C_SDA_PIN = 4    # GP4
I2C_SCL_PIN = 5    # GP5
I2C_FREQ_HZ = 400000
INA_ADDR_RAW = 0x40   # RAW Rail (Akku)
INA_ADDR_12V = 0x41   # 12V Rail
INA_ADDR_5V = 0x44    # 5V Rail
INA_CONFIG = 0x219F      # 32V Bus, +/-40mV Shunt-Range (4A * 0.01 Ohm), 12-bit, Dauerbetrieb
INA_CALIBRATION = 32768  # ergibt CurrentLSB = 0.125 mA und PowerLSB = 2.5 mW

# LEDs (PWM, Helligkeit ueber Duty Cycle)
LED_MAIN_PIN = 15  # GP15
LED_LOGO_PIN = 14  # GP14
PWM_FREQ_HZ = 40000
DIM_PERCENT = 5         # "dim" Helligkeit in %
FADE_DEFAULT_MS = 700   # Standard-Fadedauer

BREATH_MIN_PERCENT = 1  # Atmen in der Wartephase
BREATH_MAX_PERCENT = 10
BREATH_FREQ_HZ = 0.5    # Atemfrequenz in Hz (0.5 = ein Zyklus alle 2 Sekunden)

# Hook (Hoerer)
HOOK_PIN = 2             # GP2
HOOK_ACTIVE_HIGH = True  # False = Logik umkehren
HOOK_DEBOUNCE_MS = 30

# 2. Digital Input (Debug)
DEBUG_IN_PIN = 3           # GP3
DEBUG_IN_ACTIVE_HIGH = True  # False = Logik umkehren
DEBUG_IN_DEBOUNCE_MS = 30

# Helligkeits-Poti
POTI_PIN = 27              # GP27 (ADC)
POTI_HYSTERESIS_PERCENT = 5

# Temperatur
TEMP_ADC_ID = 4  # Interner Temperatursensor des RP2040

# Akku (V-Mount 4S): 16.3 V = 100 %, 12.8 V = 20 % (kritisch),
# 12.0 V = 10 %, 11.2 V = 0 % (Abschaltung). Dazwischen linear.
# Lineare LUT: Spannungsschwelle -> Ladezustand in 10%-Schritten
BAT_LUT = [
    (16.3, 100),
    (15.85, 90),
    (15.4, 80),
    (15.0, 70),
    (14.55, 60),
    (14.1, 50),
    (13.7, 40),
    (13.25, 30),
    (12.8, 20),
    (12.0, 10),
    (11.2, 0),
]

# --- Globale Variablen ------------------------------------------

uart = None       # UART Objekt, wird in init_hardware() angelegt
uart_buffer = ""  # Sammelpuffer fuer eingehende UART Daten
i2c = None        # I2C Objekt, wird in init_hardware() angelegt

pwm_main = None  # PWM Objekt fuer Main LED, wird in init_hardware() angelegt
pwm_logo = None  # PWM Objekt fuer Logo LED, wird in init_hardware() angelegt

# Fade-Zustand je LED: Startwert, Zielwert, Startzeit, Dauer, aktueller Wert,
# follow_poti=True wenn die LED auf "bright" steht und dem Poti folgen soll
led_state = {
    "main": {"from": 0.0, "target": 0.0, "start": 0, "duration": 0, "current": 0.0, "follow_poti": False},
    "logo": {"from": 0.0, "target": 0.0, "start": 0, "duration": 0, "current": 0.0, "follow_poti": False},
}

hook_event_flag = False  # wird vom Hook-Interrupt gesetzt
hook_event_time = 0      # Zeitpunkt des letzten Hook-Interrupts (fuer Debounce)
hook_active = False      # True = Hoerer abgehoben
hook_pin = None          # Pin Objekt, wird in init_hardware() angelegt

debug_in_event_flag = False  # wird vom Debug-Input-Interrupt gesetzt
debug_in_event_time = 0      # Zeitpunkt des letzten Debug-Input-Interrupts (fuer Debounce)
debug_in_active = False      # True = Debug-Input aktiv
debug_in_pin = None          # Pin Objekt, wird in init_hardware() angelegt

poti_adc = None          # ADC Objekt, wird in init_hardware() angelegt
bright_percent = 80      # aktuell gueltiger "bright"-Wert aus dem Poti (mit Hysterese)
temp_adc = None          # ADC Objekt (interner Temp-Sensor), wird in init_hardware() angelegt

host_connected = False   # True ab erster Nachricht vom Pi4

# --- Hardware Initialisierung -----------------------------------

def init_hardware():
    global uart, i2c, pwm_main, pwm_logo, hook_pin, hook_active
    global poti_adc, bright_percent, debug_in_pin, debug_in_active, temp_adc

    # UART zum Pi4 (rxbuf: genug Platz fuer eine volle JSON-Zeile)
    uart = UART(UART_ID, baudrate=UART_BAUD, tx=UART_TX_PIN, rx=UART_RX_PIN, rxbuf=256)

    # I2C fuer die INA219 Sensoren
    i2c = I2C(0, sda=Pin(I2C_SDA_PIN), scl=Pin(I2C_SCL_PIN), freq=I2C_FREQ_HZ)
    if DEBUG:
        print("I2C Geraete:", [hex(a) for a in i2c.scan()])
    init_ina219(INA_ADDR_RAW)
    init_ina219(INA_ADDR_12V)
    init_ina219(INA_ADDR_5V)

    # LEDs als PWM konfigurieren
    pwm_main = PWM(Pin(LED_MAIN_PIN))
    pwm_logo = PWM(Pin(LED_LOGO_PIN))
    pwm_main.freq(PWM_FREQ_HZ)
    pwm_logo.freq(PWM_FREQ_HZ)

    # Hook (Hoerer). Pull-up: Ruhe-Opfer liegt an, abgehoben = high.
    # Bei anderer Verdrahtung HOOK_ACTIVE_HIGH auf False setzen.
    hook_pin = Pin(HOOK_PIN, Pin.IN, Pin.PULL_UP)
    hook_pin.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=hook_interrupt)

    # Startzustand einlesen
    hook_active = is_hook_active_now()

    # 2. Digital Input (Debug), gleiche Logik wie Hook
    debug_in_pin = Pin(DEBUG_IN_PIN, Pin.IN, Pin.PULL_UP)
    debug_in_pin.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=debug_in_interrupt)
    debug_in_active = is_debug_in_active_now()

    # Helligkeits-Poti
    poti_adc = ADC(POTI_PIN)
    bright_percent = read_poti_percent()  # Startwert einmal ohne Hysterese lesen

    # Interner Temperatursensor
    temp_adc = ADC(TEMP_ADC_ID)

# --- Helligkeits-Poti -----------------------------------------------

def read_poti_percent():
    # Poti an GP27 auslesen, 0-65535 -> DIM_PERCENT bis 100 %
    # ("bright" ist immer mindestens so hell wie "dim")
    return DIM_PERCENT + poti_adc.read_u16() * (100 - DIM_PERCENT) / 65535

def update_bright_percent():
    # Poti mit Hysterese auswerten: Der Wert aendert sich nur, wenn die
    # neue Messung mehr als POTI_HYSTERESIS_PERCENT vom aktuellen Wert
    # abweicht. So sorgt das ADC-Rauschen nicht fuer stetiges Zappeln.
    global bright_percent

    measured = read_poti_percent()
    if abs(measured - bright_percent) > POTI_HYSTERESIS_PERCENT:
        bright_percent = measured
        # LEDs auf "bright" sofort auf den neuen Wert setzen (kein Fade)
        for name in ("main", "logo"):
            if led_state[name]["follow_poti"]:
                start_fade(name, bright_percent, 100, True)

def get_bright_percent():
    # Wert fuer das Keyword "bright". Wird vom Poti bestimmt.
    return bright_percent

# --- LED Steuerung ------------------------------------------------

def set_led_percent(pwm, percent):
    # 0-100 % in 16-bit Duty Cycle umrechnen und setzen
    duty = int(percent * 65535 / 100)
    pwm.duty_u16(duty)

def get_pwm(name):
    # Liefert das PWM Objekt zur LED ("main" oder "logo")
    if name == "main":
        return pwm_main
    return pwm_logo

def start_fade(name, target, duration_ms, follow_poti=False):
    # Setzt ein Fade-Ziel. follow_poti=True: LED steht auf "bright" und
    # zieht spaeter von selbst mit, wenn am Poti gedreht wird.
    state = led_state[name]
    state["from"] = state["current"]
    state["target"] = target
    state["start"] = time.ticks_ms()
    state["duration"] = duration_ms
    state["follow_poti"] = follow_poti
    if duration_ms <= 0:
        # Ohne Fade: sofort setzen
        state["current"] = target
        set_led_percent(get_pwm(name), target)

def update_led_fades():
    # Tickt jeden Loop-Durchlauf und bewegt laufende Fades weiter (non-blocking).
    now = time.ticks_ms()
    for name in led_state:
        state = led_state[name]
        if state["current"] == state["target"]:
            continue  # nichts zu tun
        elapsed = time.ticks_diff(now, state["start"])
        if state["duration"] <= 0 or elapsed >= state["duration"]:
            state["current"] = state["target"]
        else:
            progress = elapsed / state["duration"]
            state["current"] = state["from"] + (state["target"] - state["from"]) * progress
        set_led_percent(get_pwm(name), state["current"])

def resolve_led_value(value):
    # Zahl (0-100) oder Keyword ("bright"/"dim") aufloesen.
    # Liefert (prozent, follow_poti) oder (None, False) bei ungueltigem Wert.
    if value == "bright":
        return get_bright_percent(), True
    if value == "dim":
        return DIM_PERCENT, False
    # bool ist in Python auch int -> explizit ausschliessen
    if isinstance(value, (int, float)) and not isinstance(value, bool) and 0 <= value <= 100:
        return value, False
    return None, False

def is_hook_active_now():
    # Pinzustand lesen, mit HOOK_ACTIVE_HIGH umkehrbar
    raw = hook_pin.value()
    if HOOK_ACTIVE_HIGH:
        return raw == 1
    return raw == 0

# --- 2. Digital Input (Debug) ---------------------------------------

def debug_in_interrupt(pin):
    # Nur Flag + Zeitpunkt merken, alles andere macht der Loop.
    global debug_in_event_flag, debug_in_event_time
    debug_in_event_flag = True
    debug_in_event_time = time.ticks_ms()

def is_debug_in_active_now():
    # Pinzustand lesen, mit DEBUG_IN_ACTIVE_HIGH umkehrbar
    raw = debug_in_pin.value()
    if DEBUG_IN_ACTIVE_HIGH:
        return raw == 1
    return raw == 0

def handle_debug_in_event():
    # Wird im Loop aufgerufen, wenn der Interrupt eine Flanke gemeldet hat.
    # Debounce wie beim Hook.
    global debug_in_event_flag, debug_in_active

    if time.ticks_diff(time.ticks_ms(), debug_in_event_time) < DEBUG_IN_DEBOUNCE_MS:
        return  # Debounce noch nicht um, Flag bleibt gesetzt

    debug_in_event_flag = False

    new_active = is_debug_in_active_now()
    if new_active == debug_in_active:
        return  # nur Prellen, keine echte Aenderung

    debug_in_active = new_active
    send_json({"event": "debug", "state": "active" if debug_in_active else "inactive"})

# --- Startphase: warten auf den Pi4 ------------------------------

def wait_for_host():
    global host_connected

    # LEDs atmen langsam (Werte siehe Konfiguration oben),
    # bis die erste Nachricht vom Pi4 kommt.

    # Test: Lebenszeichen an den Pi senden (prueft TX-Richtung)
    send_json({"event": "boot"})

    BREATH_PERIOD_MS = int(1000 / BREATH_FREQ_HZ)
    BREATH_MID = (BREATH_MAX_PERCENT + BREATH_MIN_PERCENT) / 2
    BREATH_AMP = (BREATH_MAX_PERCENT - BREATH_MIN_PERCENT) / 2

    while True:
        t = time.ticks_ms() % BREATH_PERIOD_MS

        # Sinuskurve: 0 -> Maximum -> 0 -> Minimum -> 0
        phase = 2 * math.pi * t / BREATH_PERIOD_MS
        percent = BREATH_MID + BREATH_AMP * math.sin(phase)

        # led_state mitfuehren, damit der erste Fade nach dem Atmen
        # von der aktuellen Atem-Helligkeit starten kann
        led_state["main"]["current"] = percent
        led_state["main"]["target"] = percent
        led_state["logo"]["current"] = percent
        led_state["logo"]["target"] = percent

        set_led_percent(pwm_main, percent)
        set_led_percent(pwm_logo, percent)

        # Erste Nachricht vom Pi4: uebergehen in den normalen Betrieb
        msg = read_message()
        if msg is not None:
            host_connected = True
            handle_message(msg)  # direkt beantworten (z.B. get_telemetry)
            apply_hook_leds()    # LEDs aus der Atem-Phase in Hook-Grundstellung
            break

        time.sleep_ms(20)

# --- UART Nachrichten --------------------------------------------

def read_message():
    # Liest alle neuen Zeichen vom UART und sucht nach einer Zeile.
    # Gibt geparstes JSON zurueck oder None, wenn keine (gueltige) Zeile da ist.
    global uart_buffer

    n = uart.any()
    if n == 0:
        return None

    data = uart.read(n).decode()

    if DEBUG:
        print("UART roh empfangen:", repr(data))

    uart_buffer += data

    # Schutz: Muell ohne Zeilenende darf den Puffer nicht endlos wachsen lassen
    if len(uart_buffer) > 512:
        uart_buffer = ""
        if DEBUG:
            print("UART: Puffer uebergelaufen, verworfen")
        return None

    if "\n" in uart_buffer:
        line, uart_buffer = uart_buffer.split("\n", 1)
    elif "\r" in uart_buffer:
        line, uart_buffer = uart_buffer.split("\r", 1)
    else:
        return None

    line = line.strip()
    if not line:
        return None

    if DEBUG:
        print("UART empfangen:", line)

    try:
        return json.loads(line)
    except ValueError:
        if DEBUG:
            print("UART: ungültiges JSON:", line)
        return None

def send_json(obj):
    uart.write((json.dumps(obj) + "\n").encode())

def handle_message(msg):
    cmd = msg.get("cmd", "")

    if cmd == "set_led":
        handle_set_led(msg)
    elif cmd == "get_telemetry":
        send_json(get_telemetry_response())
    else:
        # Noch nicht implementierte Befehle: nur Error antworten
        send_json({"status": "error", "cmd": cmd, "data": {"error": "unknown command"}})

def handle_set_led(msg):
    fade_ms = msg.get("fade", FADE_DEFAULT_MS)  # optional, Default aus Config
    result = {}

    for name in ("main", "logo"):
        if name not in msg:
            continue
        percent, follow_poti = resolve_led_value(msg[name])
        if percent is None:
            send_json({"status": "error", "cmd": "set_led",
                       "data": {"error": "invalid value for '" + name + "'"}})
            return
        start_fade(name, percent, fade_ms, follow_poti)
        result[name] = percent

    send_json({"status": "ok", "cmd": "set_led", "data": result})

# --- Hook (Hoerer) ------------------------------------------------

def hook_interrupt(pin):
    # Nur Flag + Zeitpunkt merken, alles andere macht der Loop.
    global hook_event_flag, hook_event_time
    hook_event_flag = True
    hook_event_time = time.ticks_ms()

def handle_hook_event():
    # Wird im Loop aufgerufen, wenn der Interrupt ein Flanke gemeldet hat.
    # Debounce: erst reagieren, wenn seit der letzten Flanke nichts mehr kam.
    global hook_event_flag, hook_active

    if time.ticks_diff(time.ticks_ms(), hook_event_time) < HOOK_DEBOUNCE_MS:
        return  # Debounce noch nicht um, Flag bleibt gesetzt, naechster Loop versucht es wieder

    hook_event_flag = False

    new_active = is_hook_active_now()
    if new_active == hook_active:
        return  # nur Prellen, keine echte Aenderung

    hook_active = new_active
    send_json({"event": "hook", "state": "active" if hook_active else "inactive"})
    apply_hook_leds()

def apply_hook_leds():
    # LED-Grundstellung je nach Hook-Zustand (Spec: active = main bright / logo dim)
    if hook_active:
        start_fade("main", get_bright_percent(), FADE_DEFAULT_MS, True)
        start_fade("logo", DIM_PERCENT, FADE_DEFAULT_MS, False)
    else:
        start_fade("main", DIM_PERCENT, FADE_DEFAULT_MS, False)
        start_fade("logo", get_bright_percent(), FADE_DEFAULT_MS, True)

# --- INA219 Sensoren ----------------------------------------------

INA_REG_CONFIG = 0x00
INA_REG_BUSV = 0x02
INA_REG_POWER = 0x03
INA_REG_CURRENT = 0x04
INA_REG_CAL = 0x05

def i2c_write_reg(addr, reg, value):
    i2c.writeto_mem(addr, reg, bytes([value >> 8, value & 0xFF]))

def i2c_read_reg(addr, reg):
    data = i2c.readfrom_mem(addr, reg, 2)
    return (data[0] << 8) | data[1]

def to_signed16(value):
    # 16-bit Registerwerte sind vorzeichenbehaftet
    if value >= 0x8000:
        value -= 0x10000
    return value

def init_ina219(addr):
    # Konfiguration + Kalibrierung schreiben
    i2c_write_reg(addr, INA_REG_CONFIG, INA_CONFIG)
    i2c_write_reg(addr, INA_REG_CAL, INA_CALIBRATION)

def read_ina219(addr):
    # Liest Busspannung (V), Strom (mA) und Leistung (mW) eines INA219.
    try:
        bus_raw = i2c_read_reg(addr, INA_REG_BUSV)
        volts = (bus_raw >> 3) * 0.004            # LSB 4 mV
        current_raw = to_signed16(i2c_read_reg(addr, INA_REG_CURRENT))
        milliamp = current_raw * 0.125            # CurrentLSB 0.125 mA
        power_raw = to_signed16(i2c_read_reg(addr, INA_REG_POWER))
        milliwatt = power_raw * 2.5               # PowerLSB 2.5 mW
        return volts, milliamp, milliwatt
    except OSError:
        return 0.0, 0, 0  # Sensor nicht erreichbar -> 0 statt Absturz

def get_battery_percent(volts):
    # Liest den Ladezustand aus der LUT. Erste Schwelle, die unterschritten
    # wird nicht: Es gilt die letzte Schwelle, die noch erreicht ist.
    for threshold, percent in BAT_LUT:
        if volts >= threshold:
            return percent
    return 0  # unter 12.0 V -> 0 % (Abschaltung bei 11.2 V)

def read_cpu_temp():
    # Interner Temperatursensor des RP2040 (ADC Kanal 4)
    voltage = temp_adc.read_u16() * 3.3 / 65535
    return 27 - (voltage - 0.706) / 0.001721

# --- Telemetrie ---------------------------------------------------

def get_telemetry_response():
    # Liest die 3 Power Rails, Akku, Temperaturen, Poti-Wert und
    # Digital-Input-Zustaende und baut die Antwort-Nachricht.
    raw_v, raw_c, raw_p = read_ina219(INA_ADDR_RAW)
    v12_v, v12_c, v12_p = read_ina219(INA_ADDR_12V)
    v5_v, v5_c, v5_p = read_ina219(INA_ADDR_5V)

    return {
        "status": "ok",
        "cmd": "get_telemetry",
        "data": {
            "rails": {
                "raw": {"v": round(raw_v, 2), "c": round(raw_c), "p": round(raw_p)},
                "12v": {"v": round(v12_v, 2), "c": round(v12_c), "p": round(v12_p)},
                "5v1": {"v": round(v5_v, 2), "c": round(v5_c), "p": round(v5_p)},
            },
            "bat": get_battery_percent(raw_v),
            "temp": {"case": None, "cpu": round(read_cpu_temp(), 1)},  # case = Gehaeuse, folgt
            "poti": {"bright": round(bright_percent, 1)},
            "digital_in": {"hook": hook_active, "debug": debug_in_active},
        },
    }

# --- Setup und Loop ------------------------------------------------

def setup():
    init_hardware()
    print(get_telemetry_response())  # einmal lesen, damit die INA219 Sensoren initialisiert werden

def loop():
    msg = read_message()
    if msg is not None:
        handle_message(msg)

    update_bright_percent()
    update_led_fades()

    if hook_event_flag:
        handle_hook_event()

    if debug_in_event_flag:
        handle_debug_in_event()

# --- Programmstart ---------------------------------------------------

setup()
wait_for_host()

while True:
    loop()
