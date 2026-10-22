# =============================================================
# Videoregal Firmware — Driver Board Rev 2.1
# Waveshare RP2350-Zero (MicroPython, RP2350A), aufgeloetet auf das
# Driver Board Rev 2.1
# Kommunikation mit dem Pi4 ueber UART (JSON, 115200 Baud)
# Siehe FWSpec.md (Pin-Map + Flashing je Box) und pcb_ref/
# (Schaltplan, Layout, RP2350-Zero-Pinout)
#
# Hardware — fuer ALLE Boxen identisch (Rev 2.1):
#   - RP2350-Zero (Waveshare) mit MicroPython v1.29.0
#     (offizieller RPI_PICO2-Build, siehe FWSpec.md "Flashing je Box")
#   - MAIN_LED GP15, LOGO_LED GP14 (PWM, PWM_FREQ_HZ)
#   - Status-LED = Onboard-WS2812 des RP2350-Zero auf GP16.
#     GP16 ist auf dem PCB NICHT herausgefuehrt (Waveshare-Pinout:
#     "WS2812 RGB LED used pin: GP16 | DIN") — der Pin gehoert
#     allein der Status-LED, kein PWM-Ausgang liegt darauf.
#   - DIN1 (Hook) GP2, DIN2 GP3, AIN1 (Helligkeits-Poti) GP27,
#     AIN2 GP26, I2C SDA/SCL GP4/GP5, UART TX/RX GP0/GP1
#
# Protokoll (seit der Portierung unveraendert): boot-Event,
# wait_for_host-Breathing, Hook-Debounce, set_led/get_telemetry.
#   - analog_in.ain2 (additiv); digital_in.debug bleibt Schluesselname
#     fuer DIN2/GP3 (Protokoll-Kompatibilitaet zur Pi-Haupt-App v2)
#   - Rev 2.1.3: Neu "bat_charging" (bool) im get_telemetry-Contract
#     (additiv). bat-p bekommt das Vorzeichen von bat-c, weil das
#     INA219-Power-Register staets positive Werte meldet
#     (Laden -> c < 0 und p < 0).
#   - Rev 2.1.4: Status-LED (WS2812) ueberarbeitet — feste 10 %
#     Helligkeit (STATUS_BRIGHT = 26), Startup gelb, Fehler rot,
#     ready gruen, blauer Kurzblitz bei jeder Nachricht rein UND
#     raus (vorher nur RX). GP16 wird schon beim Import auf low
#     gezogen (weniger weisses Boot-Latchen).
#   - Rev 2.1.5: Nur noch EINE Hardware (RP2350-Zero + PCB 2.1).
#     BOARD_REV-Weiche entfernt, Pins fest (Main GP15 / Logo GP14 /
#     WS2812 GP16), Kommentare und Doku auf diese Hardware gezogen.

#
# RP2350-Hinweise (MicroPython):
#   - ADC-Kanalzuordnung: GP26 -> ADC0, GP27 -> ADC1, GP28 -> ADC2.
#     machine.ADC(pin) mappt automatisch richtig; der interne
#     Temperatursensor ist Kanal 4 (ADC(4)).
#   - PWM-Slices: 12 Slices (je 2 Channels), Zuordnung
#     slice = (pin // 2) % 12, channel = pin % 2.
#       GP15 (MAIN) -> Slice 7, Channel 1
#       GP14 (LOGO) -> Slice 7, Channel 0
#     => MAIN und LOGO liegen auf demselben Slice 7, aber auf
#     verschiedenen Channels: die Duty-Cycles sind unabhaengig, die
#     Frequenz ist fuer beide gleich (PWM_FREQ_HZ). pwm_main und
#     pwm_logo bekommen je ein eigenes PWM-Objekt.
# =============================================================

# --- Imports -----------------------------------------------------

import gc
import time
import math
import json
import os
from machine import Pin, PWM, UART, I2C, ADC

# --- Konfiguration ---------------------------------------------

DEBUG = True  # Debug-Ausgaben ueber USB print() einschalten

# Boot-Marker ganz am Anfang. WICHTIG: Prints, die vor dem Verbinden des
# USB-Hosts (REPL) passieren, werden vom RP2350 verworfen — der
# Heartbeat in wait_for_host() sorgt dafuer, dass man nach dem Verbinden
# trotzdem lebt-der-Pico-Ausgaben sieht.
# Firmware-Versionsnummer. Wird beim Start ausgegeben und in der
# Telemetrie mitgeliefert (Pi-App kann die Firmware-Version anzeigen).
FIRMWARE_VERSION = "2.1.5"

print("=== main.py Boot === Firmware", FIRMWARE_VERSION)

# UART zum Pi4
UART_ID = 0
UART_BAUD = 115200
UART_TX_PIN = 0    # GP0
UART_RX_PIN = 1    # GP1
SEND_TIMEOUT_MS = 200  # Notausstieg, falls der UART dauerhaft blockiert

# I2C fuer die INA219 Sensoren
I2C_SDA_PIN = 4    # GP4
I2C_SCL_PIN = 5    # GP5
I2C_FREQ_HZ = 400000
INA_ADDR_RAW = 0x40   # RAW Rail (Akku)
INA_ADDR_12V = 0x41   # 12V Rail
INA_ADDR_5V = 0x44    # 5V Rail
INA_ADDR_BAT = 0x45   # BAT Rail (A1=VS+, A0=VS+) — Akku-Strom,
                      # Vorzeichen: c < 0 = Laden, c > 0 = Entladen
INA_CONFIG = 0x219F      # 32V Bus, +/-40mV Shunt-Range (4A * 0.01 Ohm), 12-bit, Dauerbetrieb
INA_CALIBRATION = 32768  # ergibt CurrentLSB = 0.125 mA und PowerLSB = 2.5 mW

# LEDs (PWM, Helligkeit ueber Duty Cycle)
# EINE Hardware fuer alle Boxen: RP2350-Zero auf Driver Board Rev 2.1.
# Pin-Map der 2.1-Platine (pcb_ref/Schematic_2609 ... 2.1):
#   MAIN_LED = GP15, LOGO_LED = GP14
#   GP16 ist auf dem PCB NICHT herausgefuehrt und gehoert im Modul
#   fest zur Onboard-WS2812 (dient nur als Status-LED).
LED_MAIN_PIN = 15  # GP15 -> MAIN_LED
LED_LOGO_PIN = 14  # GP14 -> LOGO_LED
WS2812_PIN = 16    # GP16 -> Onboard-WS2812 des RP2350-Zero (Status-LED)

# Status-Farben der WS2812. STATUS_BRIGHT ist die maximale Kanalintensitaet
# und entspricht IMMER 10 % der vollen 8-Bit-Skala (255 * 0.10 = 25.5 -> 26).
# Die Status-LED ist vom Poti/Atmen unabhaengig, die Helligkeit ist damit fest.
# Main-/Logo-PWM-LEDs (DIM_PERCENT/bright) sind davon unberuehrt.
STATUS_BRIGHT = 26
STATUS_RED = (STATUS_BRIGHT, 0, 0)                 # Fehler
STATUS_YELLOW = (STATUS_BRIGHT, STATUS_BRIGHT, 0)  # Startup / wartet auf Host
STATUS_GREEN = (0, STATUS_BRIGHT, 0)               # ready (Host verbunden)
STATUS_BLUE = (0, 0, STATUS_BRIGHT)                # Nachricht rein/raus (Kurzblitz)

# Kanalreihenfolge der LED. Das neopixel-Modul schreibt standardmaessig
# GRB (WS2812-Datenblatt). Zeigt die LED die falsche Farbe (z. B. GRUEN
# erscheint als ROT), ist die Verdrahtung/Variante RGB — dann hier
# "RGB" eintragen.
WS2812_ORDER = "RGB"

# GP16 so frueh wie moeglich hart auf low treiben: Der WS2812 hat einen
# eigenen Controller im Gehaeuse und latched waehrend des Pico/MicroPython-
# Boots die Noise auf der noch offenen (floating) Datenleitung — typisch
# WEISS. Ein low-Pegel verhindert weitere Zufallsrahmen, bis
# set_status_led() den ersten gueltigen 24-Bit-Rahmen schreibt.
_status_led_pin = Pin(WS2812_PIN, Pin.OUT, value=0)

_ws2812_np = None  # gecachtes NeoPixel-Objekt (lazy in set_status_led)

# Diagnostik:
#   - Nachricht rein (RX, gueltig) ODER raus (TX, send_json): kurzer
#     BLAUER Blitz (RX_FLASH_MS), danach schaltet update_status_led()
#     zurueck auf den Status (gruen, bzw. gelb im Wartemodus). Blinkt nur
#     pro Nachricht — Fragmente auf der Leitung verlaengern die Deadline
#     nicht, die LED bleibt also nicht haengen.
#   - Ungueltige Nachricht (Binaermuell / kein JSON / kein Objekt): ROT
#     statt gelb, danach ebenfalls zurueck auf den Status. Rot hat
#     Vorrang und wird von einem Blau-Blitz nicht ueberdeckt.
RX_FLASH_MS = 300
_rx_flash_until = None

# Soft-Fehler (z. B. ungueltige Nachricht): kurz ROT zeigen, dann im
# Control-Loop zurueck auf gruen (bzw. gelb, solange nicht ready).
STATUS_ERROR_HOLD_MS = 200
_status_error_until = None  # ticks_ms-Deadline oder None


def signal_soft_error():
    # Nicht-kritischer Fehler: kurz rot, danach wieder den normalen Status
    # (gruen wenn ready, sonst gelb — der Loop schaltet zurueck, sobald
    # die Deadline abgelaufen ist).
    global _status_error_until
    set_status_led(STATUS_RED)
    _status_error_until = time.ticks_add(time.ticks_ms(), STATUS_ERROR_HOLD_MS)


def flash_blue():
    # Nachricht rein (RX) oder raus (TX): kurzer blauer Blitz, danach
    # schaltet update_status_led() zurueck auf den normalen Status.
    # Ein aktiver Fehler (rot) hat Vorrang und wird nicht ueberdeckt.
    global _rx_flash_until
    if _status_error_until is not None:
        return
    set_status_led(STATUS_BLUE)
    _rx_flash_until = time.ticks_add(time.ticks_ms(), RX_FLASH_MS)


def flash_rx_ok():
    # Gueltige Nachricht am UART (RX) -> kurzer blauer Blitz.
    flash_blue()


def update_status_led():
    # Teil des Control-Loops: Blau-Blitz (RX/TX) ausblenden und Soft-Fehler
    # (rot) zurueck auf den normalen Status schalten.
    global _status_error_until, _rx_flash_until
    now = time.ticks_ms()
    if _rx_flash_until is not None:
        if time.ticks_diff(now, _rx_flash_until) >= 0:
            _rx_flash_until = None
            if _status_error_until is None:  # Fehler hat Vorrang
                _restore_status()
    if _status_error_until is not None:
        if time.ticks_diff(now, _status_error_until) >= 0:
            _status_error_until = None
            _restore_status()


def _restore_status():
    # Normaler Status zurueck: gruen wenn ready, sonst gelb (Wartemodus).
    if host_connected:
        set_status_led(STATUS_GREEN)
    else:
        set_status_led(STATUS_YELLOW)


def set_status_led(color):
    # Setzt die Status-LED (Onboard-WS2812 des RP2350-Zero auf GP16).
    # Ein gueltiger 24-Bit-Rahmen ersetzt auch eine vorher gelatchte
    # Farbe (z. B. weiss nach dem Boot). Der Pin ist oben schon als
    # Ausgang/low angelegt. Fehler duerfen den Betrieb nie blockieren.
    global _ws2812_np
    try:
        if _ws2812_np is None:
            from neopixel import NeoPixel
            _ws2812_np = NeoPixel(_status_led_pin, 1)
        if WS2812_ORDER == "RGB":
            # RGB-Variante: R und G tauschen, damit das neopixel-Modul
            # (GRB-Buffer) physikalisch R/G/B an die LED liefert.
            color = (color[1], color[0], color[2])
        _ws2812_np[0] = color
        _ws2812_np.write()
        if DEBUG:
            print("Status-LED:", color)
    except Exception as e:
        if DEBUG:
            print("Status-LED fehlgeschlagen:", e)
PWM_FREQ_HZ = 40000
DIM_PERCENT = 5         # "dim" Helligkeit in %
FADE_DEFAULT_MS = 700   # Standard-Fadedauer

BREATH_MIN_PERCENT = 1  # Atmen in der Wartephase
BREATH_MAX_PERCENT = 10
BREATH_FREQ_HZ = 0.5    # Atemfrequenz in Hz (0.5 = ein Zyklus alle 2 Sekunden)

# DIN1: Hook (Hoerer)
# HOOK_ACTIVE_HIGH ist NUR noch der Default-Wert (erste Box / keine
# Config-Datei). Die Auswertung laeuft ueber die Laufzeit-Variable
# hook_active_high (per UART umschaltbar, im Flash gespeichert).
HOOK_PIN = 2             # GP2 (unveraendert)
HOOK_ACTIVE_HIGH = True  # Default-Polaritaet (wie bisher)
HOOK_DEBOUNCE_MS = 30

# Persistenz-File im Flash des RP2350-Zero. WICHTIG: MicroPython
# mountet das interne Flash am ROOT "/", nicht "/flash" (das ist eine
# ESP32-Konvention - unter Pico-MicroPython scheitert /flash still).
HOOK_CONFIG_FILE = "/hook_config.json"

# DIN2: 2. Digital-Input (GP3, war Debug-Input, jetzt offizieller Input)
DIN2_PIN = 3              # GP3
DIN2_ACTIVE_HIGH = True   # False = Logik umkehren
DIN2_DEBOUNCE_MS = 30

# AIN1: Helligkeits-Poti (unveraendert)
POTI_PIN = 27              # GP27 (ADC1)
POTI_HYSTERESIS_PERCENT = 5

# AIN2: 2. Analog-Input (NEU, GP26 = ADC0)
AIN2_PIN = 26               # GP26 (ADC0)
AIN2_HYSTERESIS_PERCENT = 5

# Temperatur
TEMP_ADC_ID = 4  # Interner Temperatursensor des RP2350 (ADC-Kanal 4)

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
hook_active_high = HOOK_ACTIVE_HIGH  # Laufzeit-Polaritaet, beim Boot aus Flash geladen
hook_pin = None          # Pin Objekt, wird in init_hardware() angelegt

din2_event_flag = False  # wird vom DIN2-Interrupt gesetzt
din2_event_time = 0      # Zeitpunkt des letzten DIN2-Interrupts (fuer Debounce)
din2_active = False      # True = DIN2 aktiv
din2_pin = None          # Pin Objekt, wird in init_hardware() angelegt

poti_adc = None          # ADC Objekt, wird in init_hardware() angelegt
bright_percent = 80      # aktuell gueltiger "bright"-Wert aus dem Poti (mit Hysterese)
ain2_adc = None          # ADC Objekt fuer AIN2, wird in init_hardware() angelegt
ain2_percent = 0.0       # zuletzt gueltiger AIN2-Wert in % (mit Hysterese)
temp_adc = None          # ADC Objekt (interner Temp-Sensor), wird in init_hardware() angelegt

host_connected = False   # True ab erster Nachricht vom Pi4

# --- Hardware Initialisierung -----------------------------------

def init_hardware():
    global uart, i2c, pwm_main, pwm_logo, hook_pin, hook_active
    global poti_adc, bright_percent, din2_pin, din2_active, temp_adc
    global ain2_adc, ain2_percent

    # UART zum Pi4 (rxbuf: genug Platz fuer eine volle JSON-Zeile)
    uart = UART(UART_ID, baudrate=UART_BAUD, tx=UART_TX_PIN, rx=UART_RX_PIN, rxbuf=256)

    # I2C fuer die INA219 Sensoren
    i2c = I2C(0, sda=Pin(I2C_SDA_PIN), scl=Pin(I2C_SCL_PIN), freq=I2C_FREQ_HZ)
    if DEBUG:
        print("I2C Geraete:", [hex(a) for a in i2c.scan()])
    init_ina219(INA_ADDR_RAW)
    init_ina219(INA_ADDR_12V)
    init_ina219(INA_ADDR_5V)
    init_ina219(INA_ADDR_BAT)

    # WS2812-Status-LED (Onboard-LED des RP2350-Zero, GP16): wird von
    # set_status_led() verwaltet — setup() setzt sie noch VOR
    # init_hardware() auf gelb (Startup). Der Pin ist bereits beim
    # Import auf low gezogen, ein gueltiger 24-Bit-Rahmen ueberschreibt
    # damit auch eine waehrend des Boots evtl. gelatchte weisse Farbe.

    # LEDs als PWM konfigurieren
    # RP2350: GP15 (MAIN) -> Slice 7/Ch1, GP14 (LOGO) -> Slice 7/Ch0
    # (siehe Kommentar im Dateikopf). Einzelnes PWM-Objekt je LED.
    # (GP16 ist oben auf low gezogen und bleibt allein der
    # Onboard-WS2812 — kein PWM-Ausgang kann dort kollidieren.)
    pwm_main = PWM(Pin(LED_MAIN_PIN))
    pwm_logo = PWM(Pin(LED_LOGO_PIN))
    pwm_main.freq(PWM_FREQ_HZ)
    pwm_logo.freq(PWM_FREQ_HZ)

    # DIN1: Hook (Hoerer). Pull-up: Ruhe-Kontakt liegt an, abgehoben = high.
    # Bei anderer Verdrahtung HOOK_ACTIVE_HIGH auf False setzen.
    hook_pin = Pin(HOOK_PIN, Pin.IN, Pin.PULL_UP)
    hook_pin.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=hook_interrupt)

    # Startzustand einlesen
    hook_active = is_hook_active_now()

    # DIN2: 2. Digital-Input, gleiche Logik wie Hook
    din2_pin = Pin(DIN2_PIN, Pin.IN, Pin.PULL_UP)
    din2_pin.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=din2_interrupt)
    din2_active = is_din2_active_now()

    # AIN1: Helligkeits-Poti (GP27 -> ADC1, machine.ADC mappt automatisch)
    poti_adc = ADC(POTI_PIN)
    bright_percent = read_poti_percent()  # Startwert einmal ohne Hysterese lesen

    # AIN2: 2. Analog-Input (GP26 -> ADC0, machine.ADC mappt automatisch)
    ain2_adc = ADC(AIN2_PIN)
    ain2_percent = read_ain2_percent()  # Startwert einmal ohne Hysterese lesen

    # Interner Temperatursensor (RP2350: ADC-Kanal 4)
    temp_adc = ADC(TEMP_ADC_ID)

# --- AIN2 (2. Analog-Input) ----------------------------------------

def read_ain2_percent():
    # AIN2 an GP26 auslesen, 0-65535 -> 0-100 %
    return ain2_adc.read_u16() * 100 / 65535

def update_ain2_percent():
    # Hysterese wie beim Poti: ADC-Rauschen soll nicht zapplen.
    global ain2_percent
    measured = read_ain2_percent()
    if abs(measured - ain2_percent) > AIN2_HYSTERESIS_PERCENT:
        ain2_percent = measured

# --- AIN1: Helligkeits-Poti -----------------------------------------------

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
    # Pinzustand lesen, mit hook_active_high umkehrbar (Laufzeit-Polaritaet,
    # per UART umschaltbar und im Flash gespeichert; HOOK_ACTIVE_HIGH ist
    # nur noch der Code-Default).
    raw = hook_pin.value()
    if hook_active_high:
        return raw == 1
    return raw == 0

# --- DIN2: 2. Digital-Input ---------------------------------------

def din2_interrupt(pin):
    # Nur Flag + Zeitpunkt merken, alles andere macht der Loop.
    global din2_event_flag, din2_event_time
    din2_event_flag = True
    din2_event_time = time.ticks_ms()

def is_din2_active_now():
    # Pinzustand lesen, mit DIN2_ACTIVE_HIGH umkehrbar
    raw = din2_pin.value()
    if DIN2_ACTIVE_HIGH:
        return raw == 1
    return raw == 0

def handle_din2_event():
    # Wird im Loop aufgerufen, wenn der Interrupt eine Flanke gemeldet hat.
    # Debounce wie beim Hook. Protokoll-Schluessel bleibt "debug" (GP3 war
    # der Debug-Input; die Pi-Haupt-App v2 kennt diesen Event-Namen).
    global din2_event_flag, din2_active

    if time.ticks_diff(time.ticks_ms(), din2_event_time) < DIN2_DEBOUNCE_MS:
        return  # Debounce noch nicht um, Flag bleibt gesetzt

    din2_event_flag = False

    new_active = is_din2_active_now()
    if new_active == din2_active:
        return  # nur Prellen, keine echte Aenderung

    din2_active = new_active
    send_json({"event": "debug", "state": "active" if din2_active else "inactive"})

# --- Startphase: warten auf den Pi4 ------------------------------

def wait_for_host():
    global host_connected

    # LEDs atmen langsam (Werte siehe Konfiguration oben),
    # bis die erste Nachricht vom Pi4 kommt.

    # Test: Lebenszeichen an den Pi senden (prueft TX-Richtung)
    # hook_active_high im Boot-Event: alte Pi-Apps ignorieren den
    # zusaetzlichen Key, neue Apps koennen die Polaritaet beim Boot loggen.
    send_json({"event": "boot", "hook_active_high": hook_active_high})
    set_status_led(STATUS_YELLOW)  # Status: wartet auf Host

    BREATH_PERIOD_MS = int(1000 / BREATH_FREQ_HZ)
    BREATH_MID = (BREATH_MAX_PERCENT + BREATH_MIN_PERCENT) / 2
    BREATH_AMP = (BREATH_MAX_PERCENT - BREATH_MIN_PERCENT) / 2

    last_heartbeat = time.ticks_ms()

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
        update_status_led()  # Soft-Fehler-Restore (waere in dieser Phase ohnehin inaktiv)
        msg = read_message()
        if msg is not None:
            host_connected = True
            set_status_led(STATUS_GREEN)  # Status: aktiv (Host verbunden)
            handle_message(msg)  # direkt beantworten (z.B. get_telemetry)
            apply_hook_leds()    # LEDs aus der Atem-Phase in Hook-Grundstellung
            break

        time.sleep_ms(20)

        # Heartbeat: sichtbare Lebenszeichen, sobald die REPL verbunden ist
        # (Prints vor dem Verbinden werden vom USB-Stack verworfen).
        if DEBUG and time.ticks_diff(time.ticks_ms(), last_heartbeat) >= 1000:
            last_heartbeat = time.ticks_ms()
            print("wait_for_host: warte auf Pi4 ...")

# --- UART Nachrichten --------------------------------------------

def read_message():
    # Liest alle neuen Zeichen vom UART und sucht nach einer Zeile.
    # Gibt geparstes JSON zurueck oder None, wenn keine (gueltige) Zeile da ist.
    global uart_buffer

    n = uart.any()
    if n == 0:
        return None

    try:
        data = uart.read(n).decode()
    except ValueError:
        # Binaermuell auf der Leitung (z.B. waehrend der Pi4 bootet):
        # Puffer verwerfen, statt die Firmware abzustuerzen
        uart_buffer = ""
        signal_soft_error()  # nicht dekodierbare Nachricht: rot zeigen
        if DEBUG:
            print("UART: Dekodierfehler, Puffer verworfen")
        return None

    if DEBUG:
        print("UART roh empfangen:", repr(data))

    uart_buffer += data

    # Schutz: Muell ohne Zeilenende darf den Puffer nicht endlos wachsen lassen
    if len(uart_buffer) > 512:
        uart_buffer = ""
        signal_soft_error()  # unparsebare Nachricht: rot zeigen
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
        parsed = json.loads(line)
    except ValueError:
        signal_soft_error()  # falsche Nachricht (ungueltiges JSON): rot zeigen
        if DEBUG:
            print("UART: ungueltiges JSON:", line)
        return None

    if not isinstance(parsed, dict):
        # Gueltiges JSON, aber kein Objekt (z. B. Array): genauso eine
        # falsche Nachricht — rot zeigen und verwerfen, nicht crashen.
        signal_soft_error()
        if DEBUG:
            print("UART: Nachricht ist kein JSON-Objekt:", line)
        return None

    # Gueltige Nachricht: kurzer blauer Blitz (Diagnostik).
    flash_rx_ok()
    return parsed

def send_json(obj):
    # uart.write() darf laut MicroPython weniger Bytes schreiben als
    # uebergeben (Rueckgabewert = Anzahl geschriebener Bytes). Eine
    # einfache write()-Zeile hat deshalb hier schon still Bytes verloren
    # (Pi4 erhielt truncierte JSONs). Diese Schleife schickt den Rest
    # nach, bis der komplette Payload draussen ist. Der Timeout
    # verhindert, dass die Firmware bei blockiertem UART stecken bleibt.
    # gc.collect() raeumt den String-Heap danach auf.
    flash_blue()  # TX: kurzer blauer Blitz (Nachricht geht raus)
    payload = json.dumps(obj).encode() + b"\n"
    deadline = time.ticks_add(time.ticks_ms(), SEND_TIMEOUT_MS)
    while payload:
        n = uart.write(payload)
        if n:
            payload = payload[n:]
        elif time.ticks_diff(deadline, time.ticks_ms()) <= 0:
            # UART haengt dauerhaft (z.B. Hardware-Problem): abbrechen,
            # damit die Firmware nicht in dieser Schleife stecken bleibt
            set_status_led(STATUS_RED)  # Status: Fehler (UART blockiert)
            if DEBUG:
                print("UART: Senden blockiert, Rest verworfen")
            break
        else:
            # TX-FIFO voll: kurz warten und den Rest nachschicken
            time.sleep_us(100)
    gc.collect()

def handle_message(msg):
    cmd = msg.get("cmd", "")

    if cmd == "set_led":
        handle_set_led(msg)
    elif cmd == "get_telemetry":
        send_json(get_telemetry_response())
    elif cmd == "set_hook_polarity":
        handle_set_hook_polarity(msg)
    elif cmd == "get_hook_polarity":
        send_json({"status": "ok", "cmd": "get_hook_polarity",
                   "data": {"hook_active_high": hook_active_high}})
    else:
        # Noch nicht implementierte Befehle: Error antworten + kurz gelb
        signal_soft_error()
        send_json({"status": "error", "cmd": cmd, "data": {"error": "unknown command"}})

def handle_set_led(msg):
    fade_ms = msg.get("fade", FADE_DEFAULT_MS)  # optional, Default aus Config
    result = {}

    for name in ("main", "logo"):
        if name not in msg:
            continue
        percent, follow_poti = resolve_led_value(msg[name])
        if percent is None:
            signal_soft_error()  # nicht-kritisch: kurz rot, dann wieder gruen
            send_json({"status": "error", "cmd": "set_led",
                       "data": {"error": "invalid value for '" + name + "'"}})
            return
        start_fade(name, percent, fade_ms, follow_poti)
        result[name] = percent

    send_json({"status": "ok", "cmd": "set_led", "data": result})

# --- DIN1: Hook (Hoerer) -------------------------------------------

def hook_interrupt(pin):
    # Nur Flag + Zeitpunkt merken, alles andere macht der Loop.
    global hook_event_flag, hook_event_time
    hook_event_flag = True
    hook_event_time = time.ticks_ms()

def handle_hook_event():
    # Wird im Loop aufgerufen, wenn der Interrupt eine Flanke gemeldet hat.
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

# --- Hook-Polaritaet (per UART umschaltbar, im Flash gespeichert) ---

def load_hook_polarity():
    # Gespeicherte Polaritaet beim Boot laden. Datei fehlt (erste Box)
    # oder kaputtes JSON -> still Code-Default behalten, nie crashen.
    # Nur echte bool-Werte akzeptieren, alles andere ignorieren.
    global hook_active_high
    try:
        with open(HOOK_CONFIG_FILE) as f:
            cfg = json.loads(f.read())
        if isinstance(cfg.get("hook_active_high"), bool):
            hook_active_high = cfg["hook_active_high"]
    except OSError:
        pass            # Datei fehlt -> Code-Default (normal bei erster Box)
    except ValueError:
        pass            # kaputtes JSON -> Code-Default

def save_hook_polarity():
    # Polaritaet dauerhaft speichern. Atomar schreiben (erst .tmp, dann
    # rename), damit ein Stromausfall waehrend des Schreibens keine halbe
    # Datei hinterlaesst. Fehler (Flash voll, FS readonly) nur loggen,
    # nie crashen - die FW muss auch bei Speicherproblemen weiterlaufen.
    try:
        tmp = HOOK_CONFIG_FILE + ".tmp"
        with open(tmp, "w") as f:
            f.write(json.dumps({"hook_active_high": hook_active_high}))
        os.rename(tmp, HOOK_CONFIG_FILE)
    except OSError:
        if DEBUG:
            print("Hook-Polaritaet konnte nicht gespeichert werden")

def handle_set_hook_polarity(msg):
    # Setzt die Hook-Polaritaet dauerhaft. Reihenfolge bewusst (Spec):
    # Polaritaet setzen -> speichern -> Zustand neu lesen -> ok-Antwort
    # -> hook-Event -> apply_hook_leds().
    # WICHTIG: Alle drei Variablen ins global - fehlt hook_event_flag,
    # legt MicroPython implizit eine lokale Variable an und der
    # Debounce-Zustand bleibt stehen.
    global hook_active, hook_active_high, hook_event_flag

    value = msg.get("active_high")
    if not isinstance(value, bool):
        send_json({"status": "error", "cmd": "set_hook_polarity",
                   "data": {"error": "active_high muss bool sein"}})
        return

    hook_active_high = value
    save_hook_polarity()

    hook_event_flag = False            # alte Flanke ist ungueltig
    hook_active = is_hook_active_now() # Zustand mit neuer Polaritaet
    send_json({"status": "ok", "cmd": "set_hook_polarity",
               "data": {"hook_active_high": hook_active_high,
                        "hook_active": hook_active}})
    send_json({"event": "hook", "state": "active" if hook_active else "inactive"})
    apply_hook_leds()

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
    # Konfiguration + Kalibrierung schreiben.
    # Tolerant: Sensor nicht da -> nur Debug-Meldung, kein Crash,
    # damit das Board immer bootet und das Boot-Event rausgeht.
    try:
        i2c_write_reg(addr, INA_REG_CONFIG, INA_CONFIG)
        i2c_write_reg(addr, INA_REG_CAL, INA_CALIBRATION)
    except OSError:
        if DEBUG:
            print("INA219", hex(addr), "nicht erreichbar")

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
    # Interner Temperatursensor des RP2350 (ADC-Kanal 4).
    # Formel laut RP2350-Datenblatt (Vbe 0.706 V bei 27 C, -1.721 mV/C),
    # Referenz 3.3 V. Falls eine MicroPython-Version anders skaliert,
    # hier nachjustieren (Smoke-Test Punkt 9).
    voltage = temp_adc.read_u16() * 3.3 / 65535
    return 27 - (voltage - 0.706) / 0.001721

# --- Telemetrie ---------------------------------------------------

def get_telemetry_response():
    # Liest die 4 Power Rails, Akku, Temperaturen, Poti-Wert und
    # Digital-/Analog-Input-Zustaende und baut die Antwort-Nachricht.
    raw_v, raw_c, raw_p = read_ina219(INA_ADDR_RAW)
    v12_v, v12_c, v12_p = read_ina219(INA_ADDR_12V)
    v5_v, v5_c, v5_p = read_ina219(INA_ADDR_5V)
    bat_v, bat_c, bat_p = read_ina219(INA_ADDR_BAT)

    # Das INA219-Power-Register meldet staets einen positiven Wert,
    # egal in welche Richtung der Strom fliesst (anders als beim
    # Current-Register gibt es dort keine Richtungsinformation).
    # Das Vorzeichen wird daher hier aus dem Strom uebernommen:
    # c < 0 (Laden) -> p auch negativ, sonst wuerde die Pi-App beim
    # Laden positive Leistung anzeigen, obwohl Strom zurueck in den
    # Akku fliesst.
    if bat_c < 0:
        bat_p = -bat_p

    return {
        "status": "ok",
        "cmd": "get_telemetry",
        "firmware": FIRMWARE_VERSION,
        "data": {
            "rails": {
                # Akku: c < 0 = Laden, c > 0 = Entladen (Shunt-Richtung am
                # BAT-INA, 0x45). p kann beim Laden negativ sein.
                "bat": {"v": round(bat_v, 2), "c": round(bat_c), "p": round(bat_p)},
                "raw": {"v": round(raw_v, 2), "c": round(raw_c), "p": round(raw_p)},
                "12v": {"v": round(v12_v, 2), "c": round(v12_c), "p": round(v12_p)},
                "5v1": {"v": round(v5_v, 2), "c": round(v5_c), "p": round(v5_p)},
            },
            # Ladezustand (%) aus der BAT-LUT — unveraendert wie vorher (RAW).
            "bat": get_battery_percent(raw_v),
            # True, solange Strom in den Akku zurueckfliesst (bat.c < 0).
            # Additiv; bestaehende Pi-Apps koennen den Key ignorieren.
            "bat_charging": bat_c < 0,
            "temp": {
                "case": 0, # nicht unterstuetzt (Rev 2.1), konstant 0
                "pico": round(read_cpu_temp(), 1)
            },
            # LEDs in Prozent (0-100, mit 1 Nachkommastelle) — matches Pi-App
            "leds": {
                "main": round(led_state["main"]["current"], 1),
                "logo": round(led_state["logo"]["current"], 1),
            },
            "analog_in":
                {"brightness": round(bright_percent, 1),
                 "ain2": round(ain2_percent, 1)
            },
            "digital_in":
                {"hook": 1 if hook_active else 0,
                 "debug": 1 if din2_active else 0  # DIN2/GP3, Schluessel bleibt "debug"
            },
        },
    }

# --- Setup und Loop ------------------------------------------------

def setup():
    global hook_active

    set_status_led(STATUS_YELLOW)  # Status: Startup / wartet auf Host
    init_hardware()

    # Gespeicherte Hook-Polaritaet laden (Datei fehlt/kaputt -> Default).
    # Danach den Startzustand mit der geladenen Polaritaet neu einlesen,
    # weil init_hardware() oben noch mit dem Code-Default ausgewertet hat.
    load_hook_polarity()
    hook_active = is_hook_active_now()

    print(get_telemetry_response())  # einmal lesen, damit die INA219 Sensoren initialisiert werden

def loop():
    msg = read_message()
    if msg is not None:
        handle_message(msg)

    update_status_led()  # Flash/Soft-Fehler zurueck auf normalen Status
    update_bright_percent()
    update_ain2_percent()
    update_led_fades()

    if hook_event_flag:
        handle_hook_event()

    if din2_event_flag:
        handle_din2_event()

# --- Programmstart ---------------------------------------------------

try:
    setup()
    wait_for_host()

    while True:
        loop()
except Exception as e:
    # Crash-Handler: Fehler ueber USB sichtbar machen, statt still zu
    # sterben (sonst sieht der Pi nur "UART kommt nicht zurueck").
    set_status_led(STATUS_RED)  # Status: Fehler
    import sys
    sys.print_exception(e)
    print("FIRMWARE-CRASH im Setup/Loop — oben stehenden Traceback pruefen.")
    while True:
        time.sleep_ms(1000)