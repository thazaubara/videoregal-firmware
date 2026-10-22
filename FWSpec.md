## FW Spec (Oktober 22, 2026, Rev 5)

Die Firmware läuft auf einem **Waveshare RP2350-Zero** (RP2350A), der auf das **Driver Board Rev 2.1** aufgelötet ist. Sie ist in MicroPython geschrieben und kommuniziert über UART (JSON, 115200 Baud) mit einem Raspberry Pi 4, der die übergeordnete Logik steuert.

**Diese Firmware und diese Hardware gelten für ALLE Boxen** (RP2350-Zero + Driver Board 2.1 + offizielles RPI_PICO2-MicroPython, siehe „Flashing je Box"). Es gibt keine Board-Revisions-Weiche mehr.

Sie bedient die Elektronik der Videoregal-Installation: ein Wählscheibentelefon, bei dem das Abheben des Hörers als Eingabe-Event dient, dazu steuerbare LEDs rundherum und ein eigenes, separat ansteuerbares LED-Schild. 

Kamera und Display gehören nicht zum RP2350-Zero-System. Die Videoaufnahme übernimmt der Pi 4, während der RP2350-Zero ausschließlich die Echtzeit-Elektronik kontrolliert. Er sendet beim Abheben des Hörers ein Event zum Pi, der dann die Videoaufnahme starten soll. 

**Hardware / Pin-Map (Driver Board Rev 2.1)**

Quelle: `pcb_ref/Schematic_2609 Videoregal Driver Board 2.1_*.pdf`, Layout-Screenshots und RP2350-Zero-Pinout in `pcb_ref/`.

| Signal | GPIO | Anmerkung |
| --- | --- | --- |
| UART TX (zum Pi4) | GP0 | 115200 Baud |
| UART RX (vom Pi4) | GP1 | 115200 Baud |
| DIN1 (Hook / Hörer) | GP2 | Digital In, Pull-up, IRQ |
| DIN2 (Debug) | GP3 | Digital In, Pull-up, IRQ |
| I2C SDA | GP4 | INA219, 400 kHz |
| I2C SCL | GP5 | INA219, 400 kHz |
| GP6–GP13 | — | auf dem PCB nicht verwendet |
| LOGO LED | GP14 | PWM, 40 kHz |
| MAIN LED | GP15 | PWM, 40 kHz |
| Status-LED | GP16 | Onboard-WS2812 des RP2350-Zero |
| AIN2 | GP26 | ADC0 |
| BRIGHTNESS POTI (AIN1) | GP27 | ADC1 |
| GP28, GP29 | — | auf dem PCB nicht verwendet |

**Wichtig zu GP16:** Er wird auf dem PCB **nicht herausgeführt**. Beim RP2350-Zero ist GP16 im Modul fest mit der Onboard-WS2812 (DIN) verbunden (Waveshare-Pinout: „WS2812 RGB LED used pin: GP16 | DIN", Demo: `ws2812_program_init(..., 16, ...)`). Der Pin gehört daher allein der Status-LED — es gibt auf dem 2.1-Board keinen Konflikt mit MAIN/LOGO.

**Hinweis PWM-Slices:** GP15 (Slice 7/Ch1) und GP14 (Slice 7/Ch0) liegen auf demselben Slice, aber auf verschiedenen Channels. Duty-Cycles sind unabhängig, die Frequenz ist für beide identisch (40 kHz).

**Geräte**

- UART
    - 115200 Baud
    - Pico: GP0 TX, GP1 RX
    - Pi4: GPIO14 TX, GPIO15 RX
- I2C
    - SDA Pin: GP4, SCL Pin GP5
- INA219
    - RAW Power Rail Monitoring (Battery Out)
    - Shunt 0.01 Ohm, Max 4A
    - I2C Address: `0x40`
- INA219
    - 12V Power Rail (DC Converter Out)
    - Shunt 0.01 Ohm, Max 4A
    - I2C Address: `0x41`
- INA219
    - 5V Power Rail Monitoring (DC Converter Out)
    - Shunt 0.01 Ohm, Max 4A
    - I2C Address: `0x44`
- MAIN LED
    - PWM Pin GP15
    - f=40 kHz
- LOGO LED
    - PWM Pin GP14
    - f=40 kHz
- STATUS LED (Onboard-WS2812 des RP2350-Zero)
    - Datenpin GP16 (auf dem PCB nicht herausgeführt)
    - Helligkeit fix 10 % (STATUS_BRIGHT = 26), unabhängig vom Poti
    - Startup/Wartemodus: gelb, Fehler: rot, ready (Host verbunden): grün
    - Nachricht rein ODER raus: kurzer blauer Blitz (300 ms)
- HOOK INPUT (DIN1)
    - Digital In Pin GP2
- DEBUG INPUT (DIN2)
    - Digital In Pin GP3
- BRIGHTNESS POTI (AIN1)
    - Analog In Pin GP27
- AIN2
    - Analog In Pin GP26

**UART communication to host**

- Turn on/off LEDs
    - Pi4 → Pico:
      
        ```json
        {"cmd": "set_led", "main": 10, "logo": 10}\n
        
        {"cmd": "set_led", "main": "bright", "logo": "dim", "fade": 1000}\n
        ```
        
        - fade als optionaler parameter.
        - 2 leds in einer message steuern.
        - keywords wie “bright” und “dim” für werte die der Pico intern kennt
        (poti auslesen oder so)
    - Pico → Pi4:
      
        ```json
        {
        "status": "ok", 
        "cmd": "set_led",
        "data": {
        	"main": 80,
        	"logo": 20
        	}
        }\n
        ```
        
    
- Get Telemetry Info (Einheiten: v = Volt, c = mA, p = mW)
    - Pi4 → Pico:
      
        ```json
        {"cmd": "get_telemetry"}\n
        ```
        
    - Pico → Pi4:
      
        ```json
        {
        "status": "ok",
        "cmd": "get_telemetry",
        "data": {
        	"rails": {
        		"bat": {"v": 14.80, "c": -420, "p": -6210},
        		"raw": {"v": 14.80, "c": 82, "p": 1200},
        		"12v": {"v": 12.12, "c": 60, "p": 727},
        		"5v1": {"v": 5.09, "c": 16, "p": 93}
        		},
        	"bat": 80,
        	"bat_charging": true,
        	"temp": {"case": null, "cpu": 41.2},
        	"poti": {"bright": 80.0},
	"leds": {"main": 80.0, "logo": 5.0},
        	"digital_in": {"hook": 1, "debug": 0}
        	}
        }\n
        ```
    
      - Vorzeichen an bat (BAT-Rail, INA 0x45): c < 0 = Laden,
        c > 0 = Entladen. Beim Laden ist auch p negativ (das INA219
        meldet im Power-Register immer positive Werte; die Firmware
        uebernimmt das Vorzeichen vom Strom).
      - bat_charging ist true, solange geladen wird (c < 0),
        additiv; aeltere Pi-Apps ignorieren den Key.
- Set Hook Polarity (dauerhaft gespeichert)
    - Pi4 -> Pico:

        ```json
        {"cmd": "set_hook_polarity", "active_high": false}\n
        ```

    - Pico -> Pi4:

        ```json
        {"status": "ok", "cmd": "set_hook_polarity", "data": {"hook_active_high": false, "hook_active": true}}\n
        ```

        - active_high muss ein echtes JSON-boolean sein (true/false),
          sonst Error-Antwort und KEINE Aenderung/kein Speichern.
        - Direkt danach sendet der Pico ein normales hook-Event mit dem
          neu ausgewerteten Zustand und setzt die LED-Grundstellung
          (apply_hook_leds). Das ist gewollt: Der Pi braucht den
          aktuellen Zustand nach jedem Set-Vorgang.
        - Der Wert wird im Pico-Flash dauerhaft gespeichert
          (/hook_config.json) und ueberlebt Reboot und Stromausfall.

- Get Hook Polarity
    - Pi4 -> Pico:

        ```json
        {"cmd": "get_hook_polarity"}\n
        ```

    - Pico -> Pi4:

        ```json
        {"status": "ok", "cmd": "get_hook_polarity", "data": {"hook_active_high": true}}\n
        ```

- Push Message (Hook)
  
    ```json
    {"event": "hook", "state": "active"}\n
    ```
    
    - Asynchrones Event!

- Push Message (Debug Input)

    ```json
    {"event": "debug", "state": "active"}\
    ```

    - Asynchrones Event!

- Push Message (Boot)

    ```json
    {"event": "boot", "hook_active_high": true}\n
    ```
    
    - Wird einmal beim Start des Pico gesendet, bevor auf den Pi4 gewartet wird.
      Der Pi4 kann das als "Pico ist bereit"-Signal nutzen.
      hook_active_high = aktuell gespeicherte Hook-Polaritaet (additiv; alte
      Pi-Apps ignorieren den zusaetzlichen Key).

**Funktionen**

- Power Monitoring:
    - Wenn Nachricht kommt, lies sensoren aus und antworte.
    - temp enthaelt mehrere Werte: cpu = CPU-Temperatur des Pico,
      case = Gehaeusetemperatur (wird noch implementiert, aktuell null).
    - Telemetry enthaelt zusaetzlich den aktuellen Poti-Wert ("bright"),
      die aktuellen LED-Helligkeiten in Prozent ("leds", main/logo,
      inkl. laufender Fades) und die Zustaende der Digital-Inputs (hook, debug,
      als 1/0 statt true/false, fuer den Mimir Exporter).
- Hook recording enable
    - Hook active: Hörer abgehoben. Hook Inactive: hörer aufgelegt.
    - Wenn Hook Flanke sich ändert, dann schick die Async Message an den Pi4.
    - Interrupt & Debounce! Debounce-Zeit: 30 ms.
    - Möglichkeit für einfache Umkehr der Logik in der Firmware (high vs low active)
    - Hook-Polaritaet per UART umschaltbar (set_hook_polarity) und dauerhaft
      im Pico-Flash gespeichert (/hook_config.json). HOOK_ACTIVE_HIGH im Code
      ist nur noch der Default (erste Box / keine Config-Datei). DIN2 bleibt
      unberuehrt (DIN2_ACTIVE_HIGH, kein Command).
    - Wenn sich flanke ändert, dann setze die leds gleich auf den passenden wert.
        - Hook active: Main LED “bright”, Logo LED “dim”
        - Hook inactive: Main LED “dim”, Logo LED “bright”
- Debug Input (GP3):
    - Gleiche Flanken-/Debounce-Logik wie Hook (30 ms, Logik umkehrbar).
    - Sendet bei Flankenwechsel das async Event "debug".
- LED switching
    - PWM Frequenz fix. Helligkeit über Duty Cycle.
    - Interne Keywords "bright" und "dim". für jede LED.
    Initial: bright: über Poti auslesbar, das Poti regelt im Bereich
    dim bis 100% ("bright" ist nie dunkler als "dim"). dim: 5%.
    - Standardmäßig fade von 700ms, optionaler parameter.
    - Poti auslesen gibt Rauschen. Hysterese von 5% einbauen.
        - LEDs, die auf "bright" gesetzt wurden, folgen dem Poti live
          mit kurzem Fade (100ms). LEDs mit festem Wert oder "dim"
          folgen dem Poti nicht.
- Battery Percentage Track in Firmware based on voltage.
    - Lineare LUT in 10%-Schritten. Ankerpunkte: 16.3 V = 100%,
      12.8 V = 20% (kritisch), 12.0 V = 10%, 11.2 V = 0% (Abschaltung).
      Dazwischen linear, Schwellen auf 0.05 V gerundet.
      Rückgabewert ist die Bracket-Untergrenze (0, 10, 20, ... 100).

**FW Architecture & Mode of Operations**

- Superloop. Arduino Like mit Setup und Loop.
    - Configuration values on top of code. (c++ define style)
    - Hardware Functions and Classes below.
    - Setup:
        - Init Hardware
        - Initial State of Leds: breathing from 1-10%. 
        Indicates waiting for Pi4 startup & connection.
        Use extra non blocking loop for that before entering main Loop.
        - If first message from Pi4 is received: Activate loop, set LEDs to initial Condition (based on hook). Der erste Fade startet
        dabei von der aktuellen Atem-Helligkeit (kein Sprung). 
        Indicates normal program flow.
    - Loop:
        - Get Message from Uart
        - React to Message based on “Funktionen”
        - If Interrupt flag set: react to that.
- FW soll komplett basic bleiben. ich weiß man kann viel machen mit der hardware, aber das brauche ich jetzt noch nicht. upgrades gehen ja immer. nur die funktionen umsetzen die für den aktuellen betrieb notwendig sind.
- FW soll so einfach geschrieben sein wie möglich. Keine Advanced Python Features, ich muss das in 2 Jahren auch noch lesen können.
## Smoke-Test Hook-Polaritaet (per UART umschaltbar, dauerhaft gespeichert)

1. get_hook_polarity -> hook_active_high: true (Default, keine Datei).
2. Hoerer abheben -> hook-Event active; auflegen -> inactive (wie bisher).
3. set_hook_polarity {"active_high": false} -> ok-Antwort, dann sofort
   hook-Event mit INVERTIERTEM Zustand; LED-Grundstellung wechselt.
4. get_telemetry -> digital_in.hook zeigt invertierten Wert.
5. Nochmal abheben/auflegen -> Events weiterhin konsistent (invertiert).
6. Pico stromlos machen, wieder an -> get_hook_polarity -> immer noch false.
   (Der eigentliche Punkt des Features.)
7. Fehlerfall: set_hook_polarity {"active_high": "nein"} (String) ->
   Error-Antwort, Zustand unveraendert, nichts gespeichert.
8. Datei loeschen (os.remove('/hook_config.json') ueber REPL) -> nach
   Reboot wieder Default true.

## Flashing je Box (Standard-Workflow)

Gilt identisch fuer **alle** Boxen: Waveshare RP2350-Zero + Driver Board Rev 2.1.

1. **BOOTSEL** halten und USB anstecken -> Laufwerk erscheint.
2. `flash_nuke.uf2` ins Laufwerk ziehen -> Laufwerk entfernt sich selbst.
3. **BOOTSEL** halten und USB anstecken -> Laufwerk erscheint.
4. `RPI_PICO2-20260824-v1.29.0.uf2` ins Laufwerk ziehen -> Laufwerk entfernt sich selbst.
5. In PyCharm (MicroPico-Plugin) die aktuelle `main.py` hochladen, dann Reset
   (Soft-Reboot).
6. Im REPL pruefen: Boot-Banner `=== main.py Boot === Firmware 2.1.5` erscheint,
   und die Status-LED leuchtet **gelb**, weil der Pi4 noch nicht verbunden ist.

**Dateien und Bezugsquellen**

| Datei | Quelle | Anmerkung |
| --- | --- | --- |
| `flash_nuke.uf2` | https://datasheets.raspberrypi.com/soft/flash_nuke.uf2 | loescht den kompletten Flash (114.688 Bytes) |
| `RPI_PICO2-20260824-v1.29.0.uf2` | https://micropython.org/resources/firmware/RPI_PICO2-20260824-v1.29.0.uf2 (Download-Seite: https://micropython.org/download/RPI_PICO2/) | offizieller Pico-2-Build, `MicroPython v1.29.0 on 2026-08-24`, 661.504 Bytes, SHA256 `6bfc5d110f293870f60150468dfeec8aec5323b57415a0b9a86e6c334e876e93` |

- **Der Flash-Nuke-Schritt ist Pflicht**, wenn vorher ein anders konfigurierter
  Build lief (z. B. der Waveshare-16MB-Build): das LittleFS-Layout passt sonst
  nicht und MicroPython meldet `MicroPython filesystem scan failed … OSError: 84`.
- **Nicht** die Variante „Firmware (RISC-V CPU mode)" derselben Download-Seite
  verwenden.
- Der alte Waveshare-Preview-Build
  `WAVESHARE-RP2350A-20260609-v1.29.0-16MB.uf2` **wird nicht mehr verwendet**:
  Preview-/„dirty"-Build aus Waveshares RP2350A-Library, mit dem die
  Onboard-WS2812 auf GP16 dauerhaft weiss blieb und teils das Dateisystem
  kaputtging.

**Kontrolle nach dem Flashen (REPL)**

```python
import os
print(os.uname())      # erwartet: 'Raspberry Pi Pico2 with RP2350', release '1.29.0'
```

**Status-LED-Verhalten zum Pruefen** (Onboard-WS2812 auf GP16, Helligkeit fix 10 %):

| Zustand | Farbe |
| --- | --- |
| Startup / wartet auf Pi4 | gelb |
| Fehler (ungueltige Nachricht, UART blockiert, Crash) | rot |
| ready / UART-Loop (Pi4 verbunden) | gruen |
| Nachricht rein ODER raus | blauer Kurzblitz (300 ms), danach zurueck auf Status |
