## FW Spec (September 15, 2026, Rev 4)

Die Firmware läuft auf einem Raspberry Pi Pico (RP2040), ist in MicroPython geschrieben und kommuniziert über UART (JSON, 115200 Baud) mit einem Raspberry Pi 4, der die übergeordnete Logik steuert. 

Sie bedient die Elektronik der Videoregal-Installation: ein Wählscheibentelefon, bei dem das Abheben des Hörers als Eingabe-Event dient, dazu steuerbare LEDs rundherum und ein eigenes, separat ansteuerbares LED-Schild. 

Kamera und Display gehören nicht zum Raspberry Pi Pico System. Die Videoaufnahme übernimmt der Pi 4, während der Pico ausschließlich die Echtzeit-Elektronik kontrolliert. Der Pico sendet beim abheben des Hörers ein Event zum Pi, der dann die Videoaufnahme starten soll. 

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
- HOOK INPUT
    - Digital In Pin GP2
- DEBUG INPUT
    - Digital In Pin GP3
- BRIGHTNESS POTI
    - Analog In Pin GP27

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
        		"raw": {"v": 14.80, "c": 82, "p": 1200},
        		"12v": {"v": 12.12, "c": 60, "p": 727},
        		"5v1": {"v": 5.09, "c": 16, "p": 93}
        		},
        	"bat": 80,
        	"temp": {"case": null, "cpu": 41.2},
        	"poti": {"bright": 80.0},
        	"digital_in": {"hook": true, "debug": false}
        	}
        }\n
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
    {"event": "boot"}\n
    ```
    
    - Wird einmal beim Start des Pico gesendet, bevor auf den Pi4 gewartet wird.
      Der Pi4 kann das als "Pico ist bereit"-Signal nutzen.

**Funktionen**

- Power Monitoring:
    - Wenn Nachricht kommt, lies sensoren aus und antworte.
    - temp enthaelt mehrere Werte: cpu = CPU-Temperatur des Pico,
      case = Gehaeusetemperatur (wird noch implementiert, aktuell null).
    - Telemetry enthaelt zusaetzlich den aktuellen Poti-Wert ("bright")
      und die Zustaende der Digital-Inputs (hook, debug).
- Hook recording enable
    - Hook active: Hörer abgehoben. Hook Inactive: hörer aufgelegt.
    - Wenn Hook Flanke sich ändert, dann schick die Async Message an den Pi4.
    - Interrupt & Debounce! Debounce-Zeit: 30 ms.
    - Möglichkeit für einfache Umkehr der Logik in der Firmware (high vs low active)
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