FASTLANE ONE-WAY ENTRANCE RFID GATE - MODULAR BUILD 1.3.0
ESP32-S3 + RDM6300 + WS2812B + DS3231 + MicroSD + W5500 + TB6600 + 2x NEMA17 + SOLENOID RELAY + BUZZER

============================================================
PROJECT STRUCTURE
============================================================

/boot.py
/config.py
/main.py
/webui.html
/components/__init__.py
/components/tb6600.py
/components/w5500.py
/components/ds3231.py
/components/sdcard.py
/components/rdm6300.py
/components/ws2812b.py
/components/rfid.py
/components/solenoid.py
/components/buzzer.py

main.py = application/orchestration only.
Hardware/component code is separated under /components so each device can be
updated independently later.

============================================================
TB6600 + TWO NEMA17
============================================================

ESP32-S3 5V -> TB6600 PUL+
ESP32-S3 GPIO1 -> TB6600 PUL-
ESP32-S3 5V -> TB6600 DIR+
ESP32-S3 GPIO2 -> TB6600 DIR-
TB6600 ENA+ -> NOT CONNECTED
TB6600 ENA- -> NOT CONNECTED

TB6600 SW1 -> OFF
TB6600 SW2 -> ON
TB6600 SW3 -> OFF
TB6600 microstep -> 1/8
TB6600 pulses/revolution -> 1600

TB6600 SW4 -> ON
TB6600 SW5 -> OFF
TB6600 SW6 -> OFF
TB6600 current setting -> 2.0 A

TB6600 A+ -> Motor 1 Wire 4 BLACK
TB6600 A- -> Motor 1 Wire 2 GREEN
TB6600 B+ -> Motor 1 Wire 3 RED
TB6600 B- -> Motor 1 Wire 1 BLUE

TB6600 A+ -> Motor 2 Wire 2 GREEN
TB6600 A- -> Motor 2 Wire 4 BLACK
TB6600 B+ -> Motor 2 Wire 3 RED
TB6600 B- -> Motor 2 Wire 1 BLUE

Motor 2 has one coil pair reversed so it rotates opposite Motor 1 while both
motors receive the same STEP/DIR signal from the single TB6600.

TB6600 V+ -> 12V PSU +
TB6600 V- -> 12V PSU GND

============================================================
ESP32 GPIO -> COMPONENT WIRING
============================================================

ESP32 GPIO1 -> TB6600 PUL- / STEP
ESP32 GPIO2 -> TB6600 DIR-
ESP32 GPIO4 -> W5500 SCK
ESP32 GPIO5 -> W5500 MOSI
ESP32 GPIO6 -> W5500 MISO
ESP32 GPIO7 -> W5500 CS
ESP32 GPIO8 -> DS3231 SDA
ESP32 GPIO9 -> DS3231 SCL
ESP32 GPIO10 -> MicroSD CS
ESP32 GPIO11 -> MicroSD MOSI
ESP32 GPIO12 -> MicroSD SCK
ESP32 GPIO13 -> MicroSD MISO
ESP32 GPIO14 -> W5500 INT
ESP32 GPIO15 -> Solenoid Relay IN
ESP32 GPIO16 <- RDM6300 TX through voltage divider
ESP32 GPIO17 -> Buzzer driver/module IN
ESP32 GPIO18 -> WS2812B DIN through 330 ohm resistor
ESP32 GPIO21 -> W5500 RST
ESP32 GPIO47 -> RDM6300 UART TX dummy assignment only, NOT CONNECTED

============================================================
RDM6300
============================================================

RDM6300 VCC -> 5V
RDM6300 GND -> Common GND
RDM6300 TX -> 1k resistor -> ESP32 GPIO16
ESP32 GPIO16 -> 2k resistor -> GND
RDM6300 RX -> NOT CONNECTED

============================================================
WS2812B 8x8
============================================================

WS2812B 5V -> 5V buck output
WS2812B GND -> Common GND
ESP32 GPIO18 -> 330 ohm resistor -> WS2812B DIN

============================================================
DS3231
============================================================

DS3231 VCC -> ESP32 3V3
DS3231 GND -> Common GND
DS3231 SDA -> ESP32 GPIO8
DS3231 SCL -> ESP32 GPIO9

============================================================
MICROSD
============================================================

MicroSD CS -> ESP32 GPIO10
MicroSD MOSI -> ESP32 GPIO11
MicroSD SCK -> ESP32 GPIO12
MicroSD MISO -> ESP32 GPIO13
MicroSD GND -> Common GND
MicroSD VCC -> module-appropriate supply

The existing patched SDCard driver is now stored unchanged as:
/components/sdcard.py

============================================================
W5500
============================================================

W5500 SCK -> ESP32 GPIO4
W5500 MOSI -> ESP32 GPIO5
W5500 MISO -> ESP32 GPIO6
W5500 CS -> ESP32 GPIO7
W5500 INT -> ESP32 GPIO14
W5500 RST -> ESP32 GPIO21
W5500 GND -> Common GND
W5500 VCC -> 3.3V if the exact module is 3.3V-only
W5500 VIN/5V -> 5V only if the exact breakout has a 5V/VIN regulator input

============================================================
SOLENOID RELAY
============================================================

ESP32 GPIO15 -> Relay IN
Relay module VCC -> 5V
Relay module GND -> Common GND
12V PSU + -> Relay COM
Relay NO -> Solenoid +
Solenoid - -> 12V PSU GND
Flyback diode stripe/cathode -> Solenoid +
Flyback diode other side/anode -> Solenoid -

Firmware relay assumption:
GPIO15 LOW -> Relay OFF -> Solenoid unpowered -> LOCKED
GPIO15 HIGH -> Relay ON -> Solenoid powered -> UNLOCKED

On GRANTED:
Relay unlocks immediately.
Motor opening starts immediately.
Relay returns to LOCK after 1000 ms by default.
The relay does not wait for the motor to return.
The mechanical lock re-catches when the barrier returns to CLOSED.

============================================================
BUZZER
============================================================

ESP32 GPIO17 -> Buzzer driver/module IN
5V/12V buzzer supply -> External correct-voltage supply
Buzzer/driver GND -> Common GND

============================================================
ESP32 POWER
============================================================

12V PSU + -> Buck converter IN+
12V PSU GND -> Buck converter IN-
Buck converter output -> approximately 5.0V
Buck USB output -> USB cable -> ESP32-S3 USB power
ESP32 external 5V/VIN header -> NOT CONNECTED when powering by buck USB

============================================================
RFID ACCESS FLOW
============================================================

RDM6300 reads card.
RFID component checks SD-backed bucket database.
DENIED -> Red X + denied buzzer pattern.
GRANTED -> Green arrow + solenoid unlock pulse + TB6600 OPEN movement.
Gate stays OPEN for configured gate_unlock_sec.
TB6600 returns to CLOSED angle.
Mechanical solenoid latch re-catches automatically.

Default CLOSED angle -> 0 degrees
Default OPEN angle -> 90 degrees
Default solenoid unlock pulse -> 1000 ms
Default gate open hold -> 12 seconds

============================================================
IMPORTANT MOTOR POSITION ASSUMPTION
============================================================

There is currently no home/limit sensor.
At ESP32 boot the firmware assumes the physical gate is already at the configured
CLOSED angle. If the gate is moved while power is off or the motors skip steps,
the software angle can become different from the real mechanical angle.
