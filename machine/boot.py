# ============================================================
# FASTLANE ESP32-S3 BOOT / FAIL-SAFE OUTPUT INITIALIZATION
# ============================================================
#
# This file runs BEFORE main.py. GPIO15 is forced to the relay
# LOCKED/OFF state immediately so the solenoid cannot remain
# energized while the rest of the application is importing.
#
# IMPORTANT HARDWARE ASSUMPTION - FIXED FOR THIS MACHINE:
#   Relay module = ACTIVE LOW
#   GPIO15 HIGH  = relay OFF = solenoid unpowered = LOCKED
#   GPIO15 LOW   = relay ON  = solenoid powered   = UNLOCKED
#
# Keep these values synchronized with config.py:
#   SOLENOID_RELAY_ACTIVE_LOW = True
# ============================================================

from machine import Pin
import gc

BOOT_SOLENOID_RELAY_PIN = 15
BOOT_RELAY_LOCKED_LEVEL = 1

# Set the fail-safe output before doing diagnostics or importing main.py.
_boot_solenoid_relay = Pin(
    BOOT_SOLENOID_RELAY_PIN,
    Pin.OUT,
    value=BOOT_RELAY_LOCKED_LEVEL,
)

print(
    "BOOT SOLENOID RELAY: LOCKED - GPIO{}={}".format(
        BOOT_SOLENOID_RELAY_PIN,
        _boot_solenoid_relay.value(),
    )
)

gc.collect()

try:
    print(
        "BOOT HEAP BEFORE MAIN: free={} bytes, allocated={} bytes".format(
            gc.mem_free(), gc.mem_alloc()
        )
    )
except Exception:
    pass
