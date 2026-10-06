from machine import Pin, I2C, RTC
import esp32
import time


def calculate_weekday(year, month, day):
    offsets = [0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4]
    y = year
    if month < 3:
        y -= 1
    sunday_zero = (y + y // 4 - y // 100 + y // 400 + offsets[month - 1] + day) % 7
    return (sunday_zero - 1) % 7


def datetime_valid(dt):
    try:
        year, month, day, hour, minute, second = dt
        return (
            2024 <= year <= 2199
            and 1 <= month <= 12
            and 1 <= day <= 31
            and 0 <= hour <= 23
            and 0 <= minute <= 59
            and 0 <= second <= 59
        )
    except Exception:
        return False


def format_datetime(dt):
    return "{:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}".format(
        dt[0], dt[1], dt[2], dt[3], dt[4], dt[5]
    )


class DS3231Clock:
    def __init__(
        self,
        sda_pin,
        scl_pin,
        address=0x68,
        frequency=100000,
        force_set=False,
        initial_datetime=(2026, 9, 17, 4, 30, 0),
        resync_ms=60000,
        nvs_namespace="gateclock",
        i2c=None,
    ):
        self.sda_pin = sda_pin
        self.scl_pin = scl_pin
        self.address = address
        self.frequency = frequency
        self.force_set = bool(force_set)
        self.initial_datetime = initial_datetime
        self.resync_ms = int(resync_ms)
        self.nvs_namespace = nvs_namespace

        # Optional shared I2C bus. FASTLANE v1.6 shares I2C0 between
        # DS3231 and the two VL53L0X sensors.
        self.i2c = i2c
        self.machine_rtc = RTC()
        self.nvs = esp32.NVS(nvs_namespace)
        self.ready = False
        self.last_sync = 0

    @staticmethod
    def _bcd_to_dec(value):
        return ((value >> 4) * 10) + (value & 0x0F)

    @staticmethod
    def _dec_to_bcd(value):
        return ((value // 10) << 4) | (value % 10)

    def _lost_power(self):
        status = self.i2c.readfrom_mem(self.address, 0x0F, 1)[0]
        return bool(status & 0x80)

    def _clear_lost_power(self):
        status = self.i2c.readfrom_mem(self.address, 0x0F, 1)[0]
        status &= 0x7F
        self.i2c.writeto_mem(self.address, 0x0F, bytes([status]))

    def _read_chip_datetime(self):
        data = self.i2c.readfrom_mem(self.address, 0x00, 7)
        second = self._bcd_to_dec(data[0] & 0x7F)
        minute = self._bcd_to_dec(data[1] & 0x7F)
        hour_register = data[2]

        if hour_register & 0x40:
            hour = self._bcd_to_dec(hour_register & 0x1F)
            pm = bool(hour_register & 0x20)
            if hour == 12:
                hour = 0
            if pm:
                hour += 12
        else:
            hour = self._bcd_to_dec(hour_register & 0x3F)

        day = self._bcd_to_dec(data[4] & 0x3F)
        month_register = data[5]
        month = self._bcd_to_dec(month_register & 0x1F)
        year = 2000 + self._bcd_to_dec(data[6])
        if month_register & 0x80:
            year += 100

        return (year, month, day, hour, minute, second)

    def _write_chip_datetime(self, dt):
        year, month, day, hour, minute, second = dt
        weekday = calculate_weekday(year, month, day) + 1
        century = 0
        if year >= 2100:
            century = 0x80
            year -= 2100
        else:
            year -= 2000

        data = bytes([
            self._dec_to_bcd(second),
            self._dec_to_bcd(minute),
            self._dec_to_bcd(hour),
            self._dec_to_bcd(weekday),
            self._dec_to_bcd(day),
            self._dec_to_bcd(month) | century,
            self._dec_to_bcd(year),
        ])
        self.i2c.writeto_mem(self.address, 0x00, data)
        self._clear_lost_power()

    def _save_nvs(self, dt):
        try:
            self.nvs.set_i32("yr", dt[0])
            self.nvs.set_i32("mo", dt[1])
            self.nvs.set_i32("dy", dt[2])
            self.nvs.set_i32("hr", dt[3])
            self.nvs.set_i32("mi", dt[4])
            self.nvs.set_i32("sc", dt[5])
            self.nvs.commit()
        except Exception as e:
            print("NVS RTC SAVE ERROR:", repr(e))

    def _load_nvs(self):
        try:
            dt = (
                self.nvs.get_i32("yr"),
                self.nvs.get_i32("mo"),
                self.nvs.get_i32("dy"),
                self.nvs.get_i32("hr"),
                self.nvs.get_i32("mi"),
                self.nvs.get_i32("sc"),
            )
            if datetime_valid(dt):
                return dt
        except Exception:
            pass
        return None

    def _set_machine_rtc(self, dt):
        year, month, day, hour, minute, second = dt
        weekday = calculate_weekday(year, month, day)
        self.machine_rtc.datetime((year, month, day, weekday, hour, minute, second, 0))

    def _get_machine_datetime(self):
        dt = self.machine_rtc.datetime()
        return (dt[0], dt[1], dt[2], dt[4], dt[5], dt[6])

    def initialize(self):
        print()
        print("========================================")
        print("CHECKING DS3231 RTC")
        print("========================================")

        if self.i2c is None:
            self.i2c = I2C(
                0,
                sda=Pin(self.sda_pin),
                scl=Pin(self.scl_pin),
                freq=self.frequency,
            )

        try:
            devices = self.i2c.scan()
            print("I2C devices:", [hex(x) for x in devices])
        except Exception as e:
            devices = []
            print("I2C SCAN ERROR:", repr(e))

        self.ready = self.address in devices
        print("DS3231 FOUND" if self.ready else "DS3231 NOT FOUND")

        if self.ready:
            try:
                if self.force_set:
                    self._write_chip_datetime(self.initial_datetime)
                if not self._lost_power():
                    dt = self._read_chip_datetime()
                    if datetime_valid(dt):
                        self._set_machine_rtc(dt)
                        self._save_nvs(dt)
                        print("CLOCK SOURCE: DS3231")
                        print("TIME:", format_datetime(dt))
                        return True
            except Exception as e:
                print("DS3231 READ ERROR:", repr(e))

        stored = self._load_nvs()
        if stored:
            self._set_machine_rtc(stored)
            print("CLOCK SOURCE: NVS")
            print("TIME:", format_datetime(stored))
            return self.ready

        self._set_machine_rtc(self.initial_datetime)
        print("CLOCK SOURCE: INITIAL_DATETIME")
        print("TIME:", format_datetime(self.initial_datetime))
        return self.ready

    def current_datetime(self):
        if self.ready:
            try:
                if not self._lost_power():
                    dt = self._read_chip_datetime()
                    if datetime_valid(dt):
                        return dt
            except Exception:
                pass
        return self._get_machine_datetime()

    def update(self):
        if not self.ready:
            return
        now = time.ticks_ms()
        if time.ticks_diff(now, self.last_sync) < self.resync_ms:
            return
        self.last_sync = now
        try:
            dt = self._read_chip_datetime()
            if datetime_valid(dt):
                self._set_machine_rtc(dt)
                self._save_nvs(dt)
        except Exception as e:
            print("RTC SYNC ERROR:", repr(e))
