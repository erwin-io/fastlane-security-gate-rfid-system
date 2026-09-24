import os


class RFIDDatabase:
    """SD-backed A/B bucket RFID database.

    Raw RDM6300 IDs are 10 hex characters. The first byte chooses one of 256
    bucket files; the remaining 4 bytes are stored as a compact binary record.
    """

    def __init__(
        self,
        db_a,
        db_b,
        active_file,
        old_db,
        legacy_txt,
        record_size=4,
        read_chunk=1024,
        page_default=50,
        page_max=200,
    ):
        self.db_a = db_a
        self.db_b = db_b
        self.active_file = active_file
        self.old_db = old_db
        self.legacy_txt = legacy_txt
        self.record_size = record_size
        self.read_chunk = read_chunk
        self.page_default = page_default
        self.page_max = page_max

        self.sd_ready = False
        self.active_dir = db_a
        self.active_name = "A"
        self.count = 0

    @staticmethod
    def file_exists(path):
        try:
            os.stat(path)
            return True
        except OSError:
            return False

    @staticmethod
    def make_directory(path):
        try:
            os.mkdir(path)
        except OSError:
            pass

    @staticmethod
    def safe_file_size(path):
        try:
            return os.stat(path)[6]
        except Exception:
            return 0

    def normalize_uid(self, uid):
        try:
            uid = str(uid).strip().upper()
        except Exception:
            return None
        if len(uid) != 10:
            return None
        try:
            int(uid, 16)
        except Exception:
            return None
        return uid

    def uid_record(self, uid):
        uid = self.normalize_uid(uid)
        if uid is None:
            return None
        value = int(uid[2:], 16)
        return bytes([
            (value >> 24) & 0xFF,
            (value >> 16) & 0xFF,
            (value >> 8) & 0xFF,
            value & 0xFF,
        ])

    def record_uid(self, bucket, record):
        if len(record) != 4:
            return None
        return "{:02X}{:02X}{:02X}{:02X}{:02X}".format(
            bucket, record[0], record[1], record[2], record[3]
        )

    def db_count_path(self, db_dir):
        return db_dir + "/count.txt"

    def bucket_path_for(self, uid, db_dir=None):
        uid = self.normalize_uid(uid)
        if uid is None:
            return None
        if db_dir is None:
            db_dir = self.active_dir
        return db_dir + "/" + uid[0:2] + ".bin"

    def count_database_records(self, db_dir):
        total = 0
        for bucket in range(256):
            path = db_dir + "/{:02X}.bin".format(bucket)
            total += self.safe_file_size(path) // self.record_size
        return total

    def read_db_count(self, db_dir):
        path = self.db_count_path(db_dir)
        try:
            with open(path, "r") as f:
                return int(f.read().strip())
        except Exception:
            count = self.count_database_records(db_dir)
            self.write_db_count(db_dir, count)
            return count

    def write_db_count(self, db_dir, count):
        try:
            with open(self.db_count_path(db_dir), "w") as f:
                f.write(str(int(count)))
            if hasattr(os, "sync"):
                os.sync()
            return True
        except Exception as e:
            print("COUNT FILE ERROR:", repr(e))
            return False

    def contains(self, uid, db_dir=None):
        if not self.sd_ready:
            return False
        uid = self.normalize_uid(uid)
        if uid is None:
            return False
        if db_dir is None:
            db_dir = self.active_dir

        path = self.bucket_path_for(uid, db_dir)
        needle = self.uid_record(uid)
        if not self.file_exists(path):
            return False

        try:
            with open(path, "rb") as f:
                while True:
                    data = f.read(self.read_chunk)
                    if not data:
                        break
                    usable = len(data) - (len(data) % self.record_size)
                    for i in range(0, usable, self.record_size):
                        if data[i:i + self.record_size] == needle:
                            return True
            return False
        except Exception as e:
            print("RFID LOOKUP ERROR:", repr(e))
            return False

    def append_uid_raw(self, uid, db_dir):
        uid = self.normalize_uid(uid)
        if uid is None:
            return False
        self.make_directory(db_dir)
        path = self.bucket_path_for(uid, db_dir)
        try:
            with open(path, "ab") as f:
                f.write(self.uid_record(uid))
            return True
        except Exception as e:
            print("RFID RAW APPEND ERROR:", repr(e))
            return False

    def add(self, uid, quiet=False, db_dir=None):
        uid = self.normalize_uid(uid)
        if uid is None:
            if not quiet:
                print("INVALID RFID")
            return False
        if not self.sd_ready:
            if not quiet:
                print("SD NOT READY")
            return False
        if db_dir is None:
            db_dir = self.active_dir

        if self.contains(uid, db_dir):
            if not quiet:
                print("RFID ALREADY EXISTS:", uid)
            return False
        if not self.append_uid_raw(uid, db_dir):
            return False

        if db_dir == self.active_dir:
            self.count += 1
            self.write_db_count(self.active_dir, self.count)
        else:
            count = self.read_db_count(db_dir) + 1
            self.write_db_count(db_dir, count)

        if hasattr(os, "sync"):
            os.sync()
        if not quiet:
            print("RFID ADDED TO SD:", uid)
        return True

    def remove(self, uid, quiet=False, db_dir=None):
        uid = self.normalize_uid(uid)
        if uid is None:
            if not quiet:
                print("INVALID RFID")
            return False
        if not self.sd_ready:
            if not quiet:
                print("SD NOT READY")
            return False
        if db_dir is None:
            db_dir = self.active_dir

        path = self.bucket_path_for(uid, db_dir)
        if not self.file_exists(path):
            if not quiet:
                print("RFID NOT FOUND:", uid)
            return False

        needle = self.uid_record(uid)
        tmp = path + ".tmp"
        removed = 0
        try:
            with open(path, "rb") as src, open(tmp, "wb") as dst:
                while True:
                    record = src.read(self.record_size)
                    if not record:
                        break
                    if len(record) != self.record_size:
                        break
                    if record == needle:
                        removed += 1
                    else:
                        dst.write(record)

            if removed == 0:
                try:
                    os.remove(tmp)
                except Exception:
                    pass
                if not quiet:
                    print("RFID NOT FOUND:", uid)
                return False

            os.remove(path)
            os.rename(tmp, path)

            if db_dir == self.active_dir:
                self.count = max(0, self.count - removed)
                self.write_db_count(self.active_dir, self.count)
            else:
                count = max(0, self.read_db_count(db_dir) - removed)
                self.write_db_count(db_dir, count)

            if hasattr(os, "sync"):
                os.sync()
            if not quiet:
                print("RFID REMOVED:", uid)
            return True
        except Exception as e:
            try:
                os.remove(tmp)
            except Exception:
                pass
            if not quiet:
                print("REMOVE RFID ERROR:", repr(e))
            return False

    def find_next_cursor(self, bucket, offset):
        for b in range(bucket, 256):
            path = self.active_dir + "/{:02X}.bin".format(b)
            records = self.safe_file_size(path) // self.record_size
            start = offset if b == bucket else 0
            if start < records:
                return "{:02X}:{}".format(b, start)
        return None

    def list_page(self, cursor=None, limit=None):
        if limit is None:
            limit = self.page_default
        try:
            limit = int(limit)
        except Exception:
            limit = self.page_default
        limit = max(1, min(self.page_max, limit))

        bucket = 0
        offset = 0
        if cursor:
            try:
                parts = str(cursor).split(":", 1)
                bucket = int(parts[0], 16)
                offset = int(parts[1])
                if bucket < 0 or bucket > 255 or offset < 0:
                    raise ValueError()
            except Exception:
                bucket = 0
                offset = 0

        records_out = []
        current_bucket = bucket
        current_offset = offset

        while current_bucket < 256 and len(records_out) < limit:
            path = self.active_dir + "/{:02X}.bin".format(current_bucket)
            count = self.safe_file_size(path) // self.record_size
            if current_offset < count and self.file_exists(path):
                try:
                    with open(path, "rb") as f:
                        f.seek(current_offset * self.record_size)
                        while current_offset < count and len(records_out) < limit:
                            record = f.read(self.record_size)
                            if len(record) != self.record_size:
                                current_offset = count
                                break
                            records_out.append(self.record_uid(current_bucket, record))
                            current_offset += 1
                except Exception as e:
                    print("RFID PAGE READ ERROR:", repr(e))
                    current_offset = count

            if len(records_out) >= limit:
                break
            current_bucket += 1
            current_offset = 0

        return {
            "items": records_out,
            "next_cursor": self.find_next_cursor(current_bucket, current_offset),
            "count": self.count,
            "active_db": self.active_name,
        }

    def _migrate_old_database_layout(self):
        if self.file_exists(self.old_db) and not self.file_exists(self.db_a):
            try:
                os.rename(self.old_db, self.db_a)
                print("OLD RFID DB MOVED:", self.old_db, "->", self.db_a)
            except Exception as e:
                print("OLD RFID DB MOVE ERROR:", repr(e))

    def _load_active_marker(self):
        name = "A"
        try:
            with open(self.active_file, "r") as f:
                value = f.read().strip().upper()
                if value in ("A", "B"):
                    name = value
        except Exception:
            pass
        self.active_name = name
        self.active_dir = self.db_a if name == "A" else self.db_b
        self.make_directory(self.active_dir)

    def save_active_marker(self, name):
        try:
            with open(self.active_file, "w") as f:
                f.write(name)
            if hasattr(os, "sync"):
                os.sync()
            return True
        except Exception as e:
            print("ACTIVE DB MARKER ERROR:", repr(e))
            return False

    def activate(self, name, db_dir):
        if not self.save_active_marker(name):
            return False
        self.active_name = name
        self.active_dir = db_dir
        self.count = self.read_db_count(self.active_dir)
        return True

    def inactive_db(self):
        if self.active_name == "A":
            return "B", self.db_b
        return "A", self.db_a

    def remove_directory_files(self, path):
        self.make_directory(path)
        try:
            for name in os.listdir(path):
                p = path + "/" + name
                try:
                    os.remove(p)
                except Exception:
                    pass
        except Exception:
            pass

    def _migrate_legacy_txt_if_needed(self):
        if self.count > 0 or not self.file_exists(self.legacy_txt):
            return
        print("MIGRATING LEGACY RFID TXT ...")
        migrated = 0
        try:
            with open(self.legacy_txt, "r") as f:
                for line in f:
                    uid = line.strip().upper()
                    if not uid or uid.startswith("#"):
                        continue
                    uid = self.normalize_uid(uid)
                    if uid is None:
                        continue
                    if not self.contains(uid, self.active_dir):
                        if self.append_uid_raw(uid, self.active_dir):
                            migrated += 1
            self.count = self.count_database_records(self.active_dir)
            self.write_db_count(self.active_dir, self.count)
            print("LEGACY MIGRATION COMPLETE:", migrated, "RFIDs")
        except Exception as e:
            print("LEGACY MIGRATION ERROR:", repr(e))

    def initialize(self, sd_ready):
        self.sd_ready = bool(sd_ready)
        if not self.sd_ready:
            print("RFID DATABASE NOT READY: SD unavailable")
            self.count = 0
            return False

        self._migrate_old_database_layout()
        self.make_directory(self.db_a)
        self.make_directory(self.db_b)
        self._load_active_marker()
        self.count = self.read_db_count(self.active_dir)
        self._migrate_legacy_txt_if_needed()
        self.count = self.read_db_count(self.active_dir)

        print()
        print("RFID DATABASE MODE: SD-BACKED A/B BUCKET DATABASE")
        print("RFID IDs loaded into RAM: 0")
        print("Active DB:", self.active_name)
        print("Database records:", self.count)
        return True
