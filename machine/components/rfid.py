import os

from components.card_id import (
    normalize_card_id,
    card_id_record,
    card_id_from_record,
    card_id_bucket,
    card_id_from_frame_hex,
)


class RFIDDatabase:
    """SD-backed A/B bucket database of decimal card IDs.

    v2.2.0 layout (decimal card IDs, see components/card_id.py):
      <db_dir>/NNN.bin   NNN = card value mod 256, written as 3 decimal digits
                         (000.bin .. 255.bin); each record is the 4-byte
                         big-endian card value, so "0008511448" -> 00 81 DF D8
      <db_dir>/count.txt record count
      active_file        "A" or "B"

    The pre-v2.2.0 hex-UID database (/sd/rfid_db_A|B, /sd/rfid_db,
    /sd/rfids.txt) is read ONCE at boot and converted (see
    _migrate_legacy_hex_if_needed). It is never written or deleted, so an
    older firmware can still be rolled back onto the same SD card.
    """

    BUCKETS = 256

    def __init__(
        self,
        db_a,
        db_b,
        active_file,
        legacy_hex_sources=(),
        legacy_hex_txt=None,
        record_size=4,
        read_chunk=1024,
        page_default=50,
        page_max=200,
    ):
        self.db_a = db_a
        self.db_b = db_b
        self.active_file = active_file
        # Ordered (dir) or (active_marker_file, dir_a, dir_b) tuples of old
        # hex-bucket databases; first one with records wins.
        self.legacy_hex_sources = tuple(legacy_hex_sources)
        self.legacy_hex_txt = legacy_hex_txt
        self.record_size = record_size
        self.read_chunk = read_chunk
        self.page_default = page_default
        self.page_max = page_max

        self.sd_ready = False
        self.active_dir = db_a
        self.active_name = "A"
        self.count = 0
        self.migrated_from = ""
        self.migrated_count = 0
        # v2.3.0 bucket-size cache: record count of each of the 256 bucket
        # files of ONE directory, filled by a single directory listing
        # (~30 ms) instead of 256 os.stat() calls (~31 ms EACH on the gate's
        # SoftSPI SD card; Browse took 8.4 s for 4 cards in v2.2.0).
        self._sizes = None
        self._sizes_dir = None

    # ------------------------------------------------------------------
    # filesystem helpers
    # ------------------------------------------------------------------
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

    @staticmethod
    def _sync():
        if hasattr(os, "sync"):
            os.sync()

    # ------------------------------------------------------------------
    # card ID / record helpers
    # ------------------------------------------------------------------
    @staticmethod
    def normalize_card_id(card_id):
        return normalize_card_id(card_id)

    def card_id_record(self, card_id):
        card_id = normalize_card_id(card_id)
        if card_id is None:
            return None
        return card_id_record(card_id)

    @staticmethod
    def bucket_file(db_dir, bucket):
        return db_dir + "/{:03d}.bin".format(bucket)

    def db_count_path(self, db_dir):
        return db_dir + "/count.txt"

    def bucket_path_for(self, card_id, db_dir=None):
        card_id = normalize_card_id(card_id)
        if card_id is None:
            return None
        if db_dir is None:
            db_dir = self.active_dir
        return self.bucket_file(db_dir, card_id_bucket(card_id))

    def scan_bucket_sizes(self, db_dir):
        """Record count per bucket from ONE directory listing.

        Raises OSError when the directory cannot be read (SD fault / missing),
        so callers can tell "empty" from "unreadable".
        """
        sizes = [0] * self.BUCKETS
        rs = self.record_size
        if hasattr(os, "ilistdir"):
            entries = os.ilistdir(db_dir)
            for entry in entries:
                name = entry[0]
                if len(name) != 7 or not name.endswith(".bin"):
                    continue
                try:
                    b = int(name[0:3])
                except ValueError:
                    continue
                if 0 <= b < self.BUCKETS:
                    size = entry[3] if len(entry) > 3 else self.safe_file_size(db_dir + "/" + name)
                    sizes[b] = int(size) // rs
        else:  # CPython host tests
            for name in os.listdir(db_dir):
                if len(name) == 7 and name.endswith(".bin") and name[0:3].isdigit():
                    b = int(name[0:3])
                    if b < self.BUCKETS:
                        sizes[b] = os.stat(db_dir + "/" + name)[6] // rs
        return sizes

    def bucket_sizes(self, db_dir=None, refresh=False):
        if db_dir is None:
            db_dir = self.active_dir
        if refresh or self._sizes is None or self._sizes_dir != db_dir:
            self._sizes = self.scan_bucket_sizes(db_dir)
            self._sizes_dir = db_dir
        return self._sizes

    def invalidate_sizes(self, db_dir=None):
        if db_dir is None or db_dir == self._sizes_dir:
            self._sizes = None
            self._sizes_dir = None

    def _cache_adjust(self, db_dir, bucket, delta):
        if self._sizes is not None and self._sizes_dir == db_dir:
            self._sizes[bucket] = max(0, self._sizes[bucket] + delta)

    def count_database_records(self, db_dir):
        try:
            return sum(self.scan_bucket_sizes(db_dir))
        except Exception:
            return 0

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
            self._sync()
            return True
        except Exception as e:
            print("COUNT FILE ERROR:", repr(e))
            return False

    # ------------------------------------------------------------------
    # lookup / add / remove
    # ------------------------------------------------------------------
    def _bucket_contains(self, path, needle):
        # No separate os.stat(): a missing bucket raises ENOENT on open.
        try:
            f = open(path, "rb")
        except OSError as e:
            if e.args and e.args[0] == 2:
                return False
            raise
        with f:
            while True:
                data = f.read(self.read_chunk)
                if not data:
                    break
                usable = len(data) - (len(data) % self.record_size)
                for i in range(0, usable, self.record_size):
                    if data[i:i + self.record_size] == needle:
                        return True
        return False

    def contains(self, card_id, db_dir=None):
        if not self.sd_ready:
            return False
        card_id = normalize_card_id(card_id)
        if card_id is None:
            return False
        if db_dir is None:
            db_dir = self.active_dir
        # v2.3.0: an empty bucket (known from the cache) needs no SD access.
        if self._sizes is not None and self._sizes_dir == db_dir:
            if self._sizes[card_id_bucket(card_id)] == 0:
                return False
        try:
            return self._bucket_contains(
                self.bucket_path_for(card_id, db_dir), card_id_record(card_id)
            )
        except Exception as e:
            print("RFID LOOKUP ERROR:", repr(e))
            return False

    def append_card_id_raw(self, card_id, db_dir):
        card_id = normalize_card_id(card_id)
        if card_id is None:
            return False
        self.make_directory(db_dir)
        try:
            with open(self.bucket_path_for(card_id, db_dir), "ab") as f:
                f.write(card_id_record(card_id))
            self._cache_adjust(db_dir, card_id_bucket(card_id), 1)
            return True
        except Exception as e:
            print("RFID RAW APPEND ERROR:", repr(e))
            return False

    def add(self, card_id, quiet=False, db_dir=None):
        card_id = normalize_card_id(card_id)
        if card_id is None:
            if not quiet:
                print("INVALID CARD ID")
            return False
        if not self.sd_ready:
            if not quiet:
                print("SD NOT READY")
            return False
        if db_dir is None:
            db_dir = self.active_dir

        if self.contains(card_id, db_dir):
            if not quiet:
                print("CARD ALREADY EXISTS:", card_id)
            return False
        if not self.append_card_id_raw(card_id, db_dir):
            return False

        if db_dir == self.active_dir:
            self.count += 1
            self.write_db_count(self.active_dir, self.count)
        else:
            self.write_db_count(db_dir, self.read_db_count(db_dir) + 1)

        self._sync()
        if not quiet:
            print("CARD ADDED TO SD:", card_id)
        return True

    def remove(self, card_id, quiet=False, db_dir=None):
        card_id = normalize_card_id(card_id)
        if card_id is None:
            if not quiet:
                print("INVALID CARD ID")
            return False
        if not self.sd_ready:
            if not quiet:
                print("SD NOT READY")
            return False
        if db_dir is None:
            db_dir = self.active_dir

        path = self.bucket_path_for(card_id, db_dir)
        if not self.file_exists(path):
            if not quiet:
                print("CARD NOT FOUND:", card_id)
            return False

        needle = card_id_record(card_id)
        tmp = path + ".tmp"
        removed = 0
        try:
            with open(path, "rb") as src, open(tmp, "wb") as dst:
                while True:
                    record = src.read(self.record_size)
                    if not record or len(record) != self.record_size:
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
                    print("CARD NOT FOUND:", card_id)
                return False

            os.remove(path)
            os.rename(tmp, path)
            self._cache_adjust(db_dir, card_id_bucket(card_id), -removed)

            if db_dir == self.active_dir:
                self.count = max(0, self.count - removed)
                self.write_db_count(self.active_dir, self.count)
            else:
                self.write_db_count(db_dir, max(0, self.read_db_count(db_dir) - removed))

            self._sync()
            if not quiet:
                print("CARD REMOVED:", card_id)
            return True
        except Exception as e:
            try:
                os.remove(tmp)
            except Exception:
                pass
            if not quiet:
                print("REMOVE CARD ERROR:", repr(e))
            return False

    # ------------------------------------------------------------------
    # paging (cursor "BBB:offset", bucket in decimal)
    # ------------------------------------------------------------------
    def find_next_cursor(self, bucket, offset, sizes=None):
        if sizes is None:
            sizes = self.bucket_sizes()
        for b in range(bucket, self.BUCKETS):
            start = offset if b == bucket else 0
            if start < sizes[b]:
                return "{:03d}:{}".format(b, start)
        return None

    def list_page(self, cursor=None, limit=None):
        """One page of decimal card IDs, read straight from the SD card.

        v2.3.0: bucket sizes come from the cache (refreshed by one directory
        listing on the first page), so only buckets that hold records are
        opened. SD read failures RAISE OSError instead of returning an empty
        page, so the web UI can show the error instead of "no rows".
        """
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
                bucket = int(parts[0])
                offset = int(parts[1])
                if bucket < 0 or bucket >= self.BUCKETS or offset < 0:
                    raise ValueError()
            except Exception:
                bucket = 0
                offset = 0

        sizes = self.bucket_sizes(refresh=not cursor)
        records_out = []
        current_bucket = bucket
        current_offset = offset

        while current_bucket < self.BUCKETS and len(records_out) < limit:
            count = sizes[current_bucket]
            if current_offset < count:
                path = self.bucket_file(self.active_dir, current_bucket)
                with open(path, "rb") as f:
                    f.seek(current_offset * self.record_size)
                    want = min(count - current_offset, limit - len(records_out))
                    data = f.read(want * self.record_size)
                for i in range(0, len(data) - self.record_size + 1, self.record_size):
                    card_id = card_id_from_record(data[i:i + self.record_size])
                    if card_id is not None:
                        records_out.append(card_id)
                    current_offset += 1
                if len(data) < want * self.record_size:
                    current_offset = count     # file shorter than cached size
            if len(records_out) >= limit:
                break
            current_bucket += 1
            current_offset = 0

        return {
            "items": records_out,
            "next_cursor": self.find_next_cursor(current_bucket, current_offset, sizes),
            "count": self.count,
            "active_db": self.active_name,
        }

    # ------------------------------------------------------------------
    # A/B activation
    # ------------------------------------------------------------------
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
            self._sync()
            return True
        except Exception as e:
            print("ACTIVE DB MARKER ERROR:", repr(e))
            return False

    def activate(self, name, db_dir):
        if not self.save_active_marker(name):
            return False
        self.active_name = name
        self.active_dir = db_dir
        self.invalidate_sizes()
        self.count = self.read_db_count(self.active_dir)
        return True

    def inactive_db(self):
        if self.active_name == "A":
            return "B", self.db_b
        return "A", self.db_a

    def remove_directory_files(self, path):
        self.invalidate_sizes(path)
        self.make_directory(path)
        try:
            for name in os.listdir(path):
                try:
                    os.remove(path + "/" + name)
                except Exception:
                    pass
        except Exception:
            pass

    # ------------------------------------------------------------------
    # one-time migration from the pre-v2.2.0 hex-UID database
    # ------------------------------------------------------------------
    def _legacy_hex_dir(self, source):
        """Resolve a legacy source to one hex-bucket directory (or None)."""
        if isinstance(source, (tuple, list)):
            marker, dir_a, dir_b = source
            name = "A"
            try:
                with open(marker, "r") as f:
                    if f.read().strip().upper() == "B":
                        name = "B"
            except Exception:
                pass
            return dir_a if name == "A" else dir_b
        return source

    def _legacy_hex_dir_ids(self, hex_dir):
        """Yield decimal card IDs from an old <hex_dir>/VV.bin bucket layout."""
        for bucket in range(256):
            path = hex_dir + "/{:02X}.bin".format(bucket)
            size = self.safe_file_size(path)
            if size < self.record_size:
                continue
            with open(path, "rb") as f:
                while True:
                    record = f.read(self.record_size)
                    if len(record) != self.record_size:
                        break
                    # Old record = bytes 2..5 of the raw UID = the card value.
                    card_id = card_id_from_record(record)
                    if card_id is not None:
                        yield card_id

    def _legacy_txt_ids(self, path):
        with open(path, "r") as f:
            for line in f:
                line = line.strip().upper()
                if not line or line.startswith("#"):
                    continue
                card_id = card_id_from_frame_hex(line)
                if card_id is not None:
                    yield card_id

    def _write_migrated(self, ids, target_dir):
        """Bucket + de-duplicate in RAM per bucket, then one write per bucket."""
        buckets = {}
        seen = set()
        for card_id in ids:
            if card_id in seen:
                continue
            seen.add(card_id)
            b = card_id_bucket(card_id)
            if b not in buckets:
                buckets[b] = bytearray()
            buckets[b].extend(card_id_record(card_id))
        for b, data in buckets.items():
            with open(self.bucket_file(target_dir, b), "wb") as f:
                f.write(data)
        self.invalidate_sizes(target_dir)
        return len(seen)

    def _migrate_legacy_hex_if_needed(self):
        """Convert the old hex database once, when the decimal DB is new.

        Runs only when the decimal active marker does not exist yet. Writes
        into db_a, then count.txt, then the marker LAST, so a power cut part
        way simply repeats the migration on the next boot.
        """
        if self.file_exists(self.active_file):
            return

        source_name = ""
        ids = None
        for source in self.legacy_hex_sources:
            hex_dir = self._legacy_hex_dir(source)
            if hex_dir and self.file_exists(hex_dir):
                try:
                    found = list(self._legacy_hex_dir_ids(hex_dir))
                except Exception as e:
                    print("LEGACY HEX DB READ ERROR:", hex_dir, repr(e))
                    continue
                if found:
                    source_name, ids = hex_dir, found
                    break
        if ids is None and self.legacy_hex_txt and self.file_exists(self.legacy_hex_txt):
            try:
                found = list(self._legacy_txt_ids(self.legacy_hex_txt))
                if found:
                    source_name, ids = self.legacy_hex_txt, found
            except Exception as e:
                print("LEGACY HEX TXT READ ERROR:", repr(e))

        self.remove_directory_files(self.db_a)
        self.make_directory(self.db_b)
        migrated = 0
        if ids:
            print("MIGRATING HEX UID DATABASE -> DECIMAL CARD IDs:", source_name)
            try:
                migrated = self._write_migrated(ids, self.db_a)
            except Exception as e:
                print("CARD ID MIGRATION ERROR:", repr(e))
                return
        self.write_db_count(self.db_a, migrated)
        if not self.save_active_marker("A"):
            return
        self.migrated_from = source_name
        self.migrated_count = migrated
        if ids:
            print("CARD ID MIGRATION COMPLETE:", migrated, "card(s) from", len(ids),
                  "record(s) | old database left untouched")

    # ------------------------------------------------------------------
    def initialize(self, sd_ready):
        self.sd_ready = bool(sd_ready)
        if not self.sd_ready:
            print("RFID DATABASE NOT READY: SD unavailable")
            self.count = 0
            return False

        self.make_directory(self.db_a)
        self.make_directory(self.db_b)
        self._migrate_legacy_hex_if_needed()
        self._load_active_marker()
        self.count = self.read_db_count(self.active_dir)
        try:
            self.bucket_sizes(refresh=True)
        except Exception as e:
            print("RFID BUCKET SIZE CACHE ERROR:", repr(e))
            self.invalidate_sizes()

        print()
        print("RFID DATABASE MODE: SD-BACKED A/B BUCKET DATABASE (DECIMAL CARD IDs)")
        print("Card IDs loaded into RAM: 0")
        print("Active DB:", self.active_name, self.active_dir)
        print("Database records:", self.count)
        return True
