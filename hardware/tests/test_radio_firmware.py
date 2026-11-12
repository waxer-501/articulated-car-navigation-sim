"""Fake SPI and Flash tests; never connect to radio/MCU/vehicle hardware."""
import ctypes as C
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SELECT = C.CFUNCTYPE(None, C.c_void_p, C.c_int)
TRANSFER = C.CFUNCTYPE(C.c_int, C.c_void_p, C.c_uint8)
READ = C.CFUNCTYPE(C.c_uint32, C.c_void_p, C.c_uint, C.c_uint)
ERASE = C.CFUNCTYPE(C.c_int, C.c_void_p, C.c_uint)
PROGRAM = C.CFUNCTYPE(C.c_int, C.c_void_p, C.c_uint, C.c_uint, C.c_uint32)


class IO(C.Structure):
    _fields_ = [('ctx', C.c_void_p), ('select', SELECT), ('ce', SELECT), ('transfer', TRANSFER)]


class Radio(C.Structure):
    _fields_ = [('io', IO), ('started_us', C.c_uint32), ('io_errors', C.c_uint32),
                ('phase', C.c_uint8), ('last_status', C.c_uint8), ('config', C.c_uint8*8)]


class Storage(C.Structure):
    _fields_ = [('ctx', C.c_void_p), ('read', READ), ('erase', ERASE), ('program', PROGRAM)]


class FakeSPI:
    def __init__(self, locked=False, missing=False):
        self.regs = {i: bytearray(5 if i in (0x0a, 0x10) else 1) for i in range(32)}
        self.locked, self.missing, self.fail = locked, missing, False
        self.status, self.command, self.position = 14, None, 0
        self.ce_level, self.ce_highs, self.flushes = 0, 0, 0
        self.payloads, self.payload = [], bytearray()
        self.callbacks = (SELECT(self.select), SELECT(self.ce), TRANSFER(self.transfer))
        self.io = IO(None, *self.callbacks)

    def select(self, _, selected):
        if selected:
            self.command, self.position = None, 0
        elif self.command == 0xa0:
            self.payloads.append(bytes(self.payload))

    def ce(self, _, high):
        self.ce_level = high
        self.ce_highs += bool(high)

    def transfer(self, _, value):
        if self.fail: return -1
        if self.missing: return 255
        if self.command is None:
            self.command = value
            if value == 0xe1: self.flushes += 1
            if value == 0xa0: self.payload = bytearray()
            return self.status
        cmd, pos = self.command, self.position
        self.position += 1
        if cmd == 0x50 and value == 0x73:
            self.locked = not self.locked
        elif cmd == 0xa0:
            self.payload.append(value)
        elif 0x20 <= cmd <= 0x3f:
            reg = cmd & 31
            if reg == 7: self.status &= ~(value & 0x70)
            elif not (reg == 0x1d and self.locked): self.regs[reg][pos] = value
        elif cmd < 0x20:
            return self.regs[cmd][pos]
        return 255


class Flash:
    def __init__(self):
        self.words = [0xffffffff]*512
        self.fail_program = 0
        self.fail_after_write = False
        self.fail_erase = False
        self.program_calls = self.erases = 0
        self.callbacks = (READ(self.read), ERASE(self.erase), PROGRAM(self.program))
        self.io = Storage(None, *self.callbacks)

    def read(self, _, page, word): return self.words[page*256+word]

    def erase(self, _, page):
        if self.fail_erase: return -1
        self.erases += 1
        self.words[page*256:(page+1)*256] = [0xffffffff]*256
        return 0

    def program(self, _, page, word, value):
        self.program_calls += 1
        failing = self.fail_program == self.program_calls
        if failing and not self.fail_after_write: return -1
        index = page*256+word
        self.words[index] &= value
        return -1 if failing else 0


class RadioFirmwareTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix='radio-fake-')
        out = Path(cls.temp.name)/'radio.so'
        subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-fPIC', '-shared',
                        str(ROOT/'firmware/common/nrf24_tx.c'), str(ROOT/'firmware/common/boot_journal.c'),
                        '-o', str(out)], check=True)
        cls.lib = C.CDLL(str(out))
        cls.lib.nrf24_tx_init.argtypes = [C.POINTER(Radio), C.POINTER(IO)]
        cls.lib.nrf24_tx_start.argtypes = [C.POINTER(Radio), C.POINTER(C.c_uint8), C.c_uint, C.c_uint32]
        cls.lib.nrf24_tx_poll.argtypes = [C.POINTER(Radio), C.c_uint32]
        cls.lib.nrf24_tx_power_down.argtypes = [C.POINTER(Radio)]
        cls.lib.boot_journal_next.argtypes = [C.POINTER(Storage)]
        cls.lib.boot_journal_next.restype = C.c_uint32

    @classmethod
    def tearDownClass(cls): cls.temp.cleanup()

    def setup_radio(self, **kwargs):
        fake, radio = FakeSPI(**kwargs), Radio()
        result = self.lib.nrf24_tx_init(C.byref(radio), C.byref(fake.io))
        return fake, radio, result

    def start(self, radio, now=0):
        payload = (C.c_uint8*16)(*range(16))
        return self.lib.nrf24_tx_start(C.byref(radio), payload, 16, now)

    def test_profile_and_no_rf_during_initialization(self):
        f, r, rc = self.setup_radio()
        self.assertEqual(rc, 0)
        self.assertEqual(bytes(r.config), bytes([10,1,1,3,255,60,38,1]))
        self.assertEqual(f.regs[0x10], bytes.fromhex('3443101001'))
        self.assertEqual(f.ce_highs, 0)

    def test_feature_activation_when_required(self):
        f, _, rc = self.setup_radio(locked=True)
        self.assertEqual(rc, 0)
        self.assertFalse(f.locked)
        self.assertEqual(f.regs[0x1d][0], 7)

    def test_missing_device_never_asserts_ce(self):
        f, _, rc = self.setup_radio(missing=True)
        self.assertLess(rc, 0)
        self.assertEqual(f.ce_highs, 0)

    def test_ce_minimum_pulse_does_not_include_spi_loading(self):
        f, r, _ = self.setup_radio()
        self.assertEqual(self.start(r), 0)
        self.assertEqual(f.payloads, [bytes(range(16))])
        f.status |= 0x20
        self.assertEqual(self.lib.nrf24_tx_poll(C.byref(r), 1000), 0)
        self.assertEqual(self.lib.nrf24_tx_poll(C.byref(r), 1014), 0)
        self.assertEqual(f.ce_level, 1)
        self.assertEqual(self.lib.nrf24_tx_poll(C.byref(r), 1015), 1)
        self.assertEqual(f.ce_level, 0)

    def test_max_retries_and_timeout_leave_radio_idle(self):
        for status in (0x10, 0):
            f, r, _ = self.setup_radio(); self.start(r)
            self.lib.nrf24_tx_poll(C.byref(r), 0)
            f.status |= status
            self.assertEqual(self.lib.nrf24_tx_poll(C.byref(r), 1000000), -1)
            self.assertEqual(r.phase, 0)
            self.assertEqual(f.ce_level, 0)
            self.assertGreaterEqual(f.flushes, 2)

    def test_spi_error_is_reported_and_not_transmitted(self):
        f, r, _ = self.setup_radio(); f.fail = True
        self.assertEqual(self.start(r), -1)
        self.assertGreater(r.io_errors, 0)
        self.assertEqual(f.ce_highs, 0)

    def test_powerdown_drops_ce_and_pwr_up(self):
        f, r, _ = self.setup_radio(); self.start(r)
        self.lib.nrf24_tx_power_down(C.byref(r))
        self.assertEqual(f.ce_level, 0)
        self.assertEqual(f.regs[0][0], 8)

    def test_boot_ids_increase_across_page_rollovers(self):
        flash = Flash()
        ids = [self.lib.boot_journal_next(C.byref(flash.io)) for _ in range(300)]
        self.assertEqual(ids, list(range(1,301)))
        self.assertEqual(flash.erases, 2)

    def test_interrupted_record_does_not_reuse_committed_id(self):
        for fail_index in (1,2):
            for write_before_failure in (False, True):
                flash = Flash()
                first = self.lib.boot_journal_next(C.byref(flash.io))
                flash.fail_program = flash.program_calls+fail_index
                flash.fail_after_write = write_before_failure
                self.assertEqual(self.lib.boot_journal_next(C.byref(flash.io)), 0)
                flash.fail_program = 0
                recovered = self.lib.boot_journal_next(C.byref(flash.io))
                self.assertGreater(recovered, first)

    def test_erase_failure_keeps_latest_counter(self):
        flash = Flash()
        for _ in range(128): self.lib.boot_journal_next(C.byref(flash.io))
        flash.fail_erase = True
        self.assertEqual(self.lib.boot_journal_next(C.byref(flash.io)), 0)
        flash.fail_erase = False
        self.assertEqual(self.lib.boot_journal_next(C.byref(flash.io)), 129)


if __name__ == '__main__': unittest.main()
