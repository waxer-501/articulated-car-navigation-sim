"""Fake backend regression for observed transient SWD read corruption; no USB IO."""
import ctypes
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
INCLUDE = ROOT/'.toolchain/usr/include/stlink'
FAKE = r'''
#include <stlink.h>
#include <usb.h>
#include <string.h>
static stlink_t target;
static int mode, calls, bodies;
void fake_mode(int x) { mode=x; calls=0; bodies=0; }
stlink_t *stlink_open_usb(enum ugly_loglevel v, enum connect_type c,
                        char serial[STLINK_SERIAL_BUFFER_SIZE], int frequency)
{
    (void)v; (void)c; (void)serial; (void)frequency;
    target.chip_id=0x412; target.flash_size=32768;
    return &target;
}
int stlink_exit_debug_mode(stlink_t *sl) { (void)sl; return 0; }
void stlink_close(stlink_t *sl) { (void)sl; }
int stlink_read_mem32(stlink_t *sl, uint32_t address, uint16_t length)
{
    ++calls;
    memset(sl->q_buf, 0, length);
    if (address >= 0x08000000 && address < 0x08008000) {
        memset(sl->q_buf, 255, length);
        if (mode==3 && calls==1) sl->q_buf[16]=0;
        if (mode==4 && (calls&1)) sl->q_buf[16]=0;
    } else if (length==256) {
        ++bodies;
        if (mode==1 && bodies==2) sl->q_buf[40]=0xab;
        if (mode==2 && bodies==2) sl->q_buf[12]=4;
    }
    return 0;
}
'''


@unittest.skipUnless((INCLUDE/'stlink.h').exists(), 'Requires pinned local stlink development headers')
class ReadConsistencyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix='stlink-fake-')
        root = Path(cls.temp.name)
        (root/'fake.c').write_text(FAKE)
        subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-shared', '-fPIC',
                        '-I'+str(INCLUDE), '-I'+str(ROOT/'.toolchain/usr/include/libusb-1.0'),
                        '-I/usr/include/libusb-1.0', str(ROOT/'tools/stlink_readonly.c'),
                        str(root/'fake.c'), '-o', str(root/'test.so')], check=True)
        cls.lib = ctypes.CDLL(str(root/'test.so'))
        cls.lib.capture_open.argtypes = [ctypes.c_char_p]; cls.lib.capture_open.restype = ctypes.c_void_p
        cls.lib.capture_snapshot.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint8)]
        cls.lib.capture_flash_backup.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint8)]
        cls.handle = cls.lib.capture_open(b'fake')

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def test_identical_snapshot_accepted(self):
        self.lib.fake_mode(0)
        self.assertEqual(self.lib.capture_snapshot(self.handle, (ctypes.c_uint8*256)()), 1)

    def test_same_sequence_but_corrupted_body_rejected(self):
        self.lib.fake_mode(1)
        before = self.lib.capture_unstable_reads()
        self.assertEqual(self.lib.capture_snapshot(self.handle, (ctypes.c_uint8*256)()), 0)
        self.assertEqual(self.lib.capture_unstable_reads()-before, 1)

    def test_normal_publication_change_not_counted_as_corruption(self):
        self.lib.fake_mode(2)
        before = self.lib.capture_unstable_reads()
        self.assertEqual(self.lib.capture_snapshot(self.handle, (ctypes.c_uint8*256)()), 0)
        self.assertEqual(self.lib.capture_unstable_reads(), before)

    def test_transient_flash_read_not_saved_as_real_content(self):
        self.lib.fake_mode(3)
        output = (ctypes.c_uint8*32768)()
        self.assertEqual(self.lib.capture_flash_backup(self.handle, output), 0)
        self.assertEqual(bytes(output), b'\xff'*32768)

    def test_unstable_flash_read_fails_instead_of_using_last(self):
        self.lib.fake_mode(4)
        self.assertEqual(self.lib.capture_flash_backup(self.handle, (ctypes.c_uint8*32768)()), -1)


if __name__ == '__main__':
    unittest.main()
