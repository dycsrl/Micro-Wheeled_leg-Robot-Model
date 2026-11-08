"""舵机承重静差红绿回归；不涉及Viewer、硬件或控制器。"""
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from check_servo_hold import bench, statics

class ServoHoldTests(unittest.TestCase):
    def test_loaded_hold(self):
        result, _ = bench()
        self.assertTrue(result['pass'], result)

    def test_closed_chain_statics(self):
        result = statics(-.2)
        self.assertLess(result['virtual_work_error_nm'], 1e-7)

    def test_independent_mapping_unloaded(self):
        for delta in ((-.04, 0), (0, -.04), (-.03, .03)):
            with self.subTest(delta=delta):
                result, _ = bench(delta=delta, unloaded=True)
                self.assertTrue(result['pass'], result)

if __name__ == '__main__':
    unittest.main()
