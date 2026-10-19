from __future__ import annotations

from collections import deque
import importlib.util
from pathlib import Path
import sys
import unittest

import numpy as np


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "ros2_to_lerobot_openarm.py"
SPEC = importlib.util.spec_from_file_location("ros2_to_lerobot_openarm_absolute", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class AbsoluteCommandSamplingTest(unittest.TestCase):
    def test_previous_is_zero_order_hold_and_never_uses_future_command(self) -> None:
        buffer = deque(
            [
                (100, np.asarray([1.0], dtype=np.float32)),
                (120, np.asarray([2.0], dtype=np.float32)),
                (140, np.asarray([3.0], dtype=np.float32)),
            ]
        )
        value, source_time = MODULE.OpenArmConverter.previous(buffer, 139)
        self.assertEqual(source_time, 120)
        np.testing.assert_array_equal(value, np.asarray([2.0], dtype=np.float32))

    def test_previous_requires_a_real_command_not_later_than_policy_tick(self) -> None:
        buffer = deque([(120, np.asarray([2.0], dtype=np.float32))])
        with self.assertRaisesRegex(ValueError, "no sample at/before"):
            MODULE.OpenArmConverter.previous(buffer, 119)

    def test_measured_state_interpolation_reports_real_bracket(self) -> None:
        buffer = deque(
            [
                (100, np.asarray([1.0], dtype=np.float32)),
                (140, np.asarray([3.0], dtype=np.float32)),
            ]
        )
        value, before, after = MODULE.OpenArmConverter.interpolation_bracket(buffer, 120)
        self.assertEqual((before, after), (100, 140))
        np.testing.assert_allclose(value, np.asarray([2.0], dtype=np.float32))
