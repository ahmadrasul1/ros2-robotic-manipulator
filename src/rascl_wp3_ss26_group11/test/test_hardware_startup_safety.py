from __future__ import annotations

import importlib
import sys
import types
import unittest
from pathlib import Path


SCRIPTS = Path(__file__).resolve().parents[2] / "rascl_hardware_interface" / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

# The tests exercise pure startup-selection logic without EtherCAT hardware.
sys.modules.setdefault(
    "pysoem",
    types.SimpleNamespace(
        Master=object,
        PREOP_STATE=0x02,
        SAFEOP_STATE=0x04,
        OP_STATE=0x08,
        INIT_STATE=0x01,
        NONE_STATE=0x00,
    ),
)

pysoem_bridge = importlib.import_module("pysoem_bridge")


class FakeSlave:
    def __init__(self) -> None:
        self.writes: list[tuple[int, int, bytes]] = []

    def sdo_write(self, index: int, subindex: int, value: bytes) -> None:
        self.writes.append((index, subindex, value))


class StartupMotionSelectionTests(unittest.TestCase):
    def make_controller(self):
        controller = pysoem_bridge.RasclRobotController.__new__(
            pysoem_bridge.RasclRobotController
        )
        controller.slaves = [FakeSlave() for _ in range(4)]
        controller.position_units_per_output_revolution = [1000.0] * 4
        controller.homing_config = {
            "homing": {
                "pick_ready_pose": {
                    "shoulder_joint": 0.0,
                    "upperarm_joint": 0.0,
                    "lowerarm_joint": 0.0,
                    "end_effector_joint": 0.0,
                },
                "pick_ready_velocity": 200,
                "pick_ready_acceleration": 20,
            }
        }
        return controller

    def test_pick_ready_excludes_unreferenced_end_effector(self):
        controller = self.make_controller()
        captured: dict[str, object] = {}

        def fake_move_joints(positions, **kwargs):
            captured["positions"] = positions
            captured.update(kwargs)

        controller.move_joints = fake_move_joints
        controller.move_to_pick_ready(include_end_effector=False)

        self.assertEqual(captured["joint_indices"], [0, 1, 2])
        self.assertEqual(len(captured["positions"]), 4)

    def test_move_joints_only_enables_selected_axes(self):
        controller = self.make_controller()
        enabled: list[int] = []
        controller.enable_drive = lambda _slave, index, _mode: enabled.append(index)
        controller.configure_profile_position_motion = lambda *_args, **_kwargs: None
        controller.get_actual_position_counts = lambda _slave: 0
        controller.counts_to_radians = lambda _counts, _index: 0.0
        controller.radians_to_counts = lambda _radians, _index: 0
        controller.get_status_word = lambda _slave: 0
        controller._check_profile_position_status = lambda *_args, **_kwargs: None

        controller.move_joints([0.0, 0.0, 0.0, 1.5], joint_indices=[0, 1, 2])

        self.assertEqual(enabled, [0, 1, 2])
        self.assertEqual(controller.slaves[3].writes, [])


if __name__ == "__main__":
    unittest.main()
