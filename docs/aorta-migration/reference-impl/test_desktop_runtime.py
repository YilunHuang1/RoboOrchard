import subprocess
import unittest
from pathlib import Path

import flatbuffers
import single_aorta
import take_over_aorta
from aorta.services.arm.ArmTriggerResponse import ArmTriggerResponse

ROOT = Path(__file__).resolve().parents[3]
TELEOP = ROOT / "projects" / "HoloBrain" / "teleop"


class _Responder:
    def __init__(self):
        self.payload = None

    def reply(self, fill):
        builder = flatbuffers.Builder(256)
        builder.Finish(fill(builder, 0))
        self.payload = bytes(builder.Output())


class TriggerResponseTest(unittest.TestCase):
    def _assert_response(self, reply):
        responder = _Responder()
        reply(responder, True, "ok")
        message = ArmTriggerResponse.GetRootAs(responder.payload, 0)
        self.assertEqual(message.Status(), 0)
        self.assertEqual(message.Message().decode(), "ok")

    def test_muxer_trigger_response_uses_response_bindings(self):
        self._assert_response(take_over_aorta.TakeOverMuxerNode._reply)

    def test_single_trigger_response_uses_response_bindings(self):
        self._assert_response(
            lambda responder, ok, message: (
                single_aorta.PiperSingleControlNode._reply_trigger(
                    None, responder, ok, message
                )
            )
        )


class DesktopWrapperTest(unittest.TestCase):
    def test_shell_wrappers_exist_and_parse(self):
        for name in ("start_aorta_master.sh", "stop_aorta_master.sh"):
            script = TELEOP / name
            self.assertTrue(script.is_file(), name)
            subprocess.run(["bash", "-n", str(script)], check=True)

    def test_control_helper_exists_and_compiles(self):
        helper = TELEOP / "aorta_control.py"
        self.assertTrue(helper.is_file())
        subprocess.run(
            ["python3", "-m", "py_compile", str(helper)],
            check=True,
        )

    def test_start_defaults_to_safe_master_mode(self):
        script = (TELEOP / "start_aorta_master.sh").read_text()
        self.assertIn("--auto-enable-arm-ctrl false", script)
        self.assertIn("AORTA_LOCAL_ZENOH_PORT:-17447", script)
        self.assertIn("127.0.0.1:${LOCAL_ZENOH_PORT}", script)

    def test_failed_start_always_cleans_up_and_rejects_busy_port(self):
        script = (TELEOP / "start_aorta_master.sh").read_text()
        self.assertIn("trap cleanup_failed_start EXIT", script)
        self.assertIn("Zenoh tunnel port is already in use", script)
        self.assertIn('pid_is_running "${RUNTIME_DIR}/tunnel.pid"', script)

    def test_takeover_requires_all_managed_processes(self):
        script = (TELEOP / "trigger_aorta_takeover.sh").read_text()
        self.assertIn("for name in tunnel muxer master", script)
        self.assertIn('kill -0 "$(cat "${pid_file}")"', script)


if __name__ == "__main__":
    unittest.main()
