import json
import pathlib
import socket
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import main


class UdpLinkTest(unittest.TestCase):
    def setUp(self):
        self.robot_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.robot_socket.bind(("127.0.0.1", 0))
        self.robot_socket.settimeout(1.0)
        self.robot_port = self.robot_socket.getsockname()[1]
        self.sender = main.UdpSender(
            {
                "robot_id": 4,
                "robot_ip": "127.0.0.1",
                "robot_port": self.robot_port,
                "probe_hz": 2.0,
                "status_timeout_ms": 1500,
                "send_hz": 20.0,
                "ttl_ms": 350,
            },
            False,
        )

    def tearDown(self):
        self.sender.enabled = False
        self.sender.close()
        self.robot_socket.close()

    def receive_json(self):
        payload, source = self.robot_socket.recvfrom(1024)
        return json.loads(payload.decode("utf-8")), source

    def reply_status(self, source, robot_id=4, armed=True):
        status = {
            "v": 1,
            "type": "status",
            "robot_id": robot_id,
            "ack_seq": 1,
            "armed": armed,
            "wifi": True,
            "udp": True,
            "command_active": False,
            "state": "ARMED WAIT UDP",
            "ip": "127.0.0.1",
        }
        self.robot_socket.sendto(json.dumps(status).encode("utf-8"), source)

    def test_probe_status_and_terminal_a_start(self):
        self.sender.send_probe(force=True)
        probe, pc_address = self.receive_json()
        self.assertEqual(probe["type"], "probe")
        self.assertEqual(probe["robot_id"], 4)

        self.reply_status(pc_address)
        self.sender.poll_status()
        self.assertTrue(self.sender.robot_connected)
        self.assertTrue(self.sender.robot_armed)

        command = main.MotionCommand(mode="velocity", forward=0.25, reason="test")
        should_quit = main.handle_control_key("a", self.sender, command)
        self.assertFalse(should_quit)
        self.assertTrue(self.sender.enabled)

        self.sender.send(command, pwm_limit=22, force=True)
        motion, _ = self.receive_json()
        self.assertEqual(motion["type"], "motion")
        self.assertEqual(motion["robot_id"], 4)
        self.assertEqual(motion["mode"], "velocity")

    def test_wrong_robot_status_does_not_connect(self):
        self.sender.send_probe(force=True)
        _, pc_address = self.receive_json()
        self.reply_status(pc_address, robot_id=5)
        self.sender.poll_status()
        self.assertFalse(self.sender.robot_connected)

    def test_terminal_a_is_rejected_without_robot_reply(self):
        command = main.MotionCommand(mode="velocity", forward=0.25, reason="test")
        should_quit = main.handle_control_key("a", self.sender, command)
        self.assertFalse(should_quit)
        self.assertFalse(self.sender.enabled)

    def test_auto_discovery_uses_status_source_ip(self):
        auto_sender = main.UdpSender(
            {
                "robot_id": 4,
                "robot_ip": "auto",
                "robot_port": self.robot_port,
                "discovery_broadcasts": ["127.0.0.1"],
                "probe_hz": 2.0,
                "status_timeout_ms": 1500,
                "send_hz": 20.0,
                "ttl_ms": 350,
            },
            False,
        )
        try:
            auto_sender.send_probe(force=True)
            probe, pc_address = self.receive_json()
            self.assertEqual(probe["robot_id"], 4)
            self.reply_status(pc_address)
            auto_sender.poll_status()
            self.assertTrue(auto_sender.robot_connected)
            self.assertEqual(auto_sender.resolved_address[0], "127.0.0.1")
        finally:
            auto_sender.close()


if __name__ == "__main__":
    unittest.main()
