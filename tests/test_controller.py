import pathlib
import sys
import time
import unittest

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import main


def marker(marker_id, x, y):
    corners = np.array(
        [[x - 1, y - 1], [x + 1, y - 1], [x + 1, y + 1], [x - 1, y + 1]],
        dtype=np.float32,
    )
    return main.Marker(marker_id, corners, (x, y), 0.0, time.monotonic())


class ControllerTest(unittest.TestCase):
    def setUp(self):
        self.config = main.load_config(pathlib.Path(__file__).parents[1] / "config.json")

    def test_nearest_other_robot(self):
        robot = marker(self.config["self_id"], 100.0, 100.0)
        markers = {
            robot.marker_id: robot,
            10: marker(10, 125.0, 100.0),
            2: marker(2, 200.0, 100.0),
            0: marker(0, 105.0, 100.0),
        }
        distance, marker_id = main.nearest_other_robot_cm(robot, markers, self.config)
        self.assertAlmostEqual(distance, 5.0)
        self.assertEqual(marker_id, 10)

    def test_close_robot_threshold_requests_stop(self):
        robot = marker(self.config["self_id"], 100.0, 100.0)
        threat = marker(10, 150.0, 100.0)
        distance, marker_id = main.nearest_other_robot_cm(
            robot, {robot.marker_id: robot, threat.marker_id: threat}, self.config
        )
        self.assertLessEqual(distance, self.config["other_robot_stop_distance_cm"])
        self.assertEqual(marker_id, 10)


if __name__ == "__main__":
    unittest.main()
