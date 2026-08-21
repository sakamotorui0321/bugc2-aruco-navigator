import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from planner import astar, line_is_free, simplify_path


class PlannerTest(unittest.TestCase):
    def test_straight_path(self):
        grid = [[False] * 12 for _ in range(8)]
        path = simplify_path(grid, astar(grid, (1, 3), (10, 3)))
        self.assertEqual(path, [(1, 3), (10, 3)])

    def test_routes_around_wall(self):
        grid = [[False] * 14 for _ in range(10)]
        for y in range(1, 9):
            grid[y][7] = True
        path = astar(grid, (2, 5), (11, 5))
        self.assertTrue(path)
        self.assertTrue(all(not grid[y][x] for x, y in path))
        self.assertFalse(line_is_free(grid, (2, 5), (11, 5)))

    def test_no_path(self):
        grid = [[False] * 8 for _ in range(8)]
        for y in range(8):
            grid[y][4] = True
        self.assertEqual(astar(grid, (1, 3), (6, 3)), [])


if __name__ == "__main__":
    unittest.main()
