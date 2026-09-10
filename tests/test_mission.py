import unittest

from drone_swarm.motion.geometry import Vec3
from drone_swarm.swarm.mission import Mission, State
from drone_swarm.utils.config import MissionConfig


class MissionTests(unittest.TestCase):
    def test_full_mission_tracks_targets_without_teleporting(self):
        mission = Mission(MissionConfig(transition_time=2.0))
        position = Vec3(0, 0, 0.5)
        mission.start(position, 0)
        observed_states = {mission.state}
        max_speed = 0
        for step in range(1, 3001):
            velocity = mission.update(position, step / 100)
            max_speed = max(max_speed, velocity.norm)
            position = position + velocity * 0.01
            observed_states.add(mission.state)
            if mission.state == State.COMPLETE:
                break
        self.assertEqual(mission.state, State.COMPLETE)
        self.assertEqual(mission.progress, 1)
        self.assertLess((position - Vec3(0, 0, 4)).norm, 0.07)
        self.assertLessEqual(max_speed, 1.5 + 1e-9)
        self.assertIn(State.TAKEOFF, observed_states)
        self.assertIn(State.MOVING, observed_states)

    def test_pause_freezes_trajectory_and_resume_excludes_paused_time(self):
        mission = Mission()
        p = Vec3(0, 0, 0.5)
        mission.start(p, 0)
        mission.update(p, 1)
        mission.command("pause", p, 1)
        self.assertEqual(mission.update(p, 100), Vec3())
        self.assertEqual(mission.elapsed, 1)
        mission.command("resume", p, 100)
        mission.update(p, 100.5)
        self.assertAlmostEqual(mission.elapsed, 1.5)

    def test_stop_and_restart_use_measured_position(self):
        mission = Mission()
        p = Vec3(1, 2, 3)
        mission.start(Vec3(), 0)
        mission.command("stop", p, 2)
        self.assertEqual(mission.update(p, 3), Vec3())
        mission.command("start", p, 3)
        self.assertEqual(mission.trajectory.start, p)

    def test_invalid_commands_are_rejected(self):
        mission = Mission()
        for command in ("resume", "pause", "reset", "bogus"):
            with self.assertRaises(ValueError):
                mission.command(command, Vec3(), 0)
        mission.start(Vec3(), 0)
        with self.assertRaises(ValueError):
            mission.start(Vec3(), 1)

    def test_clock_rewind_stops_motion(self):
        mission = Mission()
        mission.start(Vec3(), 3)
        self.assertEqual(mission.update(Vec3(), 2), Vec3())
        self.assertEqual(mission.state, State.ERROR)

    def test_failure_is_latched_until_explicit_stop(self):
        mission = Mission()
        mission.fail("No feedback")
        with self.assertRaises(ValueError):
            mission.start(Vec3(), 0)
        self.assertEqual(mission.update(Vec3(), 1), Vec3())

    def test_stationary_drone_times_out(self):
        mission = Mission()
        mission.start(Vec3(), 0)
        self.assertEqual(mission.update(Vec3(), 30), Vec3())
        self.assertEqual(mission.state, State.ERROR)


if __name__ == "__main__":
    unittest.main()
