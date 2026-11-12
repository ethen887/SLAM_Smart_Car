import copy, math, unittest
from pathlib import Path
from run_demo import build
from simulation import defaults, invalid_segments, scan, Simulation
import numpy as np

class Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.lib=build()
    def test_scan_and_collision(self):
        s=defaults();self.assertEqual(invalid_segments(s),[])
        rays=scan(s,[-1.8,-1.7,0],np.random.default_rng(1));self.assertEqual(len(rays),180);self.assertTrue(any(r[2] for r in rays))
        s['waypoints']=[[-1,0],[1,0]];self.assertEqual(invalid_segments(s),[0])
        with self.assertRaises(ValueError):Simulation(s,self.lib)
    def test_map_history_and_odometry(self):
        s=defaults();s['waypoints']=[[-1.8,-1.7],[-.8,-1.7]];s['odom']['scale']=.85
        sim=Simulation(s,self.lib);first=sim.history[0]['grid'].copy()
        for _ in range(100):
            if sim.finished:break
            sim.step()
        self.assertTrue(sim.finished);self.assertAlmostEqual(sim.real[0],-.8,places=5);self.assertAlmostEqual(sim.estimate[0],-1.8+.85,places=4)
        self.assertGreater(len(sim.history),2);self.assertTrue(np.array_equal(first,sim.history[0]['grid']));self.assertGreater(sim.history[-1]['grid']['seen'].sum(),0)
    def test_deterministic(self):
        s=defaults();a=Simulation(s,self.lib);b=Simulation(copy.deepcopy(s),self.lib)
        for _ in range(20):a.step();b.step()
        self.assertTrue(np.array_equal(a.history[-1]['grid'],b.history[-1]['grid']))
if __name__=='__main__':unittest.main()
