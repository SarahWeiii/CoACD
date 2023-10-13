import trimesh
import os
import glob
import coacd
import numpy
import unittest


class TestExamples(unittest.TestCase):
    @staticmethod
    def single(input_file):

        def _test(self: 'TestExamples'):
            coacd.set_log_level("warn")
            mesh = trimesh.load(input_file, force="mesh")
            mesh = coacd.Mesh(mesh.vertices, mesh.faces)
            self.assertLessEqual(len(coacd.run_coacd(mesh)), 1000)

        return _test

    def test_deterministic(self):
        mesh = trimesh.load('examples/SnowFlake.obj', force="mesh")
        coacd.set_log_level("debug")
        mesh = coacd.Mesh(mesh.vertices, mesh.faces)
        acd1 = coacd.run_coacd(mesh)
        acd2 = coacd.run_coacd(mesh)
        self.assertEqual(len(acd1), len(acd2))
        for (v1, t1), (v2, t2) in zip(acd1, acd2):
            self.assertTrue(numpy.allclose(v1, v2))
            self.assertTrue(numpy.allclose(t1, t2))


for f in glob.glob("examples/*.obj"):
    setattr(TestExamples, f'test_{os.path.splitext(os.path.basename(f))[0].lower()}', TestExamples.single(f))


if __name__ == '__main__':
    unittest.main()
