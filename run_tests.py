import trimesh
import os
import glob
import coacd
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


for f in glob.glob("examples/*.obj"):
    setattr(TestExamples, f'test_{os.path.splitext(os.path.basename(f))[0].lower()}', TestExamples.single(f))


if __name__ == '__main__':
    unittest.main()
