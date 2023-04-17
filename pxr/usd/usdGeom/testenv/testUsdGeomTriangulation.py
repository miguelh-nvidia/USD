#!/pxrpythonsubst
#
# pylint: disable=range-builtin-not-iterating

from __future__ import print_function
from pxr import Usd, UsdGeom, Vt
import unittest


class testUsdGeomTriangulation(unittest.TestCase):

    def test_Concave(self):
        stage = Usd.Stage.Open('Concave.usda')
        prim = stage.GetPrimAtPath("/root/Concave")
        mesh = UsdGeom.Mesh(prim)
        result = mesh.FanTriangulate()
        self.assertTrue(result)
        self.assertEqual(mesh.GetFaceVertexCountsAttr().Get(), Vt.IntArray([4]))
        self.assertEqual(mesh.GetFaceVertexIndicesAttr().Get(), Vt.IntArray([2, 3, 1, 0]))

    def test_Convex(self):
        stage = Usd.Stage.Open('Convex.usda')
        prim = stage.GetPrimAtPath("/root/Convex")
        mesh = UsdGeom.Mesh(prim)
        result = mesh.FanTriangulate()
        self.assertTrue(result)
        self.assertEqual(mesh.GetFaceVertexCountsAttr().Get(), Vt.IntArray([4]))
        self.assertEqual(mesh.GetFaceVertexIndicesAttr().Get(), Vt.IntArray([0, 2, 3, 1]))

    def test_Faces3D(self):
        stage = Usd.Stage.Open('Faces3D.usda')
        prim = stage.GetPrimAtPath("/root/Faces3D")
        mesh = UsdGeom.Mesh(prim)
        result = mesh.FanTriangulate()
        self.assertTrue(result)
        self.assertEqual(mesh.GetFaceVertexCountsAttr().Get(), Vt.IntArray(
            [6, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 6]))
        self.assertEqual(mesh.GetFaceVertexIndicesAttr().Get(), Vt.IntArray(
            [5, 4, 3, 2, 1, 0, 0, 1, 7, 6, 1, 2, 8, 7, 2, 3, 9, 8, 3, 4, 10, 9, 4, 5, 11, 10, 5, 0, 6, 11, 6, 7, 13,
             12, 7, 8, 14, 13, 8, 9, 15, 14, 9, 10, 16, 15, 10, 11, 17, 16, 11, 6, 12, 17, 12, 13, 19, 18, 13, 14, 20,
             19, 14, 15, 21, 20, 15, 16, 22, 21, 16, 17, 23, 22, 17, 12, 18, 23, 18, 19, 25, 24, 19, 20, 26, 25, 20,
             21, 27, 26, 21, 22, 28, 27, 22, 23, 29, 28, 23, 18, 24, 29, 29, 24, 25, 26, 27, 28]))

    def test_SimplePolygon(self):
        stage = Usd.Stage.Open('SimplePolygon.usda')
        prim = stage.GetPrimAtPath("/root/SimplePolygon")
        mesh = UsdGeom.Mesh(prim)
        result = mesh.FanTriangulate()
        self.assertTrue(result)
        self.assertEqual(mesh.GetFaceVertexCountsAttr().Get(), Vt.IntArray([3, 3, 3, 3, 3, 3]))
        self.assertEqual(mesh.GetFaceVertexIndicesAttr().Get(), Vt.IntArray([7, 0, 1, 7, 1, 2, 2, 3, 4, 4, 5, 6, 2, 4, 6, 7, 2, 6]))

    def test_StarShaped(self):
        stage = Usd.Stage.Open('StarShaped.usda')
        prim = stage.GetPrimAtPath("/root/StarShaped")
        mesh = UsdGeom.Mesh(prim)
        result = mesh.FanTriangulate()
        self.assertTrue(result)
        self.assertEqual(mesh.GetFaceVertexCountsAttr().Get(), Vt.IntArray([8]))
        self.assertEqual(mesh.GetFaceVertexIndicesAttr().Get(), Vt.IntArray([0, 1, 2, 3, 4, 5, 6, 7]))


if __name__ == "__main__":
    unittest.main()
