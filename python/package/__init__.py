import numpy as np
import os

import ctypes
from ctypes import (
    Structure,
    POINTER,
    c_void_p,
    c_uint64,
    c_double,
    c_int,
    c_bool,
    c_uint,
    c_char_p,
)


_lib_files = os.listdir(os.path.dirname(os.path.abspath(__file__)))
for _file in _lib_files:
    if _file.startswith("lib_coacd"):
        _lib = ctypes.CDLL(os.path.join(os.path.dirname(os.path.abspath(__file__)), _file))


class CoACD_Mesh(ctypes.Structure):
    _fields_ = [
        ("vertices_ptr", POINTER(c_double)),
        ("vertices_count", c_uint64),
        ("triangles_ptr", POINTER(c_int)),
        ("triangles_count", c_uint64),
    ]


class CoACD_MeshArray(ctypes.Structure):
    _fields_ = [
        ("meshes_ptr", POINTER(CoACD_Mesh)),
        ("meshes_count", c_uint64),
    ]


_lib.CoACD_setLogLevel.argtypes = [c_char_p]
_lib.CoACD_setLogLevel.restype = None

_lib.CoACD_freeMeshArray.argtypes = [CoACD_MeshArray]
_lib.CoACD_freeMeshArray.restype = None

_lib.CoACD_run.argtypes = [
    POINTER(CoACD_Mesh),
    c_double,
    c_int,
    c_int,
    c_int,
    c_int,
    c_int,
    c_int,
    c_int,
    c_bool,
    c_bool,
    c_bool,
    c_int,
    c_bool,
    c_double,
    c_int,
    c_uint,
]
_lib.CoACD_run.restype = CoACD_MeshArray


class Mesh:
    def __init__(
        self,
        vertices=np.zeros((0, 3), dtype=np.double),
        indices=np.zeros((0, 3), dtype=np.int32),
    ):
        self.vertices = np.ascontiguousarray(vertices, dtype=np.double)
        self.indices = np.ascontiguousarray(indices, dtype=np.int32)
        assert len(vertices.shape) == 2 and vertices.shape[1] == 3
        assert len(indices.shape) == 2 and indices.shape[1] == 3


def run_coacd(
    mesh: Mesh,
    threshold: float = 0.05,
    max_convex_hull: int = -1,
    preprocess_mode: str = "auto",
    preprocess_resolution: int = 50,
    resolution: int = 2000,
    mcts_nodes: int = 20,
    mcts_iterations: int = 150,
    mcts_max_depth: int = 3,
    pca: int = False,
    merge: bool = True,
    decimate: bool = False,
    max_ch_vertex: int = 256,
    extrude: bool = False,
    extrude_margin: float = 0.01,
    apx_mode: str = "ch",
    seed: int = 0,
):
    vertices = np.ascontiguousarray(mesh.vertices, dtype=np.double)
    indices = np.ascontiguousarray(mesh.indices, dtype=np.int32)
    assert len(vertices.shape) == 2 and vertices.shape[1] == 3
    assert len(indices.shape) == 2 and indices.shape[1] == 3

    mesh = CoACD_Mesh()

    mesh.vertices_ptr = ctypes.cast(
        vertices.__array_interface__["data"][0], POINTER(c_double)
    )
    mesh.vertices_count = vertices.shape[0]

    mesh.triangles_ptr = ctypes.cast(
        indices.__array_interface__["data"][0], POINTER(c_int)
    )
    mesh.triangles_count = indices.shape[0]

    if preprocess_mode == "on":
        pm = 1
    elif preprocess_mode == "off":
        pm = 2
    else:
        pm = 0

    if apx_mode == "ch":
        apx = 0
    elif apx_mode == "box":
        apx = 1

    mesh_array = _lib.CoACD_run(
        mesh,
        threshold,
        max_convex_hull,
        pm,
        preprocess_resolution,
        resolution,
        mcts_nodes,
        mcts_iterations,
        mcts_max_depth,
        pca,
        merge,
        decimate,
        max_ch_vertex,
        extrude,
        extrude_margin,
        apx,
        seed,
    )

    meshes = []
    for i in range(mesh_array.meshes_count):
        mesh = mesh_array.meshes_ptr[i]
        vertices = np.ctypeslib.as_array(
            mesh.vertices_ptr, (mesh.vertices_count, 3)
        ).copy()
        indices = np.ctypeslib.as_array(
            mesh.triangles_ptr, (mesh.triangles_count, 3)
        ).copy()
        meshes.append([vertices, indices])

    _lib.CoACD_freeMeshArray(mesh_array)
    return meshes


def set_log_level(level: str):
    level = level.encode("utf-8")
    _lib.CoACD_setLogLevel(level)


if __name__ == "__main__":
    import trimesh

    set_log_level("info")

    mesh = trimesh.load("out.obj")
    mesh = Mesh(mesh.vertices, mesh.faces)
    result = run_coacd(mesh)
    mesh_parts = []
    for vs, fs in result:
        mesh_parts.append(trimesh.Trimesh(vs, fs))

    scene = trimesh.Scene()
    np.random.seed(0)
    for p in mesh_parts:
        p.visual.vertex_colors[:, :3] = (np.random.rand(3) * 255).astype(np.uint8)
        scene.add_geometry(p)
    scene.export("decomposed.obj")
