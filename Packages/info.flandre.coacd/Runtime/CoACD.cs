using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;

public unsafe class CoACD : MonoBehaviour
{
    [Serializable]
    public struct MeshInterface
    {
        public double* vertices_ptr;
        public ulong vertices_count;
        public int* triangles_ptr;
        public ulong triangles_count;
    }
    [Serializable]
    public struct MeshArrayInterface: IDisposable
    {
        public MeshInterface* meshes_ptr;
        public ulong meshes_count;

        public void Dispose()
        {
            if (meshes_ptr != null)
            {
                Free(this);
                meshes_ptr = null;
            }
        }
    }
    [DllImport("lib_coacd", CallingConvention = CallingConvention.Cdecl, EntryPoint = "CoACD_setLogLevel")]
    static extern void SetLogLevel([MarshalAs(UnmanagedType.LPStr)] string level);

    [DllImport("lib_coacd", CallingConvention = CallingConvention.Cdecl, EntryPoint = "CoACD_freeMeshArray")]
    static extern void Free(MeshArrayInterface array);

    [DllImport("lib_coacd", CallingConvention = CallingConvention.Cdecl, EntryPoint = "CoACD_run")]
    static extern MeshArrayInterface Run(
        ref MeshInterface mesh, double threshold,
        int max_convex_hull, int preprocess_mode,
        int prep_resolution, int sample_resolution,
        int mcts_nodes, int mcts_iteration,
        int mcts_max_depth, bool pca, bool merge,
        uint seed
    );

    public enum PreprocessMode
    {
        Auto = 0,
        On = 1,
        Off = 2,
    }

    public enum LogLevel
    {
        Off,
        Info,
        Warn,
        Error,
        Critical
    }

    public MeshFilter target;

    [Range(0.01f, 1f)]
    public double threshold = 0.05;
    public PreprocessMode preprocessMode = PreprocessMode.Auto;
    [Range(20, 100)]
    public int preprocessResolution = 50;
    [Range(1000, 10000)]
    public int sampleResolution = 2000;
    [Range(10, 40)]
    public int mctsNodes = 20;
    [Range(60, 2000)]
    public int mctsIteration = 150;
    [Range(2, 7)]
    public int mctsMaxDepth = 3;
    public bool pca = false;
    public bool merge = true;
    public int maxConvexHull = -1;
    public uint seed = 0;

    public List<Mesh> RunACD(Mesh mesh)
    {
        var unityV = mesh.vertices;
        var unityF = mesh.triangles;
        var v = new double[mesh.vertexCount * 3];
        for (int i = 0; i < mesh.vertexCount; i++)
        {
            v[3 * i + 0] = unityV[i].x;
            v[3 * i + 1] = unityV[i].y;
            v[3 * i + 2] = unityV[i].z;
        }
        fixed (double* vptr = v) fixed (int* fptr = unityF)
        {
            MeshInterface mi = new()
            {
                vertices_ptr = vptr,
                vertices_count = (ulong)mesh.vertexCount,
                triangles_ptr = fptr,
                triangles_count = (ulong)(unityF.LongLength / 3)
            };
            using var res = Run(
                ref mi, threshold, maxConvexHull,
                (int)preprocessMode, preprocessResolution,
                sampleResolution, mctsNodes, mctsIteration, mctsMaxDepth, pca, merge, seed
            );
            var meshes = new List<Mesh>();
            for (ulong i = 0; i < res.meshes_count; i++)
            {
                var rmesh = new Mesh();
                var verts = new Vector3[res.meshes_ptr[i].vertices_count];
                var tris = new int[res.meshes_ptr[i].triangles_count * 3];
                for (ulong j = 0; j < res.meshes_ptr[i].vertices_count; j++)
                {
                    verts[j] = new(
                        (float)res.meshes_ptr[i].vertices_ptr[j * 3 + 0],
                        (float)res.meshes_ptr[i].vertices_ptr[j * 3 + 1],
                        (float)res.meshes_ptr[i].vertices_ptr[j * 3 + 2]
                    );
                }
                for (ulong j = 0; j < res.meshes_ptr[i].triangles_count * 3; j++)
                {
                    tris[j] = res.meshes_ptr[i].triangles_ptr[j];
                }
                rmesh.SetVertices(verts);
                rmesh.SetTriangles(tris, 0);
                meshes.Add(rmesh);
            }
            return meshes;
        }
    }

    void Reset()
    {
        target = GetComponent<MeshFilter>();
    }

    [ContextMenu("Generate Collision Meshes")]
    public void GenerateCollisionMeshes()
    {
        GenerateCollisionMeshes(target);
    }

    [ContextMenu("Generate Collision Meshes For Hierarchy")]
    public void GenerateCollisionMeshesForHierarchy()
    {
        foreach (var item in GetComponentsInChildren<MeshFilter>())
        {
            GenerateCollisionMeshes(item);
        }
    }

    public void GenerateCollisionMeshes(MeshFilter target)
    {
        var baseMesh = target.sharedMesh;
        var meshes = RunACD(baseMesh);
        var empty = new GameObject("Collision");
        empty.transform.SetParent(target.transform);
        empty.transform.localPosition = Vector3.zero;
        empty.transform.localRotation = Quaternion.identity;
        empty.transform.localScale = Vector3.one;
        foreach (var item in meshes)
        {
            var go = new GameObject("Collider " + System.Guid.NewGuid().ToString().Substring(0, 8), typeof(MeshCollider), typeof(MeshFilter));
            var col = go.GetComponent<MeshCollider>();
            var filt = go.GetComponent<MeshFilter>();
            filt.sharedMesh = item;
            col.sharedMesh = item;
            col.convex = true;
            go.transform.SetParent(empty.transform);
            go.transform.localPosition = Vector3.zero;
            go.transform.localRotation = Quaternion.identity;
            go.transform.localScale = Vector3.one;
        }
    }
}
