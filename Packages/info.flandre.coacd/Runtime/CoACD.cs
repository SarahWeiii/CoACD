using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
#if SAINTSFIELD_SAINTS_EDITOR_APPLY
using SaintsField.Playa;
#endif
#if UNITY_EDITOR
using UnityEditor;
using System.IO;
#endif
using UnityEngine;

//LL: noodled in Unity's V-HACD implementation to store mesh data neatly where the OG mesh resides
public unsafe class CoACD : MonoBehaviour
{
	[Serializable]
	public struct MeshInterface
	{
		public double* vertices_ptr;
		public ulong   vertices_count;
		public int*    triangles_ptr;
		public ulong   triangles_count;
	}
	[Serializable]
	public struct MeshArrayInterface : IDisposable
	{
		public MeshInterface* meshes_ptr;
		public ulong          meshes_count;

		public void Dispose()
		{
			if (meshes_ptr != null) {
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
	static extern MeshArrayInterface Run(ref MeshInterface mesh, double threshold, int max_convex_hull, int preprocess_mode, int prep_resolution, int sample_resolution,
																			int mcts_nodes, int mcts_iteration, int mcts_max_depth, bool pca, bool merge, uint seed);

	public enum PreprocessMode { Auto = 0, On = 1, Off = 2 }
	public enum LogLevel { Off, Info, Warn, Error, Critical }
	public MeshFilter target;
	public bool isTrigger,
							hideColliders = true;
	public PhysicMaterial physicMaterial;
	//struct 
	[Serializable]
	public unsafe struct Parameters
	{
		public static Parameters Init() => new Parameters() {
																													threshold = 0.05, preprocessMode = PreprocessMode.Auto, preprocessResolution = 50, sampleResolution = 2000,
																													mctsNodes = 20, mctsIteration    = 150, mctsMaxDepth = 3, pca = false, merge = true, maxConvexHull = -1, seed = 0
																												};

		[Range(0.01f, 1f)]
		[Tooltip("concavity threshold for terminating the decomposition")]
		public double threshold;
		[Tooltip("choose manifold preprocessing mode ('auto': automatically check input mesh manifoldness; 'on': force turn on the pre-processing; 'off': force turn off the pre-processing)")]
		public PreprocessMode preprocessMode;
		[Range(20, 100)]
		[Tooltip("resolution for manifold preprocess")]
		public int preprocessResolution;
		[Range(1000, 10000)]
		[Tooltip("sampling resolution for Hausdorff distance calculation")]
		public int sampleResolution;
		[Range(10, 40)]
		[Tooltip("max number of child nodes in MCTS")]
		public int mctsNodes;
		[Range(60, 2000)]
		[Tooltip("number of search iterations in MCTS")]
		public int mctsIteration;
		[Range(2, 7)]
		[Tooltip("max search depth in MCTS")]
		public int mctsMaxDepth;
		[Tooltip("flag to enable PCA pre-processing")]
		public bool pca;
		public bool merge;
		[Tooltip("max number of convex hulls generated, -1 for no limit")]
		public int maxConvexHull;
		[Tooltip("max # convex hulls in the result, -1 for no maximum limitation, works only when merge is enabled, default = -1 (may introduce convex hull with a concavity larger than the threshold)")]
		public uint seed;
	}
	public Parameters parameters = Parameters.Init();
	[SerializeField]
	CoACDColliderData _colliderData;
	[SerializeField]
	List<MeshCollider> _colliders;

	public List<Mesh> RunACD(Mesh mesh)
	{
		var unityV = mesh.vertices;
		var unityF = mesh.triangles;
		var v      = new double[mesh.vertexCount * 3];
		for (var i = 0; i < mesh.vertexCount; i++) {
			v[3 * i + 0] = unityV[i].x;
			v[3 * i + 1] = unityV[i].y;
			v[3 * i + 2] = unityV[i].z;
		}
		fixed (double* vptr = v) {
			fixed (int* fptr = unityF) {
				var mi = new MeshInterface() {vertices_ptr = vptr, vertices_count = (ulong) mesh.vertexCount, triangles_ptr = fptr, triangles_count = (ulong) (unityF.LongLength / 3)};
				using var res = Run(ref mi, parameters.threshold, parameters.maxConvexHull, (int) parameters.preprocessMode, parameters.preprocessResolution, parameters.sampleResolution,
					parameters.mctsNodes, parameters.mctsIteration, parameters.mctsMaxDepth, parameters.pca, parameters.merge, parameters.seed);
				var meshes = new List<Mesh>();
				for (ulong i = 0; i < res.meshes_count; i++) {
					var rmesh = new Mesh();
					var verts = new Vector3[res.meshes_ptr[i].vertices_count];
					var tris  = new int[res.meshes_ptr[i].triangles_count * 3];
					for (ulong j = 0; j < res.meshes_ptr[i].vertices_count; j++) {
						verts[j] = new Vector3((float) res.meshes_ptr[i].vertices_ptr[j * 3 + 0], (float) res.meshes_ptr[i].vertices_ptr[j * 3 + 1],
							(float) res.meshes_ptr[i].vertices_ptr[j * 3 + 2]);
					}
					for (ulong j = 0; j < res.meshes_ptr[i].triangles_count * 3; j++) { tris[j] = res.meshes_ptr[i].triangles_ptr[j]; }
					rmesh.SetVertices(verts);
					rmesh.SetTriangles(tris, 0);
					meshes.Add(rmesh);
				}
				return meshes;
			}
		}
	}

#if UNITY_EDITOR
	//called when contextmenu>reset !
	void Reset()
	{
		target = GetComponent<MeshFilter>();
	}

	// ###### noodled from V-HACD
#if SAINTSFIELD_SAINTS_EDITOR_APPLY
	[Button]
#endif
	void CalculateColliders()
	{
		var  parameters    = this.parameters;
		var  baseTransform = transform;
		Mesh mesh          = null;
		var  path          = "";
		EditorUtility.DisplayProgressBar("Calculating Colliders", "Discovering meshes...", 0.1f);
		var originalMeshes    = new List<Mesh>();
		var meshesToDecompose = new List<Mesh>();
		var transformsToCalc  = new List<Matrix4x4>();
		{
			if (transform.TryGetComponent<MeshFilter>(out var filter)) {
				{
					for (var j = 0; j < filter.sharedMesh.subMeshCount; j++) {
						meshesToDecompose.Add(ExtractSubmesh(filter.sharedMesh, j));
						transformsToCalc.Add(transform.transform.worldToLocalMatrix * filter.transform.localToWorldMatrix);
					}
					originalMeshes.Add(filter.sharedMesh);
					path = AssetDatabase.GetAssetPath(filter.sharedMesh);
				}
			}
		}
		EditorUtility.ClearProgressBar();
		EditorUtility.DisplayProgressBar("Calculating Colliders", "Combining meshes...", 0.3f);
		var decomposedMeshes = new List<Mesh>();
		{
			var c = 1;
			foreach (var meshToDecompose in meshesToDecompose) {
				EditorUtility.ClearProgressBar();
				EditorUtility.DisplayProgressBar("Calculating Colliders", $"Processing mesh... ({c++}/{meshesToDecompose.Count}) (this can take a while)",
					Mathf.Lerp(0.4f, 0.7f, Mathf.InverseLerp(1, meshesToDecompose.Count + 1, c)));
				var tempMeshes = RunACD(meshToDecompose);
				decomposedMeshes.AddRange(tempMeshes);
			}
		}
		EditorUtility.ClearProgressBar();
		if (decomposedMeshes.Count == 0) {
			EditorUtility.DisplayDialog("Error",
				$"The object you are trying to calculate colliders for did not compute any submeshes.\nTry modifying your quality parameters and try again", "Ok");
			return;
		}
		EditorUtility.DisplayProgressBar("Calculating Colliders", "Storing submeshes...", 0.8f);
		if (_colliderData == null) {
			path          = Path.ChangeExtension(path, null) + $"_Colliders.asset";
			path          = AssetDatabase.GenerateUniqueAssetPath(path);
			_colliderData = CoACDColliderData.CreateAsset(path, parameters, decomposedMeshes.ToArray(), originalMeshes.ToArray());
		} else { _colliderData.UpdateAsset(parameters, decomposedMeshes.ToArray(), originalMeshes.ToArray()); }
		AssetDatabase.SaveAssets();
		ValidateColliders(baseTransform);
		EditorUtility.ClearProgressBar();
		EditorGUIUtility.PingObject(_colliderData);
	}

	Mesh ExtractSubmesh(Mesh mesh, int submesh)
	{
		var newMesh    = new Mesh();
		var descriptor = mesh.GetSubMesh(submesh);
		newMesh.vertices = RangeSubset(mesh.vertices, descriptor.firstVertex, descriptor.vertexCount);
		var triangles = RangeSubset(mesh.triangles, descriptor.indexStart, descriptor.indexCount);
		for (var i = 0; i < triangles.Length; i++) { triangles[i] -= descriptor.firstVertex; }
		newMesh.triangles = triangles;
		if (mesh.normals != null && mesh.normals.Length == mesh.vertices.Length) { newMesh.normals = RangeSubset(mesh.normals, descriptor.firstVertex, descriptor.vertexCount); } else {
			newMesh.RecalculateNormals();
		}
		newMesh.Optimize();
		newMesh.OptimizeIndexBuffers();
		newMesh.RecalculateBounds();
		newMesh.name = mesh.name + $" Submesh {submesh}";
		return newMesh;
	}

	T[] RangeSubset<T>(T[] array, int startIndex, int length)
	{
		var subset = new T[length];
		Array.Copy(array, startIndex, subset, 0, length);
		return subset;
	}

	void ValidateColliders(Transform baseTransform)
	{
		if (_colliderData != null) {
			_colliders.RemoveAll(c => c == null);
			if (_colliders.Count != _colliderData.computedMeshes.Length) {
				if (_colliders.Count > _colliderData.computedMeshes.Length) {
					for (var i = _colliders.Count - 1; i >= _colliderData.computedMeshes.Length; i--) { DestroyImmediate(_colliders[i]); }
				} else {
					for (var i = _colliders.Count; i < _colliderData.computedMeshes.Length; i++) { _colliders.Add(baseTransform.gameObject.AddComponent<MeshCollider>()); }
				}
			}
			_colliders.RemoveAll(c => c == null);
			for (var i = 0; i < _colliders.Count; i++) {
				var mc = _colliders[i];
				mc.convex     = true;
				mc.sharedMesh = _colliderData.computedMeshes[i];
				mc.hideFlags  = hideColliders ? HideFlags.HideInInspector : HideFlags.None;
				mc.isTrigger  = isTrigger;
				mc.material   = physicMaterial;
			}
		} else {
			for (var i = 0; i < _colliders.Count; i++) {
				DestroyImmediate(_colliders[i]);
				i--;
			}
			_colliders.RemoveAll(c => c == null);
		}
	}
#endif
}