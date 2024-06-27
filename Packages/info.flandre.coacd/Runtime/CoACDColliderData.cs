using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

//LL: stored mesh assets and decomposition parameters, derived from Unity's V-HACD implementation
public class CoACDColliderData : ScriptableObject
{
	public CoACD.Parameters parameters;
	public Mesh[]           baseMeshes     = new Mesh[0];
	public Mesh[]           computedMeshes = new Mesh[0];
#if UNITY_EDITOR

	public static CoACDColliderData CreateAsset(string path, CoACD.Parameters parameters, Mesh[] meshes, Mesh[] baseMeshes)
	{
		var obj = CreateInstance<CoACDColliderData>();
		AssetDatabase.CreateAsset(obj, path);
		var c = 0;
		foreach (var mesh in meshes) {
			mesh.name = $"Computed Mesh {c++}";
			AssetDatabase.AddObjectToAsset(mesh, obj);
		}
		obj.parameters     = parameters;
		obj.baseMeshes     = baseMeshes;
		obj.computedMeshes = meshes;
		return obj;
	}

	public void UpdateAsset(CoACD.Parameters parameters, Mesh[] meshes, Mesh[] baseMeshes)
	{
		foreach (var mesh in computedMeshes) { AssetDatabase.RemoveObjectFromAsset(mesh); }
		computedMeshes = null;
		var c = 0;
		foreach (var mesh in meshes) {
			mesh.name = $"Computed Mesh {c++}";
			AssetDatabase.AddObjectToAsset(mesh, this);
		}
		this.parameters = parameters;
		this.baseMeshes = baseMeshes;
		computedMeshes  = meshes;
	}

#endif
}