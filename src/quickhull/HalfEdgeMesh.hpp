#include "Structs/Mesh.hpp"

#ifndef HalfEdgeMesh_h
#define HalfEdgeMesh_h

/*
 * Implementation of the 3D QuickHull algorithm by Antti Kuukka
 *
 * No copyrights. What follows is 100% Public Domain.
 *
 *
 *
 * INPUT:  a list of points in 3D space (for example, vertices of a 3D mesh)
 *
 * OUTPUT: a ConvexHull object which provides vertex and index buffers of the generated convex hull as a triangle mesh.
 *
 *
 *
 * The implementation is thread-safe if each thread is using its own QuickHull object.
 *
 *
 * SUMMARY OF THE ALGORITHM:
 *         - Create initial simplex (tetrahedron) using extreme points. We have four faces now and they form a convex mesh M.
 *         - For each point, assign them to the first face for which they are on the positive side of (so each point is assigned to at most
 *           one face). Points inside the initial tetrahedron are left behind now and no longer affect the calculations.
 *         - Add all faces that have points assigned to them to Face Stack.
 *         - Iterate until Face Stack is empty:
 *              - Pop topmost face F from the stack
 *              - From the points assigned to F, pick the point P that is farthest away from the plane defined by F.
 *              - Find all faces of M that have P on their positive side. Let us call these the "visible faces".
 *              - Because of the way M is constructed, these faces are connected. Solve their horizon edge loop.
 *				- "Extrude to P": Create new faces by connecting P with the points belonging to the horizon edge. Add the new faces to M and remove the visible
 *                faces from M.
 *              - Each point that was assigned to visible faces is now assigned to at most one of the newly created faces.
 *              - Those new faces that have points assigned to them are added to the top of Face Stack.
 *          - M is now the convex hull.
 *
 * TO DO:
 *  - Implement a proper 2D QuickHull and use that to solve the degenerate 2D case (when all the points lie on the same plane in 3D space).
 * */

namespace quickhull {
	
	template<typename FloatType, typename IndexType>
	class HalfEdgeMesh {
	public:
		
		struct HalfEdge {
			IndexType m_endVertex;
			IndexType m_opp;
			IndexType m_face;
			IndexType m_next;
		};
		
		struct Face {
			IndexType m_halfEdgeIndex; // Index of one of the half edges of this face
		};
		
		std::vector<Vector3<FloatType>> m_vertices;
		std::vector<Face> m_faces;
		std::vector<HalfEdge> m_halfEdges;
		
		HalfEdgeMesh(const MeshBuilder<FloatType>& builderObject, const VertexDataSource<FloatType>& vertexData )
		{
			std::unordered_map<IndexType,IndexType> faceMapping;
			std::unordered_map<IndexType,IndexType> halfEdgeMapping;
			std::unordered_map<IndexType, IndexType> vertexMapping;
			
			size_t i=0;
			for (const auto& face : builderObject.m_faces) {
				if (!face.isDisabled()) {
					m_faces.push_back({static_cast<IndexType>(face.m_he)});
					faceMapping[i] = m_faces.size()-1;
					
					const auto heIndices = builderObject.getHalfEdgeIndicesOfFace(face);
					for (const auto heIndex : heIndices) {
						const IndexType vertexIndex = builderObject.m_halfEdges[heIndex].m_endVertex;
						if (vertexMapping.count(vertexIndex)==0) {
							m_vertices.push_back(vertexData[vertexIndex]);
							vertexMapping[vertexIndex] = m_vertices.size()-1;
						}
					}
				}
				i++;
			}
			
			i=0;
			for (const auto& halfEdge : builderObject.m_halfEdges) {
				if (!halfEdge.isDisabled()) {
					m_halfEdges.push_back({static_cast<IndexType>(halfEdge.m_endVertex),static_cast<IndexType>(halfEdge.m_opp),static_cast<IndexType>(halfEdge.m_face),static_cast<IndexType>(halfEdge.m_next)});
					halfEdgeMapping[i] = m_halfEdges.size()-1;
				}
				i++;
			}
			
			for (auto& face : m_faces) {
				assert(halfEdgeMapping.count(face.m_halfEdgeIndex) == 1);
				face.m_halfEdgeIndex = halfEdgeMapping[face.m_halfEdgeIndex];
			}
			
			for (auto& he : m_halfEdges) {
				he.m_face = faceMapping[he.m_face];
				he.m_opp = halfEdgeMapping[he.m_opp];
				he.m_next = halfEdgeMapping[he.m_next];
				he.m_endVertex = vertexMapping[he.m_endVertex];
			}
		}
		
	};
}


#endif /* HalfEdgeMesh_h */
