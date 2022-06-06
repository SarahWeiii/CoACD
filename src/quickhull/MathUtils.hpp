
#ifndef QuickHull_MathUtils_hpp
#define QuickHull_MathUtils_hpp

#include "Structs/Vector3.hpp"
#include "Structs/Ray.hpp"

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
	
	namespace mathutils {
		
		template <typename T>
		inline T getSquaredDistanceBetweenPointAndRay(const Vector3<T>& p, const Ray<T>& r) {
			const Vector3<T> s = p-r.m_S;
			T t = s.dotProduct(r.m_V);
			return s.getLengthSquared() - t*t*r.m_VInvLengthSquared;
		}
		
		// Note that the unit of distance returned is relative to plane's normal's length (divide by N.getNormalized() if needed to get the "real" distance).
		template <typename T>
		inline T getSignedDistanceToPlane(const Vector3<T>& v, const Plane<T>& p) {
			return p.m_N.dotProduct(v) + p.m_D;
		}
		
		template <typename T>
		inline Vector3<T> getTriangleNormal(const Vector3<T>& a,const Vector3<T>& b,const Vector3<T>& c) {
			// We want to get (a-c).crossProduct(b-c) without constructing temp vectors
			T x = a.x - c.x;
			T y = a.y - c.y;
			T z = a.z - c.z;
			T rhsx = b.x - c.x;
			T rhsy = b.y - c.y;
			T rhsz = b.z - c.z;
			T px = y * rhsz - z * rhsy ;
			T py = z * rhsx - x * rhsz ;
			T pz = x * rhsy - y * rhsx ;
			return Vector3<T>(px,py,pz);
		}
		
		
	}
	
}


#endif
