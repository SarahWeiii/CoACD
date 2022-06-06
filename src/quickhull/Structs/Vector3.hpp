#ifndef QuickHull_Vector3_hpp
#define QuickHull_Vector3_hpp

#include <cmath>
#include <iostream>

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
	
	template <typename T>
	class Vector3
	{
	public:
		Vector3() = default;
		
		Vector3(T x, T y, T z) : x(x), y(y), z(z) {
			
		}
		
		T x,y,z;
		
		T dotProduct(const Vector3& other) const {
			return x*other.x+y*other.y+z*other.z;
		}
		
		void normalize() {
			const T len = getLength();
			x/=len;
			y/=len;
			z/=len;
		}
		
		Vector3 getNormalized() const {
			const T len = getLength();
			return Vector3(x/len,y/len,z/len);
		}
		
		T getLength() const {
			return std::sqrt(x*x+y*y+z*z);
		}
		
		Vector3 operator-(const Vector3& other) const {
			return Vector3(x-other.x,y-other.y,z-other.z);
		}
		
		Vector3 operator+(const Vector3& other) const {
			return Vector3(x+other.x,y+other.y,z+other.z);
		}
		
		Vector3& operator+=(const Vector3& other) {
			x+=other.x;
			y+=other.y;
			z+=other.z;
			return *this;
		}
		Vector3& operator-=(const Vector3& other) {
			x-=other.x;
			y-=other.y;
			z-=other.z;
			return *this;
		}
		Vector3& operator*=(T c) {
			x*=c;
			y*=c;
			z*=c;
			return *this;
		}
		
		Vector3& operator/=(T c) {
			x/=c;
			y/=c;
			z/=c;
			return *this;
		}
		
		Vector3 operator-() const {
			return Vector3(-x,-y,-z);
		}

		template<typename S>
		Vector3 operator*(S c) const {
			return Vector3(x*c,y*c,z*c);
		}
		
		template<typename S>
		Vector3 operator/(S c) const {
			return Vector3(x/c,y/c,z/c);
		}
		
		T getLengthSquared() const {
			return x*x + y*y + z*z;
		}
		
		bool operator!=(const Vector3& o) const {
			return x != o.x || y != o.y || z != o.z;
		}
		
		// Projection onto another vector
		Vector3 projection(const Vector3& o) const {
			T C = dotProduct(o)/o.getLengthSquared();
			return o*C;
		}
		
		Vector3 crossProduct (const Vector3& rhs ) {
			T a = y * rhs.z - z * rhs.y ;
			T b = z * rhs.x - x * rhs.z ;
			T c = x * rhs.y - y * rhs.x ;
			Vector3 product( a , b , c ) ;
			return product ;
		}
		
		T getDistanceTo(const Vector3& other) const {
			Vector3 diff = *this - other;
			return diff.getLength();
		}
		
		T getSquaredDistanceTo(const Vector3& other) const {
			const T dx = x-other.x;
			const T dy = y-other.y;
			const T dz = z-other.z;
			return dx*dx+dy*dy+dz*dz;
		}
		
	};
	
	// Overload also << operator for easy printing of debug data
	template <typename T>
	inline std::ostream& operator<<(std::ostream& os, const Vector3<T>& vec) {
		os << "(" << vec.x << "," << vec.y << "," << vec.z << ")";
		return os;
	}
	
	template <typename T>
	inline Vector3<T> operator*(T c, const Vector3<T>& v) {
		return Vector3<T>(v.x*c,v.y*c,v.z*c);
	}
	
}


#endif
