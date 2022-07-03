/*
Copyright (c) 2020 Jingwei Huang. All rights reserved.

This software is provided by the copyright holders and the contributors 
"as is" and any express or implied warranties, including, but not limited 
to, the implied warranties of merchantability and fitness for a particular 
purpose are disclaimed. In no event shall the copyright holders or 
contributors be liable for any direct, indirect, incidental, special, 
exemplary, or consequential damages (including, but not limited to, 
procurement of substitute goods or services; loss of use, data, or profits;
or business interruption) however caused and on any theory of liability, 
whether in contract, strict liability, or tort (including negligence or 
otherwise) arising in any way out of the use of this software, even if 
advised of the possibility of such damage.

The views and conclusions contained in the software and documentation are 
those of the authors and should not be interpreted as representing official 
policies, either expressed or implied, of Jingwei Huang.
*/
#ifndef MANIFOLD2_MANIFOLD_H_
#define MANIFOLD2_MANIFOLD_H_

#include "GridIndex_p.h"

#include "Octree_p.h"

class ManifoldP {
public:
	ManifoldP();
	~ManifoldP();
	void ProcessManifold(const MatrixD& V, const MatrixI& F, int depth,
		MatrixD* out_V, MatrixI* out_F);

protected:
	void BuildTree(int resolution);
	void CalcBoundingBox();
	void ConstructManifold();
	bool SplitGrid(const std::vector<Vector4i>& nface_indices,
		std::map<GridIndex,int>& vcolor,
		std::vector<Vector3>& nvertices,
		std::vector<std::set<int> >& v_faces,
		std::vector<Vector3i>& triangles);

private:	
	OctreeP* tree_;
	Vector3 min_corner_, max_corner_;
	MatrixD V_;
	MatrixI F_;

	std::vector<Vector3> vertices_;
	std::vector<Vector3i> face_indices_;
	std::vector<GridIndex > v_info_;

};

#endif