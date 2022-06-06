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
#ifndef OCTREE_H_
#define OCTREE_H_

#include <list>
#include <map>
#include <set>
#include <vector>


#include "GridIndex_p.h"

class Octree
{
public:
	Octree();
	Octree(const Vector3 min_c, const Vector3 max_c, const MatrixI& faces);
	Octree(const Vector3& min_c, const Vector3& volume_size);
	~Octree();

	bool IsExterior(const Vector3 &p);

	bool Intersection(int face_index, const Vector3& min_corner,
		const Vector3& size, const MatrixD& V);


	void Split(const MatrixD& V);
	void BuildConnection();
	void ConnectTree(Octree* l, Octree* r, int dim);
	void ConnectEmptyTree(Octree* l, Octree* r, int dim);

	void ExpandEmpty(std::list<Octree*>& empty_list,
		std::set<Octree*>& empty_set, int dim);

	void BuildEmptyConnection();

	void ConstructFace(const Vector3i& start,
		std::map<GridIndex,int>* vcolor,
		std::vector<Vector3>* vertices,
		std::vector<Vector4i>* faces,
		std::vector<std::set<int> >* v_faces);

	Vector3 min_corner_, volume_size_;
	int level_;
	int number_;
	int occupied_;
	int exterior_;

	Octree* children_[8];
	Octree* connection_[6];
	Octree* empty_connection_[6];
	std::list<Octree*> empty_neighbors_;

	std::vector<Vector3i> F_;
	std::vector<int> Find_;
};


#endif
