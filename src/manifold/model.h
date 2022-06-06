/*
Copyright (c) 2018 Jingwei Huang. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
   may be used to endorse or promote products derived from this software
   without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

You are under no obligation whatsoever to provide any bug fixes, patches, or
upgrades to the features, functionality or performance of the source code
("Enhancements") to anyone; however, if you choose to make your Enhancements
available either publicly, or directly to the authors of this software, without
imposing a separate written license agreement for such Enhancements, then you
hereby grant the following license: a non-exclusive, royalty-free perpetual
license to install, use, modify, prepare derivative works, incorporate into
other computer software, distribute, and sublicense such enhancements or
derivative works thereof, in binary and source code form.
*/
#ifndef Model_OBJ_H_
#define Model_OBJ_H_

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "Octree.h"
#include "BVH.h"
#include <map>
#include <cstdlib>
// #include <igl/readOBJ.h>
// using namespace Eigen;
using namespace std;
/*************************************************************************** 
  OBJ Loading 
 ***************************************************************************/
 
class Model_OBJ
{
  public: 
  	struct Edge_Info
  	{
  		int face_x, face_y;
  		map<int, int> loop_index;
  	};
  vector<set<int> > v_faces;

	Model_OBJ();			
  int Load(char *filename);	// Loads the model

 	void Calc_Bounding_Box();

  void Process_Manifold(int resolution);
  void Build_Tree(int resolution);
  void Build_BVH();
  void Construct_Manifold();
  void Project_Manifold();
  bool Project(glm::dvec3& o, glm::dvec3& d);
  void Save(const char* filename, bool color);
  void SaveOBJ(const char* filename);
  glm::dvec3 Closest_Point( const glm::dvec3 *triangle, const glm::dvec3 &sourcePosition );
  glm::dvec3 Find_Closest(int i);
  int is_manifold();
  bool Split_Grid(map<Grid_Index,int>& vcolor, vector<glm::dvec3>& nvertices, vector<glm::ivec4>& nface_indices, vector<set<int> >& v_faces, vector<glm::ivec3>& triangles);
  double clamp(double d1, double l, double r)
  {
    if (d1 < l)
      return l;
    if (d1 > r)
      return l;
    return d1;
  }
  vector<Grid_Index > v_info;
  
	vector<glm::dvec3> vertices, vertices_buf;
  vector<glm::dvec3> colors;
	vector<glm::ivec3> face_indices, face_indices_buf;
  vector<glm::dvec3> face_normals;
	
	glm::dvec3 min_corner, max_corner;
  Octree* tree;
  BVH* bvh;
  vector<BV*> bvs;
  char* fn;
  // vector field
  // Eigen::MatrixXd V;
  // Eigen::MatrixXi F;

};

#endif
