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
#include "model.h"
#include <queue>
#define ITER_NUM 20
int g_sharp = 0;
Model_OBJ::Model_OBJ()
{
}
 
int Model_OBJ::Load(char* filename)
{
	using std::endl; 
    const unsigned int BufferSize = 1024;
    FILE* fid = fopen(filename, "r");

    if (fid) {
        char buffer[BufferSize];
        int ip[4];
        double x[3];
        char* pch;
        char* str;
        while (!feof(fid)) {
            if (!fgets(buffer, BufferSize, fid)) {
                break;
            }
            else if (buffer[0] == 'v') {
                if (buffer[1] == ' ') {
                    str = buffer + 2;
                    for (int k = 0; k < 3; ++k) {
                        pch = strtok(str, " ");
                        if (pch)
                            x[k] = (double)atof(pch);
                        else {
                            return false;
                        }
                        str = NULL;
                    }
					vertices.push_back(glm::dvec3(x[0], x[1], x[2]));
                    // points.push_back({x[0], x[1], x[2]});
                }
            }
            else if (buffer[0] == 'f') {

                pch = str = buffer + 2;
                int k = 0;
                while (pch) {
                    pch = strtok(str, " ");
                    if (pch && *pch != '\n') {
                        ip[k++] = (unsigned int)atoi(pch) - 1;
                    }
                    else {
                        break;
                    }
                    str = NULL;
                }
                if (k == 3) {
					face_indices.push_back(glm::ivec3(ip[0], ip[1], ip[2]));
                }
                else if (k == 4) {
					face_indices.push_back(glm::ivec3(ip[0], ip[1], ip[2]));
					face_indices.push_back(glm::ivec3(ip[0], ip[2], ip[3]));
                }
            }
        }
        fclose(fid);
    }
    else {
        std::cout << "Open File Error!" << std::endl;
    }
    return 0;
	
	
	
    // // Load a mesh in OBJ format
    // igl::readOBJ(filename, V, F);
    // fn = filename;
    // // Make the example deterministic
    // srand(0);
    // vertices.resize(V.rows());
    // face_indices.resize(F.rows());
    // for (int i = 0; i < V.rows(); ++i)
    // 	vertices[i] = glm::dvec3(V(i,0),V(i,1),V(i,2));
    // for (int i = 0; i < F.rows(); ++i) {
    // 	face_indices[i] = glm::ivec3(F(i,0),F(i,1),F(i,2));
	// }

	// return 0;
}
 
void Model_OBJ::Calc_Bounding_Box()
{
	min_corner = glm::dvec3(1e30,1e30,1e30);
	max_corner = -min_corner;
	for (int i = 0; i < (int)vertices.size(); ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			if (vertices[i][j] < min_corner[j])
			{
				min_corner[j] = vertices[i][j];
			}
			if (vertices[i][j] > max_corner[j])
			{
				max_corner[j] = vertices[i][j];
			}
		}
	}
	glm::dvec3 length = max_corner - min_corner;
	min_corner -= length * 0.2;
	max_corner += length * 0.2;
}

void Model_OBJ::Build_Tree(int resolution)
{
	Calc_Bounding_Box();
	tree = new Octree(min_corner, max_corner, face_indices, 0.01);
	while (tree->number < resolution)
	{
		tree->Split(vertices);
	}

	tree->BuildConnection();
	tree->BuildEmptyConnection();

	std::list<Octree*> empty_list;
	std::set<Octree*> empty_set;
	for (int i = 0; i < 6; ++i)
	{
		tree->ExpandEmpty(empty_list, empty_set, i);
	}

	while ((int)empty_list.size() > 0)
	{
		Octree* empty = empty_list.front();
		empty->exterior = 1;
		for (std::list<Octree*>::iterator it = empty->empty_neighbors.begin();
			it != empty->empty_neighbors.end(); ++it)
		{
			if (empty_set.find(*it) == empty_set.end())
			{
				empty_list.push_back(*it);
				empty_set.insert(*it);
			}
		}
		empty_list.pop_front();
	}
}

glm::dvec3 Model_OBJ::Closest_Point( const glm::dvec3 *triangle, const glm::dvec3 &sourcePosition )
{
    glm::dvec3 edge0 = triangle[1] - triangle[0];
    glm::dvec3 edge1 = triangle[2] - triangle[0];
    glm::dvec3 v0 = triangle[0] - sourcePosition;

    double a = glm::dot(edge0, edge0 );
    double b = glm::dot(edge0, edge1 );
    double c = glm::dot(edge1, edge1 );
    double d = glm::dot(edge0, v0 );
    double e = glm::dot(edge1, v0 );

    double det = a*c - b*b;
    double s = b*e - c*d;
    double t = b*d - a*e;

    if ( s + t < det )
    {
        if ( s < 0.f )
        {
            if ( t < 0.f )
            {
                if ( d < 0.f )
                {
                    s = clamp( -d/a, 0.f, 1.f );
                    t = 0.f;
                }
                else
                {
                    s = 0.f;
                    t = clamp( -e/c, 0.f, 1.f );
                }
            }
            else
            {
                s = 0.f;
                t = clamp( -e/c, 0.f, 1.f );
            }
        }
        else if ( t < 0.f )
        {
            s = clamp( -d/a, 0.f, 1.f );
            t = 0.f;
        }
        else
        {
            float invDet = 1.f / det;
            s *= invDet;
            t *= invDet;
        }
    }
    else
    {
        if ( s < 0.f )
        {
            float tmp0 = b+d;
            float tmp1 = c+e;
            if ( tmp1 > tmp0 )
            {
                float numer = tmp1 - tmp0;
                float denom = a-2*b+c;
                s = clamp( numer/denom, 0.f, 1.f );
                t = 1-s;
            }
            else
            {
                t = clamp( -e/c, 0.f, 1.f );
                s = 0.f;
            }
        }
        else if ( t < 0.f )
        {
            if ( a+d > b+e )
            {
                float numer = c+e-b-d;
                float denom = a-2*b+c;
                s = clamp( numer/denom, 0.f, 1.f );
                t = 1-s;
            }
            else
            {
                s = clamp( -e/c, 0.f, 1.f );
                t = 0.f;
            }
        }
        else
        {
            float numer = c+e-b-d;
            float denom = a-2*b+c;
            s = clamp( numer/denom, 0.f, 1.f );
            t = 1.f - s;
        }
    }

    return triangle[0] + s * edge0 + t * edge1;
}

void Model_OBJ::Construct_Manifold()
{
	std::map<Grid_Index,int> vcolor;
	std::vector<glm::dvec3> nvertices;
	std::vector<glm::ivec4> nface_indices;
	std::vector<glm::ivec3> triangles;
	tree->ConstructFace(vcolor,glm::ivec3(0,0,0),nvertices,nface_indices, v_faces);
	Split_Grid(vcolor, nvertices, nface_indices, v_faces, triangles);
	std::vector<int> hash_v(nvertices.size(),0);
	for (int i = 0; i < (int)triangles.size(); ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			hash_v[triangles[i][j]] = 1;
		}
	}
	vertices.clear();
	for (int i = 0; i < (int)hash_v.size(); ++i)
	{
		if (hash_v[i])
		{
			hash_v[i] = (int)vertices.size();
			v_faces[vertices.size()] = v_faces[i];
			v_info[vertices.size()] = v_info[i];
			vertices.push_back(nvertices[i]);
			colors.push_back(glm::dvec3(1,1,1));
		}
	}
	for (int i = 0; i < (int)triangles.size(); ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			triangles[i][j] = hash_v[triangles[i][j]];
		}
	}
	face_indices = triangles;
}

glm::dvec3 Model_OBJ::Find_Closest(int i)
{
	glm::dvec3 cpoint = glm::dvec3(1e20,1e20,1e20);
	glm::dvec3 tris[3];
	glm::dvec3 normal;
	for (std::set<int>::iterator it = v_faces[i].begin();
		it != v_faces[i].end(); ++it)
	{
		int face_ind = *it;
		for (int j = 0; j < 3; ++j)
			tris[j] = vertices_buf[face_indices_buf[face_ind][j]];
		glm::dvec3 p = Closest_Point(tris, vertices[i]);
		if (glm::length(p-vertices[i]) < glm::length(cpoint-vertices[i]))
		{
			normal = glm::normalize(glm::cross(tris[1]-tris[0],tris[2]-tris[0]));
			if (glm::dot(normal,vertices[i]-cpoint)<0)
				normal = -normal;
			cpoint = p;
		}
	}
	return cpoint + normal * 5e-4;
}

void Model_OBJ::Project_Manifold()
{
	double len = glm::length(vertices[face_indices[0][1]] - vertices[face_indices[0][0]]);
	double min_len = glm::length(vertices[face_indices[0][2]] - vertices[face_indices[0][0]]);
	if (min_len < len)
		len = min_len;
	colors.clear();
	colors.resize(vertices.size(),glm::dvec3(1,1,1));

	std::vector<std::vector<int> > vertex_faces(vertices.size());
	face_normals.resize(face_indices.size());
	for (int i = 0; i < (int)face_indices.size(); ++i)
	{
		int id[3];
		id[0] = face_indices[i][0];
		id[1] = face_indices[i][1];
		id[2] = face_indices[i][2];
		for (int j = 0; j < 3; ++j)
		{
			vertex_faces[id[j]].push_back(i);
		}
	}
	std::vector<int> vertices_hash(vertices.size(), 0);
	double min_step = 2.0 / ITER_NUM;

for (int iter = 0; iter < ITER_NUM; ++iter) {

	for (int i = 0; i < (int)face_indices.size(); ++i)
	{
		int id[3];
		id[0] = face_indices[i][0];
		id[1] = face_indices[i][1];
		id[2] = face_indices[i][2];
		face_normals[i] = glm::normalize(glm::cross(vertices[id[1]] - vertices[id[0]],
			vertices[id[2]] - vertices[id[0]]));
	}

	std::vector<int> invalid_vertices;
	std::vector<int> invalid_indices(vertices.size(), -1);

	for (int i = 0; i < (int)vertices.size(); ++i)
	{
		if (vertices_hash[i])
			continue;
		glm::dvec3 cpoint = Find_Closest(i);
		glm::dvec3 move_dir = cpoint - vertices[i];
		double orig_step = glm::length(move_dir);
		move_dir /= orig_step;
		double step = orig_step;
		bool flag = step < 1e15;
		if (g_sharp) {
			vertices[i] = cpoint;
			continue;
		}
		glm::dvec3 normal(0,0,0);
		for (int j = 0; j < (int)vertex_faces[i].size(); ++j)
		{
			normal += face_normals[vertex_faces[i][j]];
		}
		normal = glm::normalize(normal);
		if (flag) {
			bool convex = true;
			for (int j = 0; j < (int)vertex_faces[i].size(); ++j)
			{
				for (int k = 0; k < 3; ++k) {
					if (glm::dot(vertices[face_indices[vertex_faces[i][j]][k]] - vertices[i], normal) > 0)
						convex = false;
				}
				if (!convex)
					break;
			}
			if (convex)
			{
				for (int j = 0; j < (int)vertex_faces[i].size(); ++j)
				{
					if (glm::dot(face_normals[vertex_faces[i][j]],move_dir)>0)
					{
						flag = false;
						break;
					}
				}
			} else 
			{
				flag = false;
				for (int j = 0; j < (int)vertex_faces[i].size(); ++j)
				{
					if (glm::dot(face_normals[vertex_faces[i][j]],move_dir)<0)
					{
						flag = true;
						break;
					}
				}
			}
		}
		if (flag)
		{
			if (step > min_step * len)
				step = min_step * len;
			for (int j = 0; j < (int)vertex_faces[i].size(); ++j)
			{
				glm::ivec3& face_index = face_indices[vertex_faces[i][j]];
				int t = 0;
				while (face_index[t] != i)
					t += 1;
				glm::dvec3 dir = glm::normalize(vertices[face_index[(t+2)%3]] - vertices[face_index[(t+1)%3]]);
				glm::dvec3 h = vertices[face_index[(t+1)%3]]+glm::dot(vertices[i] - vertices[face_index[(t+1)%3]],dir) * dir - vertices[i];
				double h_len = glm::length(h);
				h /= h_len;
				double h_step = glm::dot(h,move_dir) * step;
				if (h_step > h_len * 0.7)
				{
					step *= (h_len * 0.7) / h_step;
					invalid_indices[i] = (int)invalid_vertices.size();
					invalid_vertices.push_back(i);
					colors[i] = glm::dvec3(0,0,1);
				}
			}
			if (fabs(step - orig_step)<1e-6) {
				vertices[i] = cpoint + len * normal;
				if (step > 1e-4)
					step -= 1e-4;
				else
					step = 0;
				vertices_hash[i] = 1;
			} else
			vertices[i] += step * move_dir;//* 0.1 * glm::normalize(move_dir);
			for (int j = 0; j < (int)vertex_faces[i].size(); ++j)
			{
				int face = vertex_faces[i][j];
				face_normals[face] = glm::normalize(glm::cross(vertices[face_indices[face][1]] - vertices[face_indices[face][0]],
					vertices[face_indices[face][2]] - vertices[face_indices[face][0]]));
			}
		} else
		{
			invalid_indices[i] = (int)invalid_vertices.size();
			invalid_vertices.push_back(i);
			colors[i] = glm::dvec3(0,0,1);
		}
	}
//	cout << "Invalid " << invalid_vertices.size() << "\n";
	std::vector<int> invalid_colors(invalid_vertices.size(), -1);
	int c = 0;
	for (int i = 0; i < (int)invalid_vertices.size(); ++i)
	{
		if (invalid_colors[i] == -1)
		{
//			colors[invalid_vertices[i]] = glm::dvec3(0,0,1);
			invalid_colors[i] = c;
			std::vector<int> queue;
			int f = 0;
			queue.push_back(i);
			while (f != (int)queue.size())
			{
				int id = invalid_vertices[queue[f]];
				for (int j = 0; j < (int)vertex_faces[id].size(); ++j)
				{
					for (int k = 0; k < 3; ++k)
					{
						int index = invalid_indices[face_indices[vertex_faces[id][j]][k]];
						if (index != -1 && invalid_colors[index] == -1)
						{
							invalid_colors[index] = c;
							queue.push_back(index);
						}
					}
				}
				f++;
			}
			for (std::vector<int>::reverse_iterator it = queue.rbegin();
				it != queue.rend(); ++it)
			{
				glm::dvec3 midpoint(0,0,0);
				int count = 0;
				int id = invalid_vertices[*it];
				for (int j = 0; j < (int)vertex_faces[id].size(); ++j)
				{
					for (int k = 0; k < 3; ++k)
					{
						int vind = face_indices[vertex_faces[id][j]][k];
						if (invalid_indices[vind] == -1 || invalid_colors[invalid_indices[vind]] == -1)
						{
							midpoint += vertices[vind];
							count += 1;
						}
					}
				}
				glm::dvec3 move_dir = midpoint / (double)count - vertices[id];
				invalid_colors[*it] = -1;
				double l = glm::length(move_dir);
				if (l == 0 || count == 0)
					continue;
				move_dir /= l;
				for (int j = 0; j < (int)vertex_faces[id].size(); ++j)
				{
					glm::ivec3& face_index = face_indices[vertex_faces[id][j]];
					int t = 0;
					while (face_index[t] != id)
						t += 1;
					glm::dvec3 dir = glm::normalize(vertices[face_index[(t+2)%3]] - vertices[face_index[(t+1)%3]]);
					glm::dvec3 h = vertices[face_index[(t+1)%3]]+glm::dot(vertices[id] - vertices[face_index[(t+1)%3]],dir) * dir - vertices[id];
					double h_len = glm::length(h);
					h /= h_len;
					double h_step = glm::dot(h,move_dir) * l;
					if (h_step > h_len * 0.7)
					{
						l *= (h_len * 0.7) / h_step;
					}
				}
				move_dir *= l;
				vertices[id] += move_dir;
			}
			for (int i = 0; i < (int)queue.size(); ++i)
			{
				invalid_colors[queue[i]] = c;
			}
		}
		c++;
	}
}
}

bool Model_OBJ::Project(glm::dvec3& o, glm::dvec3& d)
{
	std::pair<glm::dvec3,bool> p = bvh->rayIntersect(o, d);
	if (!p.second)
		return false;
	o = p.first;
	return true;
}

void Model_OBJ::Build_BVH()
{
	bvh = new BVH();
	bvs.resize(face_indices.size());
	for (int i = 0; i < (int)face_indices.size(); ++i)
	{
		bvs[i] = new BV(vertices[face_indices[i][0]],
			vertices[face_indices[i][1]],
			vertices[face_indices[i][2]]);
	}
//	bvh->updateBVH(bvs, 0, 0, bvs.size()-1);
}

void Model_OBJ::Process_Manifold(int resolution)
{
	vertices_buf = vertices;
	face_indices_buf = face_indices;
	// if(is_manifold() == 0)
	// {
	// 	cout << "Input Manifold! " << endl;
	// 	return;
	// }
//	Build_BVH();
	Build_Tree(resolution);
	Construct_Manifold();
	// Project_Manifold();

	for (int iter = 0; iter < 1; ++iter)
	{
		std::vector<glm::dvec3> dis(vertices.size());
		std::vector<int> dis_weight(vertices.size());
		for (int i = 0; i < (int)face_indices.size(); ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				int x = face_indices[i][j];
				int y = face_indices[i][(j + 1) % 3];
				dis[x] += vertices[y];
				dis[y] += vertices[x];
				dis_weight[x] += 1;
				dis_weight[y] += 1;
			}
		}
		for (int i = 0; i < (int)vertices.size(); ++i)
		{
			if (dis_weight[i] > 0)
				vertices[i] = dis[i] * (1.0 / dis_weight[i]);
		}
	}
	int flag = is_manifold();
	if (flag != 0)
	{
		std::ofstream os("error.txt");
		os << fn << "\n";
		os.close();
		std::cout << "Not a Manifold! " << flag << "\n";
		exit(0);
	}
}

bool Model_OBJ::Split_Grid(std::map<Grid_Index,int>& vcolor, std::vector<glm::dvec3>& nvertices, std::vector<glm::ivec4>& nface_indices, std::vector<std::set<int> >& v_faces, std::vector<glm::ivec3>& triangles)
{
	double unit_len = 0;
	v_info.resize(vcolor.size());
	for (std::map<Grid_Index,int>::iterator it = vcolor.begin();
		it != vcolor.end(); ++it)
	{
		v_info[it->second] = it->first;
	}
	std::set<int> marked_v;
	std::map<std::pair<int,int>,std::list<std::pair<int,int> > > edge_info;
	for (int i = 0; i < (int)nface_indices.size(); ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			int x = nface_indices[i][j];
			int y = nface_indices[i][(j + 1) % 4];
			if (x > y)
			{
				int temp = x;
				x = y;
				y = temp;
			}
			std::pair<int,int> edge = std::make_pair(x,y);
			std::map<std::pair<int,int>, std::list<std::pair<int,int> > >::iterator it = edge_info.find(edge);
			if (it != edge_info.end())
			{
				it->second.push_back(std::make_pair(i,j));
			} else
			{
				std::list<std::pair<int,int> > buf;
				buf.push_back(std::make_pair(i,j));
				edge_info.insert(std::make_pair(edge, buf));
			}
		}
	}
	for (std::map<std::pair<int,int>,std::list<std::pair<int,int> > >::iterator it = edge_info.begin();
		it != edge_info.end(); ++it)
	{
		if (it->second.size() > 2) {
			marked_v.insert(it->first.first);
			marked_v.insert(it->first.second);
		}
	}
	triangles.clear();
	double half_len = glm::length(nvertices[nface_indices[0][1]] - nvertices[nface_indices[0][0]]) * 0.5;
	for (int i = 0; i < (int)nface_indices.size(); ++i)
	{
		int t = 0;
		while (t < 4 && marked_v.find(nface_indices[i][t]) == marked_v.end())
			++t;
		if (t == 4)
		{
			triangles.push_back(glm::ivec3(nface_indices[i][0],nface_indices[i][2],nface_indices[i][1]));
			triangles.push_back(glm::ivec3(nface_indices[i][0],nface_indices[i][3],nface_indices[i][2]));
			continue;
		}
		int ind[4];
		for (int j = 0; j < 4; ++j)
			ind[j] = nface_indices[i][(t+j)%4];
		bool flag1 = marked_v.find(ind[1]) != marked_v.end();
		bool flag2 = marked_v.find(ind[2]) != marked_v.end();
		bool flag3 = marked_v.find(ind[3]) != marked_v.end();
		Grid_Index pt1 = (v_info[ind[0]] + v_info[ind[1]]) / 2;
		Grid_Index pt2 = (v_info[ind[0]] + v_info[ind[3]]) / 2;
		Grid_Index pt3 = (v_info[ind[2]] + v_info[ind[3]]) / 2;
		Grid_Index pt4 = (v_info[ind[1]] + v_info[ind[2]]) / 2;
		int ind1, ind2, ind3, ind4;
		std::map<Grid_Index,int>::iterator it = vcolor.find(pt1);
		if (it == vcolor.end())
		{
			vcolor.insert(std::make_pair(pt1,nvertices.size()));
			v_info.push_back(pt1);
			ind1 = (int)nvertices.size();
			nvertices.push_back((nvertices[ind[0]]+nvertices[ind[1]])*0.5);
			v_faces.push_back(v_faces[ind[0]]);
		} else
		ind1 = it->second;
		it = vcolor.find(pt2);
		if (it == vcolor.end())
		{
			vcolor.insert(std::make_pair(pt2,nvertices.size()));
			v_info.push_back(pt2);
			ind2 = (int)nvertices.size();
			v_faces.push_back(v_faces[ind[0]]);
			nvertices.push_back((nvertices[ind[0]]+nvertices[ind[3]])*0.5);
		} else
		ind2 = it->second;
		if (flag1 || flag2)
		{
			it = vcolor.find(pt4);
			if (it == vcolor.end())
			{
				vcolor.insert(std::make_pair(pt4,nvertices.size()));
				v_info.push_back(pt4);
				ind4 = (int)nvertices.size();
				nvertices.push_back((nvertices[ind[1]]+nvertices[ind[2]])*0.5);
				if (flag1)
					v_faces.push_back(v_faces[ind[1]]);
				else
					v_faces.push_back(v_faces[ind[2]]);
			} else
			ind4 = it->second;
		}
		if (flag2 || flag3)
		{
			it = vcolor.find(pt3);
			if (it == vcolor.end())
			{
				vcolor.insert(std::make_pair(pt3,nvertices.size()));
				v_info.push_back(pt3);
				ind3 = (int)nvertices.size();
				nvertices.push_back((nvertices[ind[2]]+nvertices[ind[3]])*0.5);
				if (flag2)
					v_faces.push_back(v_faces[ind[2]]);
				else
					v_faces.push_back(v_faces[ind[3]]);
			} else
			ind3 = it->second;			
		}
		if (!flag1 && !flag2 && !flag3)
		{
			triangles.push_back(glm::ivec3(ind1,ind[2],ind[1]));
			triangles.push_back(glm::ivec3(ind2,ind[2],ind1));
			triangles.push_back(glm::ivec3(ind[3],ind[2],ind2));
		} else
		if (!flag1 && !flag2 && flag3)
		{
			triangles.push_back(glm::ivec3(ind1,ind2,ind3));
			triangles.push_back(glm::ivec3(ind1,ind3,ind[2]));
			triangles.push_back(glm::ivec3(ind1,ind[2],ind[1]));			
		} else
		if (!flag1 && flag2 && !flag3)
		{			
			triangles.push_back(glm::ivec3(ind1,ind4,ind[1]));
			triangles.push_back(glm::ivec3(ind1,ind2,ind4));
			triangles.push_back(glm::ivec3(ind2,ind[3],ind3));			
			triangles.push_back(glm::ivec3(ind2,ind3,ind4));			
		} else
		if (!flag1 && flag2 && flag3)
		{			
			triangles.push_back(glm::ivec3(ind1,ind4,ind[1]));
			triangles.push_back(glm::ivec3(ind1,ind2,ind4));
			triangles.push_back(glm::ivec3(ind2,ind3,ind4));			
		} else
		if (flag1 && !flag2 && !flag3)
		{			
			triangles.push_back(glm::ivec3(ind1,ind2,ind4));
			triangles.push_back(glm::ivec3(ind4,ind2,ind[3]));
			triangles.push_back(glm::ivec3(ind4,ind[3],ind[2]));
		} else
		if (flag1 && !flag2 && flag3)
		{			
			triangles.push_back(glm::ivec3(ind1,ind2,ind4));
			triangles.push_back(glm::ivec3(ind4,ind2,ind3));
			triangles.push_back(glm::ivec3(ind4,ind3,ind[2]));
		} else
		if (flag1 && flag2 && !flag3)
		{			
			triangles.push_back(glm::ivec3(ind1,ind2,ind4));
			triangles.push_back(glm::ivec3(ind2,ind3,ind4));
			triangles.push_back(glm::ivec3(ind2,ind[3],ind3));			
		} else
		if (flag1 && flag2 && flag3)
		{			
			triangles.push_back(glm::ivec3(ind1,ind2,ind3));
			triangles.push_back(glm::ivec3(ind1,ind3,ind4));
		}
	}
	for (std::set<int>::iterator it = marked_v.begin();
		it != marked_v.end(); ++it)
	{
		glm::dvec3 p = nvertices[*it];
		for (int dimx = -1; dimx < 2; dimx += 2) {
			for (int dimy = -1; dimy < 2; dimy += 2) {
				for (int dimz = -1; dimz < 2; dimz += 2) {
					glm::dvec3 p1 = p + glm::dvec3(dimx * half_len, dimy * half_len, dimz * half_len);
					if (tree->Is_Exterior(p1))
					{
						Grid_Index ind = v_info[*it];
						Grid_Index ind1 = ind;
						Grid_Index ind2 = ind;
						Grid_Index ind3 = ind;
						ind1.id[0] += dimx;
						ind2.id[1] += dimy;
						ind3.id[2] += dimz;
						if (vcolor.find(ind1) == vcolor.end())
						{
							vcolor.insert(std::make_pair(ind1, nvertices.size()));
							v_info.push_back(ind1);

							nvertices.push_back(glm::dvec3(p[0]+half_len*dimx,p[1],p[2]));
							v_faces.push_back(v_faces[*it]);
						}
						if (vcolor.find(ind2) == vcolor.end())
						{
							vcolor.insert(std::make_pair(ind2, nvertices.size()));
							v_info.push_back(ind2);

							nvertices.push_back(glm::dvec3(p[0],p[1]+half_len*dimy,p[2]));
							v_faces.push_back(v_faces[*it]);
						}
						if (vcolor.find(ind3) == vcolor.end())
						{
							vcolor.insert(std::make_pair(ind3, nvertices.size()));
							v_info.push_back(ind3);

							nvertices.push_back(glm::dvec3(p[0],p[1],p[2]+half_len*dimz));
							v_faces.push_back(v_faces[*it]);
						}
						int id1 = vcolor[ind1];
						int id2 = vcolor[ind2];
						int id3 = vcolor[ind3];
						glm::dvec3 norm = glm::cross(nvertices[id2]-nvertices[id1], nvertices[id3]-nvertices[id1]);
						if (glm::dot(norm, glm::dvec3(dimx,dimy,dimz)) < 0)
							triangles.push_back(glm::ivec3(id1,id3,id2));
						else
							triangles.push_back(glm::ivec3(id1,id2,id3));
					}
				}
			}
		}
	}
	std::map<int,std::set<std::pair<int,int> > > ocs, ecs;
	std::set<int> odds;
	std::set<int> evens;
	for (int i = 0; i < (int)nvertices.size(); ++i)
	{
		bool flag = false;
		for (int k = 0; k < 3; ++k)
			if (v_info[i].id[k] % 2 == 1)
				flag = true;
		if (flag) {
			odds.insert(i);
			ocs.insert(std::make_pair(i,std::set<std::pair<int,int> >()));
		}
	}
	for (int i = 0; i < (int)nvertices.size(); ++i)
	{
		Grid_Index ind = v_info[i];
		int flag = 0;
		while (flag < 3 && ind.id[flag] % 2 == 0)
		{
			flag++;
		}
		if (flag < 3)
			continue;
		for (int j = -2; j < 5; j += 4)
		{
			if (flag < 3)
				break;
			for (int k = 0; k < 3; ++k)
			{
				Grid_Index ind1 = ind;
				ind1.id[k] += j;
				std::map<Grid_Index,int>::iterator it = vcolor.find(ind1);
				if (it == vcolor.end())
				{
					flag = 0;
					break;
				}
				int y = it->second;
				unit_len = glm::length(nvertices[y] - nvertices[i]);
				std::pair<int,int> edge_id;
				if (i < y)
					edge_id = std::make_pair(i,y);
				else
					edge_id = std::make_pair(y,i);
				if (edge_info.find(edge_id) == edge_info.end())
				{
					flag = 0;
					break;
				}
			}
		}
		if (flag < 3)
			continue;
		evens.insert(i);
		ecs.insert(std::make_pair(i,std::set<std::pair<int,int> >()));
	}
	for (int i = 0; i < (int)triangles.size(); ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			int x = triangles[i][j];
			if (odds.find(x) != odds.end())
			{
				ocs[x].insert(std::make_pair(i,j));
			}
			if (evens.find(x) != evens.end())
			{
				ecs[x].insert(std::make_pair(i,j));
			}
		}
	}
	for (std::set<int>::iterator it = evens.begin();
		it != evens.end(); ++it)
	{
		int i = *it;
		glm::dvec3 dir;
		int count = 0;
		for (int j = 0; j < 8; ++j)
		{
			glm::dvec3 d((j&0x04)>0,(j&0x02)>0,(j&0x01)>0);
			d = d * 2.0 - glm::dvec3(1,1,1);
			d = glm::normalize(d) * (unit_len * 0.5);
			if (!tree->Is_Exterior(nvertices[i] + d))
			{
				dir = glm::normalize(d);
				count += 1;
			}
		}
		if (count > 2)
			continue;
		std::set<std::pair<int,int> >& p = ecs[i];
		for (std::set<std::pair<int,int> >::iterator it1 = p.begin();
			it1 != p.end(); ++it1)
		{
			assert(triangles[it1->first][it1->second] == i);
			if (glm::dot(nvertices[triangles[it1->first][(it1->second+1)%3]]-nvertices[i],dir)<0)
			{
				triangles[it1->first][it1->second] = (int)nvertices.size();
			}
		}
		nvertices[i] += dir * (0.5 * unit_len);
		v_faces.push_back(v_faces[i]);
		nvertices.push_back(nvertices[i]);
		nvertices.back() -= unit_len * dir;

	}
	for (std::set<int>::iterator it = odds.begin();
		it != odds.end(); ++it)
	{
		int i = *it;
		int k = 0;
		while (v_info[i].id[k] % 2 == 0)
			k += 1;
		Grid_Index id1, id2;
		id1 = v_info[i];
		id2 = v_info[i];
		id1.id[k] -= 1;
		id2.id[k] += 1;
		int x = vcolor[id1];
		int y = vcolor[id2];
		if (x > y)
		{
			int temp = x;
			x = y;
			y = temp;
		}
		if (edge_info[std::make_pair(x,y)].size() > 2)
		{
			glm::dvec3 vert = nvertices[x]-nvertices[y];
			double len = glm::length(vert);
			vert /= len;
			glm::dvec3 dir(len*0.5,len*0.5,len*0.5);
			dir = dir - glm::dot(dir,vert)*vert;
			if (!tree->Is_Exterior(nvertices[i]+dir))
			{
				dir = glm::cross(vert,dir);
			}
			dir = glm::normalize(dir);
			std::set<std::pair<int,int> >& p = ocs[i];
			for (std::set<std::pair<int,int> >::iterator it1 = p.begin();
				it1 != p.end(); ++it1)
			{
				assert(triangles[it1->first][it1->second] == i);
				if (glm::dot(nvertices[triangles[it1->first][(it1->second+1)%3]]-nvertices[i],dir)<0)
				{
					triangles[it1->first][it1->second] = (int)nvertices.size();
				}
			}
			nvertices[i] += dir * (0.5 * len);
			v_faces.push_back(v_faces[i]);
			nvertices.push_back(nvertices[i]);
			nvertices.back() -= len * dir;
		}
	}
	return true;
}

int Model_OBJ::is_manifold()
{
	std::map<std::pair<int,int>,std::list<glm::dvec3> > edges;
	std::vector<std::set<int> > graph(vertices.size());
	int flag = 0;
	for (int i = 0; i < (int)face_indices.size(); ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			int x = face_indices[i][j];
			int y = face_indices[i][(j + 1) % 3];
			if (x > y)
			{
				int temp = x;
				x = y;
				y = temp;
			}
			if (x >= (int)vertices.size() || y >= (int)vertices.size())
			{
				flag = -1;
			}
			std::pair<int,int> edge_id = std::make_pair(x,y);
			std::map<std::pair<int,int>,std::list<glm::dvec3> >::iterator it = edges.find(edge_id);
			if (it == edges.end())
			{
				std::list<glm::dvec3> l;
				l.push_back(vertices[face_indices[i][(j+2)%3]]);
				edges.insert(make_pair(edge_id,l));
			} else
			{
				if (it->second.size() == 2)
				{
					colors[x] = glm::dvec3(0,0,1);
					colors[y] = glm::dvec3(0,0,1);
					flag = -1;
				} else
				{
					it->second.push_back(vertices[face_indices[i][(j+2)%3]]);
					glm::dvec3 p1 = glm::cross(it->second.front()-vertices[x], vertices[y]-vertices[x]);
					glm::dvec3 p2 = glm::cross(it->second.back()-vertices[x],vertices[y]-vertices[x]);
					glm::dvec3 norm1 = glm::normalize(p1);
					glm::dvec3 norm2 = glm::normalize(p2);
					if (glm::dot(norm1,norm2)>1-1e-7 && glm::length(p1) > 1e-4 && glm::length(p2) > 1e-4)
					{
						flag = 0;
					}
				}
			}
		}
	}
//	colors[15160] = glm::dvec3(0,0,1);
//	colors[15159] = glm::dvec3(0,0,1);
	for (std::map<std::pair<int,int>,std::list<glm::dvec3> >::iterator edge_it = edges.begin();
		edge_it != edges.end(); ++edge_it)
	{
		if (edge_it->second.size() == 1)
		{
			flag = 1;
		}
	}
	return flag;
}

void Model_OBJ::SaveOBJ(const char* filename)
{
	std::ofstream os(filename);
	for (int i = 0; i < (int)vertices.size(); ++i)
	{
		os << "v " << vertices[i][0] << " " << vertices[i][1] << " " << vertices[i][2] << "\n";
	}
	for (int i = 0; i < (int)face_indices.size(); ++i)
	{
		os << "f " << face_indices[i][0] + 1 << " " << face_indices[i][1] + 1 << " " << face_indices[i][2] + 1 << "\n";
	}
	os.close();
}

void Model_OBJ::Save(const char* filename, bool color)
{
	std::ofstream os(filename);
	if (color)
		os << "COFF\n";
	else
		os << "OFF\n";
	os << vertices.size() << " " << face_indices.size() << " " << 0 << "\n";
	for (int i = 0; i < (int)vertices.size(); ++i)
	{
		if (color)
			os << vertices[i][0] << " " << vertices[i][1] << " " << vertices[i][2] << " " << (int)(colors[i][2]*255) << " " << (int)(colors[i][1]*255) << " " << (int)(colors[i][0]*255) << " 255\n";
		else
			os << vertices[i][0] << " " << vertices[i][1] << " " << vertices[i][2] << "\n";
	}
	double min_len = 1e30, max_len = -1e30;
	for (int i = 0; i < (int)face_indices.size(); ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			int x = face_indices[i][j];
			int y = face_indices[i][(j + 1) % 3];
			double len = glm::length(vertices[x] - vertices[y]);
			if (len < min_len)
				min_len = len;
			if (len > max_len)
				max_len = len;
		}
		os << "3 " << face_indices[i][0] << " " << face_indices[i][1] << " " << face_indices[i][2] << "\n";
	}
	os.close();
}
