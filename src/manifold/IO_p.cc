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
#include "IO_p.h"

#include <igl/readOFF.h>
#include <fstream>
#include <vector>

void ReadOBJ(const char* filename, MatrixD* V, MatrixI* F) {
	int len = strlen(filename);
	if (strcmp(filename + (len - 3), "off") == 0) {
		igl::readOFF(filename, *V, *F);
		return;
	}
	char buffer[1024];
	std::ifstream is(filename);
	std::vector<Vector3> vertices;
	std::vector<Vector3i> faces;
	while (is >> buffer) {
		if (strcmp(buffer, "v") == 0) {
			Vector3 v;
			is >> v[0] >> v[1] >> v[2];
			vertices.push_back(v);
		}
		if (strcmp(buffer, "f") == 0) {
			Vector3i f;
			for (int j = 0; j < 3; ++j) {
				is >> buffer;
				int t = 0;
				int k = 0;
				while (buffer[k] >= '0' && buffer[k] <= '9') {
					t = t * 10 + (buffer[k] - '0');
					k += 1;
				}
				f[j] = t - 1;
			}
			faces.push_back(f);
		}
	}
	V->resize(vertices.size(), 3);
	F->resize(faces.size(), 3);
	memcpy(V->data(), vertices.data(), sizeof(Vector3) * vertices.size());
	memcpy(F->data(), faces.data(), sizeof(Vector3i) * faces.size());
}

void WriteOBJ(const char* filename, const MatrixD& V, const MatrixI& F) {
	std::ofstream os(filename);
	for (int i = 0; i < V.rows(); ++i) {
		auto& v = V.row(i);
		os << "v " << v[0] << " " << v[1] << " " << v[2] << "\n";
	}
	for (int i = 0; i < F.rows(); ++i) {
		auto& f = F.row(i);
		os << "f " << f[0] + 1 << " " << f[1] + 1 << " " << f[2] + 1 << "\n";
	}
	os.close();
}
