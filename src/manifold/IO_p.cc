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
#include <stdint.h>

#include <igl/readOFF.h>
#include <fstream>
#include <vector>
#include <iostream>

bool ReadOBJ(const char* filename, MatrixD* V, MatrixI* F) {
	std::vector<Vector3> points;
	std::vector<Vector3i> triangles;
	const unsigned int BufferSize = 1024;
    FILE *fid = fopen(filename, "r");

    if (fid)
    {
        char buffer[BufferSize];
        int ip[4];
        double x[3];
        char *pch;
        char *str;
        while (!feof(fid))
        {
            if (!fgets(buffer, BufferSize, fid))
            {
                break;
            }
            else if (buffer[0] == 'v')
            {
                if (buffer[1] == ' ')
                {
                    str = buffer + 2;
                    for (int k = 0; k < 3; ++k)
                    {
                        pch = strtok(str, " ");
                        if (pch)
                            x[k] = (double)atof(pch);
                        else
                        {
                            return false;
                        }
                        str = NULL;
                    }
                    points.push_back({x[0], x[1], x[2]});
                }
            }
            else if (buffer[0] == 'f')
            {

                pch = str = buffer + 2;
                int k = 0;
                while (pch)
                {
                    pch = strtok(str, " ");
                    if (pch && *pch != '\n')
                    {
                        ip[k++] = (unsigned int)atoi(pch) - 1;
                    }
                    else
                    {
                        break;
                    }
                    str = NULL;
                }
                if (k == 3)
                {
                    triangles.push_back({ip[0], ip[1], ip[2]});
                }
                else if (k == 4)
                {
                    triangles.push_back({ip[0], ip[1], ip[2]});
                    triangles.push_back({ip[0], ip[2], ip[3]});
                }
            }
        }
	 	fclose(fid);
    }
    else
    {
        std::cout << "Open File Error!" << std::endl;
        return false;
    }
	V->resize(points.size(), 3);
	F->resize(triangles.size(), 3);
	memcpy(V->data(), points.data(), sizeof(Vector3) * points.size());
	memcpy(F->data(), triangles.data(), sizeof(Vector3i) * triangles.size());
	return true;
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
