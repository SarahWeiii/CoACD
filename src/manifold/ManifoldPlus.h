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
#include <stdlib.h>
#include <stdio.h>
#include <fstream>
#include <vector>

#include <igl/readOBJ.h>
#include <igl/writeOBJ.h>

#include "IO_p.h"
#include "Manifold_p.h"
#include "types_p.h"
#include "../model_obj.h"

int ManifoldPlus(ofstream& of, string input_model, Model& output, int depth=8)
{
  clock_t start, end;
  MatrixD V, out_V;
	MatrixI F, out_F;
	ReadOBJ(input_model.c_str(), &V, &F);

  of << " - Pre-processing (manifoldPlus)" << endl;
  of << "\tdepth: " << depth << endl;
  cout << " - Pre-processing (manifoldPlus)" << endl;
  cout << "\tdepth: " << depth << endl;

	ManifoldP manifold;
	start = clock();
  manifold.ProcessManifold(V, F, depth, &out_V, &out_F);
  end = clock();
  of << "ManifoldPlus time: " << double(end-start)/CLOCKS_PER_SEC << "s" << endl;
  cout << "ManifoldPlus time: " << double(end-start)/CLOCKS_PER_SEC << "s" << endl;

  output.Load(out_V, out_F);

	return 0;
}
