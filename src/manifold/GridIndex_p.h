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
#ifndef MANIFOLD2_GRIDINDEX_H_
#define MANIFOLD2_GRIDINDEX_H_

#include "types_p.h"

struct GridIndex
{
public:
	GridIndex(){}
	GridIndex(int x, int y, int z)
	: id(x,y,z)
	{}
	bool operator<(const GridIndex& ind) const
	{
		int i = 0;
		while (i < 3 && id[i] == ind.id[i])
			i++;
		return (i < 3 && id[i] < ind.id[i]);
	}
	GridIndex operator+(const GridIndex& ind) const
	{
		GridIndex grid(*this);
		grid.id += ind.id;
		return grid;
	}
	GridIndex operator/(int x) const
	{
		return GridIndex(id[0]/x,id[1]/x,id[2]/x);
	}
	Vector3i id;
};

#endif