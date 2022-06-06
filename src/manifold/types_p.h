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

#ifndef MANIFOLD2_TYPES_H_
#define MANIFOLD2_TYPES_H_

#include <Eigen/Core>

typedef double FT;

typedef Eigen::Matrix<FT, 3, 1> Vector3;
typedef Eigen::Matrix<FT, 4, 1> Vector4;
typedef Eigen::Matrix<FT, Eigen::Dynamic, 1> VectorX;

typedef Eigen::Vector3i Vector3i;
typedef Eigen::Matrix<int, 4, 1> Vector4i;
typedef Eigen::VectorXi VectorXi;

typedef Eigen::Matrix<FT,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> MatrixD;
typedef Eigen::Matrix<int,Eigen::Dynamic,Eigen::Dynamic
	,Eigen::RowMajor> MatrixI;

#endif