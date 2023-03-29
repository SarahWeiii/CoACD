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

#include "BVH.h"

void BVH::updateBVH(std::vector<BV*>& bvs, int dim, int l, int r) {
    left = 0, right = 0;
    axis = dim;
    if (l == r)
    {
        bv = bvs[l];
        return;
    }
    if (l < 0) {
        l = 0;
        r = (int)bvs.size() - 1;
    }
    num = r - l + 1;
    bv = new BV();
    bv->Include(bvs, l, r);
    double pivot = (*bv)(dim);
    int i = l, j = r;
    while (i < j) {
        while ((*bvs[i])(dim) < pivot) {
            ++i;
        }
        while ((*bvs[j])(dim) > pivot) {
            --j;
        }
        if (i <= j) {
            BV* bv = bvs[i];
            bvs[i] = bvs[j];
            bvs[j] = bv;
            ++i, --j;
        }
    }
    if (i > l + 1)
        --i;
    if (i > l)
    {
        left = new BVH();
        left->updateBVH(bvs, dim % 3, l, i - 1);
    }
    if (i <= r)
    {
        right = new BVH();
        right->updateBVH(bvs, dim % 3, i, r);
    }
}

std::pair<glm::dvec3,bool> BVH::rayIntersect(glm::dvec3& o, glm::dvec3& d)
{
    if (left == 0 && right == 0)
    {
        if (!bv->tris || !bv->HitBox(o, d))
            return std::make_pair(glm::dvec3(), false);
        return bv->rayIntersectsTriangle(o, d);
    }
    std::pair<glm::dvec3,bool> p1, p2;
    p1.second = false;
    p2.second = false;
    if (!bv->HitBox(o,d))
        return p1;
    if (left)
        p1 = left->rayIntersect(o, d);
    if (right)
        p2 = right->rayIntersect(o, d);
    if (!p2.second)
        return p1;
    if (!p1.second)
        return p2;
    if (glm::dot(p1.first-o,d)<glm::dot(p2.first-o,d))
        return p1;
    else
        return p2;
}