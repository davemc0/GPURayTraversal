/*
 *  Copyright (c) 2009-2011, NVIDIA Corporation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *      * Neither the name of NVIDIA Corporation nor the
 *        names of its contributors may be used to endorse or promote products
 *        derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "bvh/BVHNode.hpp"

namespace FW
{
    // This is where the statics live.
    ArrayAllocator<InnerNode>* InnerNode::s_AA = nullptr;
    ArrayAllocator<LeafNode>*  LeafNode::s_AA  = nullptr;

int BVHNode::getSubtreeSize(BVH_STAT stat) const
{
    int cnt;
    switch(stat)
    {
        default: FW_ASSERT(0);  // unknown mode
        case BVH_STAT_NODE_COUNT:        cnt = 1; break;
        case BVH_STAT_LEAF_COUNT:        cnt = isLeaf() ? 1 : 0; break;
        case BVH_STAT_INNER_COUNT:       cnt = isLeaf() ? 0 : 1; break;
        case BVH_STAT_TRIANGLE_COUNT:    cnt = isLeaf() ? reinterpret_cast<const LeafNode*>(this)->getNumTriangles() : 0; break;
        case BVH_STAT_CHILDNODE_COUNT:   cnt = getNumChildNodes(); break;
		case BVH_STAT_MAX_LEAF_DEPTH:
			cnt = 0;
			if (!isLeaf()) {
				for (int i = 0; i<getNumChildNodes(); i++) {
					S32 x = getChildNode(i)->getSubtreeSize(stat) + 1;
					if (x > cnt) cnt = x;
				}
			}
			return cnt;
		case BVH_STAT_MIN_LEAF_DEPTH:
			if (!isLeaf()) {
				cnt = 0x7fffffff;
				for (int i = 0; i<getNumChildNodes(); i++) {
					S32 x = getChildNode(i)->getSubtreeSize(stat) + 1;
					if (x < cnt) cnt = x;
				}
			} else
				cnt = 0;
			return cnt;
		case BVH_STAT_MIXED_INNER_COUNT:
			if (isLeaf()) return 0;
			{
				bool cl = getChildNode(0)->isLeaf() ? true : false;
				cnt = 0;
				for (int i = 1; i < getNumChildNodes(); i++) {
					if (cl != (getChildNode(i)->isLeaf() ? true : false))
						cnt = 1;
				}
			}
			break;
		case BVH_STAT_LEAF_INNER_COUNT:
			if (isLeaf()) return 0;
			cnt = 1;
			for (int i = 0; i<getNumChildNodes(); i++) {
				if (!getChildNode(i)->isLeaf())
					cnt = 0;
			}
			break;
		case BVH_STAT_INNER_INNER_COUNT:
			if (isLeaf()) return 0;
			cnt = 1;
			for (int i = 0; i<getNumChildNodes(); i++) {
				if (getChildNode(i)->isLeaf())
					cnt = 0;
			}
			break;
	}

    if(!isLeaf())
    {
        for(int i=0;i<getNumChildNodes();i++)
            cnt += getChildNode(i)->getSubtreeSize(stat);
    }

    return cnt;
}


void BVHNode::deleteSubtree()
{
    for(int i=0;i<getNumChildNodes();i++)
        getChildNode(i)->deleteSubtree();

    delete this;
}

// SAH = node cost * root cond prob + SAH of direct children (which includes all descendants)
void BVHNode::computeSubtreeValues(const Platform& p, const float rootArea, bool recomputeBounds, bool resetFrozen)
{
    for (int i = 0; i < getNumChildNodes(); i++) {
        BVHNode* ch = getChildNode(i);
        ch->m_parent = this;
        ch->computeSubtreeValues(p, rootArea, recomputeBounds, resetFrozen);
    }

    computeValues(p, rootArea, recomputeBounds, resetFrozen);
}

//-------------------------------------------------------------

void BVHNode::computeValues(const Platform& p, const float rootArea, bool recomputeBounds, bool resetFrozen)
{
    // For inner nodes recompute bounds; for leaves don't.
    if (!isLeaf() && recomputeBounds)
        m_bounds = AABB();

    m_sah = 0;
    m_tris = 0;
    for (int i = 0; i < getNumChildNodes(); i++) {
        BVHNode* ch = getChildNode(i);
        m_sah += ch->m_sah;
        m_tris += ch->m_tris;
        if (!isLeaf() && recomputeBounds)
            m_bounds.grow(ch->m_bounds);
    }

    m_probability = m_bounds.area() / rootArea;
    m_tris += getNumTriangles();
    m_sah += m_probability * p.getCost(getNumChildNodes(), getNumTriangles());
    m_treelet = 0;
    if (resetFrozen)
        m_frozen = 0;
    FW_ASSERT(m_tris > 0);
}

S32 BVHNode::getChildIndex(const BVHNode* ch) const
{
    for (int ni = 0; ni < getNumChildNodes(); ni++)
        if (getChildNode(ni) == ch)
            return ni;

    FW_ASSERT(0 && "Neither child is ch");

    return -1;
}

//-------------------------------------------------------------

void assignIndicesDepthFirstRecursive( BVHNode* node, S32& index, bool includeLeafNodes )
{
    if(node->isLeaf() && !includeLeafNodes)
        return;

    node->m_index = index++;
    for (int i = 0; i < node->getNumChildNodes(); i++)
        assignIndicesDepthFirstRecursive(node->getChildNode(i), index, includeLeafNodes);
}

void BVHNode::assignIndicesDepthFirst( S32 index, bool includeLeafNodes )
{
    assignIndicesDepthFirstRecursive( this, index, includeLeafNodes );
}

//-------------------------------------------------------------

void BVHNode::assignIndicesBreadthFirst( S32 index, bool includeLeafNodes )
{
    Array<BVHNode*> nodes;
    nodes.add(this);
    S32 head=0;

    while(head < nodes.getSize())
    {
        // pop
        BVHNode* node = nodes[head++];

        // discard
        if(node->isLeaf() && !includeLeafNodes)
            continue;

        // assign
        node->m_index = index++;

        // push children
        for(int i=0;i<node->getNumChildNodes();i++)
            nodes.add(node->getChildNode(i));
    }
}


} //

