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

#pragma once

#include "base/Array.hpp"
#include "bvh/Platform.hpp"
#include "Util.hpp"
#include "base/ArrayAllocator.hpp"

namespace FW
{

// TODO: remove m_probability. Node needed after all?

enum BVH_STAT
{
    BVH_STAT_NODE_COUNT,
    BVH_STAT_INNER_COUNT,
    BVH_STAT_LEAF_COUNT,
    BVH_STAT_TRIANGLE_COUNT,
	BVH_STAT_CHILDNODE_COUNT,
	BVH_STAT_MAX_LEAF_DEPTH,
	BVH_STAT_MIN_LEAF_DEPTH,
	BVH_STAT_MIXED_INNER_COUNT, // How many interior nodes have one leaf child and one interior child
	BVH_STAT_LEAF_INNER_COUNT, // How many interior nodes have two leaf children
	BVH_STAT_INNER_INNER_COUNT, // How many interior nodes have two interior children
};

class BVHNode
{
public:
	BVHNode() : m_probability(1.f), m_treelet(-1), m_index(-1), m_sah(0), m_tris(0), m_frozen(0), m_parent(nullptr) {}
    virtual ~BVHNode() { m_index = 0; } // XXX Mark node as deleted so it doesn't get read on GPU

    virtual bool        isLeaf() const = 0;
    virtual S32         getNumChildNodes() const = 0;
    virtual BVHNode*    getChildNode(S32 i) const = 0;
    virtual S32         getNumTriangles() const { return 0; }

    float       getArea() const     { return m_bounds.area(); }

    AABB        m_bounds;

    // These are somewhat experimental, for some specific test and may be invalid...
    float       m_probability;          // probability of coming here (widebvh uses this)
    float       m_sah;                  // SAH cost of this node and all child nodes

    int         m_treelet;              // for queuing tests (qmachine uses this)
    int         m_index;                // in linearized tree (qmachine uses this)
	int         m_tris;                 // number of triangles in the subtree (Refine uses this)
	int         m_frozen;               // How many passes since this subtree has had improvement

	BVHNode*    m_parent;               // Pointer to parent

    // Subtree functions
    int     getSubtreeSize(BVH_STAT stat=BVH_STAT_NODE_COUNT) const;
    void    computeSubtreeValues(const Platform& p, const float rootArea, bool recomputeBounds = false, bool resetFrozen = true); // Fills in m_probability, m_sah, m_tris
    void    deleteSubtree();

    void    assignIndicesDepthFirst  (S32 index=0, bool includeLeafNodes=true);
    void    assignIndicesBreadthFirst(S32 index=0, bool includeLeafNodes=true);

	void    computeValues(const Platform& p, const float rootArea, bool recomputeBounds, bool resetFrozen);  // Recompute all stats based on child data WITHOUT RECURSING
    S32     getChildIndex(const BVHNode* ch) const; // Returns the index of child ch in this

};


class InnerNode : public BVHNode
{
public:
    InnerNode(const AABB& bounds = AABB(), BVHNode* child0 = nullptr, BVHNode* child1 = nullptr) { m_bounds = bounds; m_children[0] = child0; m_children[1] = child1; }

    bool        isLeaf() const                  { return false; }
    S32         getNumChildNodes() const        { return 2; }
    BVHNode*    getChildNode(S32 i) const       { FW_ASSERT(i>=0 && i<2); return m_children[i]; }

    BVHNode*    m_children[2];

    void* operator new(size_t size)             { FW_ASSERT(s_AA); return s_AA->alloc(size); }
    void* operator new[](size_t size)           { FW_ASSERT(s_AA); return s_AA->alloc(size); }
    void operator delete(void* ptr)             { FW_ASSERT(s_AA); return s_AA->free((InnerNode*)ptr); }
    void operator delete[](void* ptr)           { FW_ASSERT(s_AA); return s_AA->free((InnerNode*)ptr); }
    static ArrayAllocator<InnerNode>*           s_AA;
};


class LeafNode : public BVHNode
{
public:
    LeafNode(const AABB& bounds = AABB() , S32 lo = -1, S32 hi = -1) { m_bounds = bounds; m_lo = lo; m_hi = hi; }
    LeafNode(const LeafNode& s)                 { *this = s; }

    bool        isLeaf() const                  { return true; }
    S32         getNumChildNodes() const        { return 0; }
    BVHNode*    getChildNode(S32) const         { return NULL; }

    S32         getNumTriangles() const         { return m_hi-m_lo; }
    S32         m_lo;
    S32         m_hi;

    void* operator new(size_t size)             { FW_ASSERT(s_AA); return s_AA->alloc(size); }
    void* operator new[](size_t size)           { FW_ASSERT(s_AA); return s_AA->alloc(size); }
    void operator delete(void* ptr)             { FW_ASSERT(s_AA); return s_AA->free((LeafNode*)ptr); }
    void operator delete[](void* ptr)           { FW_ASSERT(s_AA); return s_AA->free((LeafNode*)ptr); }
    static ArrayAllocator<LeafNode>*            s_AA;
};

} //
