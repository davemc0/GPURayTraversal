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

// #define FW_ENABLE_ASSERT

#include "bvh/BVH.hpp"
#include "bvh/SplitBVHBuilder.hpp"
#include "bvhcmpr/Refine.hpp"
#include "bvhcmpr/RefineBittner.hpp"

#include <bitset>
#include <iostream>

using namespace FW;

BVH::BVH(Scene* scene, const Platform& platform, const BuildParams& params)
{
    FW_ASSERT(scene);
    m_scene = scene;
    m_platform = platform;

    if (params.enablePrints)
        printf("BVH builder: %d tris, %d vertices\n", scene->getNumTriangles(), scene->getNumVertices());

	// Have one prim per leaf in SBVH, then refine it later
	int oldMinLeafSize = m_platform.getMinLeafSize();
	int oldMaxLeafSize = m_platform.getMaxLeafSize();
	m_platform.setLeafPreferences(1, 1);

    // XXX Disable splitting
    BuildParams sparams = params;
    sparams.splitAlpha = FW_F32_MAX;

    m_root = SplitBVHBuilder(*this, sparams).run();

	m_platform.setLeafPreferences(oldMinLeafSize, oldMaxLeafSize);

#if 0
	// XXX Prune Tree
	m_root->computeSubtreeValues(m_platform, m_root->getArea());
	while (dynamic_cast<InnerNode*>(m_root)->m_children[0]->m_tris >= 3)
		m_root = dynamic_cast<InnerNode*>(m_root)->m_children[0];
	m_root->computeSubtreeValues(m_platform, m_root->getArea());

	printTree(m_root);
#endif

	m_root->computeSubtreeValues(m_platform, m_root->getArea());

	if (params.enablePrints) {
		printf("BVH: Scene bounds: (%.1f,%.1f,%.1f) - (%.1f,%.1f,%.1f)\n", m_root->m_bounds.min().x, m_root->m_bounds.min().y, m_root->m_bounds.min().z,
			m_root->m_bounds.max().x, m_root->m_bounds.max().y, m_root->m_bounds.max().z);
		printf("top-down sah: %.6f\n", m_root->m_sah);
	}

    Refine::RefineParams rparams;
    rparams.nTrLeaves = 7;
    rparams.freezeThreshold = 6;
    rparams.maxLoops = 1;
    rparams.treeletEpsilon = 1e-8f;
    rparams.treeletHeuristic = Refine::TreeletHeur::TREELET_CYCLE;
    Refine Ref(*this, params, rparams);

    BRefine::BRefineParams bparams;
    BRefine BRef(*this, params, bparams);

    for(int x=0; x<100; x++)
    {
        BRef.run();

        Ref.run();
    }
    printf("Collapse1\n");
    Ref.collapseLeaves();
    printf("Collapse2\n");

	m_root->computeSubtreeValues(m_platform, m_root->getArea(), true);
	if (params.enablePrints)
		printf("leaf collapse sah: %.6f\n", m_root->m_sah);

	if (params.stats)
    {
        params.stats->SAHCost           = m_root->m_sah;
        params.stats->branchingFactor   = 2;
        params.stats->numLeafNodes      = m_root->getSubtreeSize(BVH_STAT_LEAF_COUNT);
        params.stats->numInnerNodes     = m_root->getSubtreeSize(BVH_STAT_INNER_COUNT);
        params.stats->numTris           = m_root->getSubtreeSize(BVH_STAT_TRIANGLE_COUNT);
        params.stats->numChildNodes     = m_root->getSubtreeSize(BVH_STAT_CHILDNODE_COUNT);
		params.stats->maxLeafDepth	    = m_root->getSubtreeSize(BVH_STAT_MAX_LEAF_DEPTH);
		params.stats->minLeafDepth	    = m_root->getSubtreeSize(BVH_STAT_MIN_LEAF_DEPTH);
		params.stats->mixedInnerNodes   = m_root->getSubtreeSize(BVH_STAT_MIXED_INNER_COUNT);
		params.stats->leafInnerNodes    = m_root->getSubtreeSize(BVH_STAT_LEAF_INNER_COUNT);
		params.stats->innerInnerNodes   = m_root->getSubtreeSize(BVH_STAT_INNER_INNER_COUNT);
	}
}

static S32 currentTreelet;
static Set<S32> uniqueTreelets;

void BVH::trace(RayBuffer& rays, RayStats* stats) const
{
    for(S32 i=0;i<rays.getSize();i++)
    {
        Ray ray = rays.getRayForSlot(i);    // takes a local copy
        RayResult& result = rays.getMutableResultForSlot(i);

        result.clear();

        currentTreelet = -2;
        uniqueTreelets.clear();

        if(stats)
        {
            stats->platform = m_platform;
            stats->numRays++;
        }

        traceRecursive(m_root, ray,result,rays.getNeedClosestHit(), stats);
    }
}

void BVH::traceRecursive(BVHNode* node, Ray& ray, RayResult& result,bool needClosestHit, RayStats* stats) const
{
    if(currentTreelet != node->m_treelet)
    {
        if(stats)
        {
//          if(!uniqueTreelets.contains(node->m_treelet))   // count unique treelets (comment this line to count all)
                stats->numTreelets++;
        }
        currentTreelet = node->m_treelet;
    }

    if(node->isLeaf())
    {
        const LeafNode* leaf = reinterpret_cast<const LeafNode*>(node);
        const Vec3i* triVtxIndex = (const Vec3i*)m_scene->getTriVtxIndexBuffer().getPtr();
        const Vec3f* vtxPos = (const Vec3f*)m_scene->getVtxPosBuffer().getPtr();

        if(stats)
            stats->numTriangleTests += m_platform.roundToTriangleBatchSize( leaf->getNumTriangles() );

        for(int i=leaf->m_lo; i<leaf->m_hi; i++)
        {
            S32 index = m_triIndices[i];
            const Vec3i& ind = triVtxIndex[index];
            const Vec3f& v0 = vtxPos[ind.x];
            const Vec3f& v1 = vtxPos[ind.y];
            const Vec3f& v2 = vtxPos[ind.z];
            Vec3f bary = Intersect::RayTriangle(v0,v1,v2, ray);
            float t = bary[2];

            if(t>ray.tmin && t<ray.tmax)
            {
                ray.tmax    = t;
                result.t    = t;
                result.id   = index;

                if(!needClosestHit)
                    return;
            }
        }
    }
    else
    {
        if(stats)
            stats->numNodeTests += m_platform.roundToNodeBatchSize( node->getNumChildNodes() );

        const int TMIN = 0;
        const int TMAX = 1;
        const InnerNode* inner = reinterpret_cast<const InnerNode*>(node);
        BVHNode* child0 = inner->m_children[0];
        BVHNode* child1 = inner->m_children[1];
        Vec2f tspan0 = Intersect::RayBox(child0->m_bounds, ray);
        Vec2f tspan1 = Intersect::RayBox(child1->m_bounds, ray);
        bool intersect0 = (tspan0[TMIN]<=tspan0[TMAX]) && (tspan0[TMAX]>=ray.tmin) && (tspan0[TMIN]<=ray.tmax);
        bool intersect1 = (tspan1[TMIN]<=tspan1[TMAX]) && (tspan1[TMAX]>=ray.tmin) && (tspan1[TMIN]<=ray.tmax);

        if(intersect0 && intersect1)
        if(tspan0[TMIN] > tspan1[TMIN])
        {
            swap(tspan0,tspan1);
            swap(child0,child1);
        }

        if(intersect0)
            traceRecursive(child0,ray,result,needClosestHit,stats);

        if(result.hit() && !needClosestHit)
            return;

//      if(tspan1[TMIN] <= ray.tmax)    // this test helps only about 1-2%
        if(intersect1)
            traceRecursive(child1,ray,result,needClosestHit,stats);
    }
}

void BVH::printTree(BVHNode* node, int level)
{
	for (int e = 0; e < level; e++)
		printf(" ");
	printf("0x%08x: prob=%f area=%f sah=%f tris=%d froz=%d ", (U64)node, node->m_probability, node->getArea(), node->m_sah, node->m_tris, node->m_frozen);
	if (node->isLeaf()) {
		LeafNode* l = dynamic_cast<LeafNode*>(node);
		printf("L: %d %d\n", l->m_lo, l->m_hi);
	}
	else {
		InnerNode* i = dynamic_cast<InnerNode*>(node);
		printf("I:0x%08x 0x%08x\n", i->getChildNode(0), i->getChildNode(1));
		printTree(i->getChildNode(0), level + 1);
		printTree(i->getChildNode(1), level + 1);
	}
}