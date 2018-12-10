#pragma once

#include "bvh/BVH.hpp"

namespace FW
{
    class BatchSplitBVHBuilder
    {
    public:
        enum
        {
            MaxDepth = 64,
            MaxSpatialDepth = 48,
            NumSpatialBins = 128,
        };

        enum {
            stratBitOffset    = 28,
            stratSpatialSplit = (0x4 << stratBitOffset), // Sort spatial split segments to the left
            stratObjectSplit  = (0x8 << stratBitOffset),
            stratLeaf         = (0xc << stratBitOffset), // Sort leaf segments to the right

            stratMask         = (0xf << stratBitOffset),
            stratDimMask      = (0x3 << stratBitOffset),
            stratNumMask      = (( 1 << stratBitOffset) - 1),
        };

        void initBBArrays(S32 maxN, FW::Scene * scene, FW::BVH & bvh);

        void freeArrays();

        void doGeneration(S32 & N, S32 & nSegments, S32 level);

        BVHNode * makeNodes(S32 N);

        BVHNode * batchRun(BatchSplitBVHBuilder & BS, AABB & rootBounds);

        BatchSplitBVHBuilder(FW::BVH& bvh, const FW::BVH::BuildParams& params);
        ~BatchSplitBVHBuilder(void);

        FW::BVHNode* run(void);

    private:
        BatchSplitBVHBuilder(const BatchSplitBVHBuilder&); // forbidden
        BatchSplitBVHBuilder& operator= (const BatchSplitBVHBuilder&); // forbidden

    public:
        FW::BVH& m_bvh;
        const FW::Platform& m_platform;
        const FW::BVH::BuildParams& m_params;

    private:

        // Data structures owned by namespace
        Array<S32>       m_intArray;    // x9 (rightIdx, leftIdx, gamma, segIdx, outIdxBest, outIdxNew, outCostBest, outCostNew, outStrategy)
        Array<AABB>      m_boundsArray; // x3 (refBounds, rightBounds, leftBounds)
        Array<U64>       m_keysArray;   // x2 (keys, keysOut)

        // Raw pointers for GPU arrays; initialized in initBBArrays(); all allocated using cudaMallocManaged
        S32*  m_rightIdx;
        S32*  m_leftIdx;
        S32*  m_gamma;
        S32*  m_segIdx;
        S32*  m_outIdxBest;
        S32*  m_outIdxNew;
        F32*  m_outCostBest;
        F32*  m_outCostNew;
        S32*  m_outStratCount;
        AABB* m_refBounds;
        AABB* m_rightBounds;
        AABB* m_leftBounds;
        U64*  m_keys;
        U64*  m_outKeys;

        const Vec3i* m_tris;
        const Vec3f* m_verts;
        S32*  m_refTriIdx;

        // Data owned by namespace
        F32 m_minOverlap;
        S32 m_numDuplicates;

    };
}
