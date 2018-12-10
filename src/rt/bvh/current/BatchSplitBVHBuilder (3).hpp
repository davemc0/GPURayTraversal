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
        Array<S32>       m_intArray;    // x9 refRightIdx, refLeftIdx, refGamma, refSegIdx, segIdxBest, segIdxNew, segCostBest, segCostNew, segStratRegIdx
        Array<AABB>      m_boundsArray; // x3 refBounds, refRightBounds, refLeftBounds
        Array<U64>       m_keysArray;   // x2 refKeys, segKeys

        // Raw pointers for GPU arrays; initialized in initBBArrays(); all allocated using cudaMallocManaged
        S32*  m_refRightIdx;    // each ref's distance to end of segment
        S32*  m_refLeftIdx;     // each ref's distance to start of segment
        S32*  m_refGamma;       // distance to the split location of a segment that starts or ends here; persists across iterations; OPT: Could I write directly into gamma in the scan?
        S32*  m_refSegIdx;      // each ref's index into segment list
        AABB* m_refBounds;      // each ref's bounding box; persists across iterations
        AABB* m_refRightBounds; // bounding box of stuff to right of each ref in segment
        AABB* m_refLeftBounds;  // bounding box of stuff to left of each ref in segment
        U64*  m_refKeys;        // each ref's bit trail / segment key; persists across iterations

        S32*  m_segIdxBest;     // each segment's best split location from start of segment
        S32*  m_segIdxNew;      // each segment's challenger split location from start of segment
        F32*  m_segCostBest;    // each segment's best split cost
        F32*  m_segCostNew;     // each segment's challenger split cost
        S32*  m_segStratRefIdx; // each segment's split strategy and index of first reference in segment
        U64*  m_segKeys;        // each segment's bit trail / segment key

        const Vec3i* m_tris;
        const Vec3f* m_verts;
        S32*  m_refTriIdx;      // persists across iterations

        // Data owned by namespace
        F32 m_minOverlap;
        S32 m_numDuplicates;

    };
}
