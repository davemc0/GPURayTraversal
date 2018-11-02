#pragma once
#include "bvh/BVH.hpp"
#include "base/Timer.hpp"

namespace FW
{
    //------------------------------------------------------------------------

    class RandomBVHBuilder
    {
    private:
        enum
        {
            MaxDepth = 64,
            MaxSpatialDepth = 48,
            NumSpatialBins = 128,
        };

        struct Reference
        {
            S32 triIdx;
            AABB bounds;

            Reference(void) : triIdx(-1) {}
        };

        struct NodeSpec
        {
            S32 numRef;
            AABB bounds;

            NodeSpec(void) : numRef(0) {}
        };

    public:
        RandomBVHBuilder(BVH& bvh, const BVH::BuildParams& params, bool randomize);
        ~RandomBVHBuilder(void);

        BVHNode* run(void);

    private:
        BVHNode* buildNode(int left, int right);
        void randomizeArray(Array<S32>& ar);

    private:
        RandomBVHBuilder(const RandomBVHBuilder&); // forbidden
        RandomBVHBuilder& operator= (const RandomBVHBuilder&); // forbidden

    private:
        BVH& m_bvh;
        const Platform& m_platform;
        const BVH::BuildParams& m_params;

        Array<Reference> m_refStack;
        F32 m_minOverlap;
        bool m_randomizeLeaves;

        Timer m_progressTimer;
        S32 m_numDuplicates;
    };

    //------------------------------------------------------------------------
}
