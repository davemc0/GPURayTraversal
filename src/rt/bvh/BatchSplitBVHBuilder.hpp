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
    };
}
