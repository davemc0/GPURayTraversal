
#pragma once

#include "bvh/BVH.hpp"
#include "base/Timer.hpp"

#include <vector>

namespace FW
{
    typedef std::pair<float, FW::BVHNode*> OptPair_T;

    class BRefine
    {
    public:

        struct BRefineParams
        {
            int maxLoops = 10;
            float batchSize = 0.01f;
            float timeBudget = 0.0f;
        };

    public:
        BRefine(BVH& bvh, const BVH::BuildParams& params, BRefineParams& rparams);
        ~BRefine(void);

        void run();

        void collapseLeaves();

        void setParams(BRefineParams& rparams) { m_rparams = rparams; }

    private:
        BRefine(const BRefine&); // forbidden
        BRefine& operator=(const BRefine&); // forbidden

        void computePriority(BVHNode* node);

        bool refineNode(BVHNode* node); // The recursive call; returns true if it made progress

        void removeForOpt(BVHNode* node);
        void insertForOpt(BVHNode* node);
        BVHNode* findInsertTarget(BVHNode* node);

    private:

        BVH& m_bvh;
        const Platform& m_platform;
        const BVH::BuildParams& m_params;
        BRefineParams& m_rparams;

        std::vector<OptPair_T> m_toOptimize; // Float key is badness metric
        std::vector<OptPair_T> m_toRecycle; // Float key is unused
        std::vector<OptPair_T> m_toInsert; // Float key is node area
        std::vector<OptPair_T> m_toSearch; // Float key is induced cost XXX Should probably be a different container for O(1) insert/remove

        float m_rootArea; // Keep this for better optimization

        Timer m_progressTimer;

        int m_nodesInserted;
    };

};
