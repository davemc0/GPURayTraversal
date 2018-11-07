
#pragma once

#include "bvh/BVH.hpp"

#include <vector>

namespace FW
{
    typedef std::pair<float, FW::BVHNode*> OptPair_T;

    class BRefine
    {
    public:

        enum PriorityHeur {
            PRIORITY_AREA = 0,
            PRIORITY_MIN_RATIO = 1,
            PRIORITY_SUM_RATIO = 2,
            PRIORITY_ALL_THREE = 3,
            PRIORITY_MIN_AND_SUM = 4,
            PRIORITY_CYCLE = 5,
            PRIORITY_RANDOM = 6,
        };

        struct Params
        {
            PriorityHeur priorityMetric = PRIORITY_ALL_THREE;
            int maxLoops = 10;
            float batchSize = 0.01f;
            float timeBudget = 0.0f;
            float perPassImprovement = 0.0001f; // Abort pass loop if a pass has less than this percentage SAH improvement
            bool removeRandomChild = true;
            int nodesToRemove = 1;
        };

    public:
        BRefine(Refine& ref, Params& rparams);
        ~BRefine(void);

        void run();

        void setParams(Params& rparams) { m_rparams = rparams; }

    private:
        BRefine(const BRefine&); // forbidden
        BRefine& operator=(const BRefine&); // forbidden

        void computePriority(BVHNode* node);
        void remove1ForOpt(BVHNode* node); // Mine
        void remove2ForOpt(BVHNode* node); // Bittner
        void insertForOpt(BVHNode* node);
        BVHNode* findInsertTarget(BVHNode* node);

        // New algorithm
        void newComputePriority(BVHNode* node);
        OptPair_T newFindInsertTarget(BVHNode* node, BVHNode* treeletTop);
        float planRemoveReplace(BVHNode * node);
        OptPair_T computeModSAH(BVHNode* node, AABB modBounds, float modSAH, bool isShrink);

    private:
        BVH& m_bvh;
        Refine& m_refine;
        const Platform& m_platform;
        Params& m_rparams;

        std::vector<OptPair_T> m_toOptimize; // Float key is badness metric
        std::vector<OptPair_T> m_toRecycle; // Float key is unused
        std::vector<OptPair_T> m_toInsert; // Float key is node area
        std::vector<OptPair_T> m_toSearch; // Float key is induced cost XXX Should probably be a different container for O(1) insert/remove

        float m_rootArea; // Keep this for better optimization

        int m_nodesInserted;
    };

};
