
#pragma once

#include "bvh/BVH.hpp"

#include <vector>

namespace FW
{
    typedef std::pair<float, FW::BVHNode*> OptPair_T;

    class BRefine
    {
    public:

        struct Params
        {
            int maxLoops = 10;
            float batchSize = 0.01f;
            float timeBudget = 0.0f;
            float perPassImprovement = 0.0001f; // Abort pass loop if a pass has less than this SAH improvement
            bool chooseRandomNode = false;
            bool removeRandomChild = true;
            int nodesToRemove = 1;
        };

    public:
        BRefine(Refine& ref, Params& rparams);
        ~BRefine(void);

        void run();

        void collapseLeaves();

        void setParams(Params& rparams) { m_rparams = rparams; }

    private:
        BRefine(const BRefine&); // forbidden
        BRefine& operator=(const BRefine&); // forbidden

        void computePriority(BVHNode* node);

        bool refineNode(BVHNode* node); // The recursive call; returns true if it made progress

        void remove1ForOpt(BVHNode* node); // Mine
        void remove2ForOpt(BVHNode* node); // Bittner
        void insertForOpt(BVHNode* node);
        void planRemoveReplace(BVHNode * node);
        BVHNode* findInsertTarget(BVHNode* node);

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
