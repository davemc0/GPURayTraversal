
#pragma once

#include "bvh/BVH.hpp"
#include "base/Timer.hpp"

#include <vector>

namespace FW
{

    class Refine
    {
    public:
        Refine(BVH& bvh);
        ~Refine(void);

        void runBestAdversarial();
        void runBestOrderedRandom();
        void runBestNoSplitsPrimPerLeaf();
        void runBestNoSplitsLeafCollapse();
        void runBestSplitsPrimPerLeaf();
        void runBestSplitsLeafCollapse();
        void runExtremeBittner();
        void runTraditionalBittner();
        void runExtremeTRBVH();
        void runGrowingTRBVH();
        void runQuickAndClean();
        void runTest();

        BVH& getBVH() { return m_bvh; }
        Timer& getTimer() { return m_progressTimer; }

        static const U32 LEAF_FLAG = 0x80000000;

    private:
        Refine(const Refine&); // forbidden
        Refine& operator=(const Refine&); // forbidden

        template<class Pr>
        void formTreeletPred(BVHNode* troot, int nTrLeaves, std::vector<BVHNode*>& internals, std::vector<BVHNode*>& leaves, Pr Pred);

        void formTreelet(BVHNode* tRoot, int nTrLeaves, std::vector<BVHNode*>& internals, std::vector<BVHNode*>& leaves);

        void collapseLeaves();

        BVHNode* Refine::collapseLeavesRecursive(BVHNode* node, Array<S32>& tris);

        float checkTree(bool recomputeBounds, bool resetFrozen);

    private:
        BVH& m_bvh;
        const Platform& m_platform;

        Timer m_progressTimer;
    };

};
