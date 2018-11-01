
#pragma once

#include "bvh/BVH.hpp"
#include "base/Timer.hpp"

#include <vector>

namespace FW
{
    const int maxTrInternals = 31;
    const int maxTrLeaves = 32;

    class TRefine
    {
    public:
        enum TreeletHeur {
            TREELET_RANDOM = 0,
            TREELET_GREATER = 1, // Choose largest leaves
            TREELET_TRUE = 2,
            TREELET_CYCLE = 3,
            TREELET_FALSE = 4,
            TREELET_LESS = 5 // Choose smallest leaves
        };

        struct RefineParams
        {
            TreeletHeur treeletHeuristic = TREELET_CYCLE;
            int maxLoops = 6;
            int nTrLeaves = 14;
            int nTrInternals = nTrLeaves - 1;
            int freezeThreshold = 15; // How many passes a treelet must have no improvement before it's no longer traversed
            int maxNoProgressPasses = 4; // Number of sequential passes with 0 refinements before terminating
            float treeletEpsilon = 1e-7f; // How much a treelet's SAH must improve by to be written out
            float epsilonScale = 1.0f; // How much epsilon grows after each pass
            float gamma = (float)nTrLeaves; // How many triangles must be in the treelet to attempt optimization
            float gammaScale = 1.0f; // How much gamma grows after each pass
            bool leafCollapsePass = false;
        };

        struct Statistics
        {
            int optVisits; // == optFailFrozen + optFailTooSmall + optFailIsLeaf + optFailNoTreelet + optAttempts
            int optFailIsLeaf;
            int optFailTooSmall;
            int optFailFrozen;
            int optFailNoTreelet;
            int optAttempts; // == optSuccess + optFailNoImprovement
            int optFailNoImprovement;
            int optSuccess;

            void print()
            {
                printf("v=%d l=%d ts=%d f=%d nt=%d a=%d ni=%d s=%d ", optVisits, optFailIsLeaf, optFailTooSmall,
                    optFailFrozen, optFailNoTreelet, optAttempts, optFailNoImprovement, optSuccess);
            }
        };

    public:
        TRefine(BVH& bvh, const BVH::BuildParams& params, RefineParams& rparams);
        ~TRefine(void);

        void run();

        void collapseLeaves();

        void setParams(RefineParams& rparams) { m_rparams = rparams; }

    private:
        TRefine(const TRefine&); // forbidden
        TRefine& operator=(const TRefine&); // forbidden

        template<class Pr>
        void formTreeletPred(BVHNode* troot, int nTrLeaves, std::vector<BVHNode*>& internals, std::vector<BVHNode*>& leaves, Pr Pred);

        void formTreelet(BVHNode* tRoot, int nTrLeaves, std::vector<BVHNode*>& internals, std::vector<BVHNode*>& leaves);

        BVHNode* formNodes(std::vector<BVHNode*>& internals, std::vector<BVHNode*>& leaves, U32 s);

        bool refineTreelet(BVHNode* troot); // The helper called by refineNode; returns true if it made progress
        bool refineTreeletKensler(BVHNode* tRoot);

        bool refineNode(BVHNode* node); // The recursive call; returns true if it made progress

        BVHNode* TRefine::collapseLeavesRecursive(BVHNode* node, Array<S32>& tris);
        const U32 LEAF_FLAG = 0x80000000;

    private:
        BVH& m_bvh;
        const Platform& m_platform;
        const BVH::BuildParams& m_params;
        RefineParams& m_rparams;

        std::vector<float> m_sahes;
        std::vector<float> m_copt; // Cost of a subtree, not scaled by root area
        std::vector<U32> m_popt;

        TreeletHeur m_treeletHeur;
        bool m_collapseToLeaves;
        int m_freezeThreshold;

        Statistics m_stats;

        Timer m_progressTimer;

    };

};
