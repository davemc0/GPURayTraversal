
#pragma once

#include "bvh/BVH.hpp"
#include "base/Timer.hpp"

#include <vector>

namespace FW
{
    const int maxTrInternals = 31;
    const int maxTrLeaves = 32;

    class Refine
    {
    public:
        enum TreeletHeur {
            TREELET_RANDOM = 0,
            TREELET_GREATER = 1,
            TREELET_TRUE = 2,
            TREELET_CYCLE = 3,
            TREELET_FALSE = 4,
            TREELET_LESS = 5
        };

        struct RefineParams
        {
            TreeletHeur treeletHeuristic = TREELET_CYCLE;
            int         maxLoops = 6;
            int         nTrLeaves = 14;
            int         nTrInternals = nTrLeaves - 1;
            int         freezeThreshold = 15;
            float       treeletEpsilon = 1e-7f;
        };

    public:
        Refine(BVH& bvh, const BVH::BuildParams& params, RefineParams& rparams);
        ~Refine(void);

        void                run();

        void                collapseLeaves();

        void setParams(RefineParams& rparams) { m_rparams = rparams; }

    private:
        Refine(const Refine&); // forbidden
        Refine& operator=(const Refine&); // forbidden

        template<class Pr>
        void formTreeletPred(BVHNode* troot, int nTrLeaves, std::vector<BVHNode*>& internals, std::vector<BVHNode*>& leaves, Pr Pred);

        void formTreelet(BVHNode* tRoot, int nTrLeaves, std::vector<BVHNode*>& internals, std::vector<BVHNode*>& leaves);

        BVHNode* formNodes(std::vector<BVHNode*>& internals, std::vector<BVHNode*>& leaves, U32 s);

        bool refineTreelet(BVHNode* troot); // The helper called by refineNode; returns true if it made progress
        bool refineTreeletKensler(BVHNode* tRoot);

        bool refineNode(BVHNode* node); // The recursive call; returns true if it made progress

        BVHNode* Refine::collapseLeavesRecursive(BVHNode* node, Array<S32>& tris);
        const U32 LEAF_FLAG = 0x80000000;

    private:
        BVH&                    m_bvh;
        const Platform&         m_platform;
        const BVH::BuildParams& m_params;
        RefineParams&     m_rparams;

        std::vector<float>      m_sahes;
        std::vector<float>      m_copt; // Cost of a subtree, not scaled by root area
        std::vector<U32>        m_popt;

        TreeletHeur             m_treeletHeur;
        int                     m_optTreeletsDone;
        int                     m_optTreeletsOutput;
        bool                    m_collapseToLeaves;
        int                     m_freezeThreshold;

        Timer                   m_progressTimer;

    };

};
