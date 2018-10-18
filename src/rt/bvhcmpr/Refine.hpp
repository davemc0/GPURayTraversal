
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
			TREELET_FALSE = 3,
			TREELET_CYCLE = 4,
			TREELET_LESS = 5,
			TREELET_CYCLEGR = 6
		};

		struct RefineParams
		{
			TreeletHeur treeletHeuristic = TREELET_CYCLE;
			int maxLoops = 10000;
            int nTrLeaves = 9;
            int nTrInternals = nTrLeaves - 1;
			int freezeThreshold = 20;
        };

    public:
        Refine(BVH& bvh, const BVH::BuildParams& params, const RefineParams& rparams);
        ~Refine(void);

        void                run();

		void                collapseLeaves();

    private:
        Refine(const Refine&); // forbidden
        Refine& operator=(const Refine&); // forbidden

        template<class Pr>
        void formTreeletPred(BVHNode* troot, int nTrLeaves, std::vector<BVHNode*>& internals, std::vector<BVHNode*>& leaves, Pr Pred);

		void formTreelet(BVHNode* tRoot, int nTrLeaves, std::vector<BVHNode*>& internals, std::vector<BVHNode*>& leaves);

		BVHNode* formNodes(std::vector<BVHNode*>& internals, std::vector<BVHNode*>& leaves, U32 s);

		bool refineTreelet(BVHNode* troot); // The helper called by refineNode; returns true if it made progress

        bool refineNode(BVHNode* node); // The recursive call; returns true if it made progress

    private:
        BVH&                    m_bvh;
        const Platform&         m_platform;
        const BVH::BuildParams& m_params;
        const RefineParams&     m_rparams;

        std::vector<float>      m_sahes;
        std::vector<float>      m_copt; // Cost of a subtree, not scaled by root area
        std::vector<U32>        m_popt;

		TreeletHeur             m_treeletHeur;
		int                     m_optTreeletsDone;
		int                     m_optTreeletsOutput;

        Timer                   m_progressTimer;

    };

};
