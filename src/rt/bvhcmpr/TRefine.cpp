
// XXX
#define FW_ENABLE_ASSERT

#include "bvhcmpr/Refine.hpp"
#include "bvhcmpr/TRefine.hpp"

#include <algorithm>

#include <intrin.h>

namespace FW
{

    TRefine::TRefine(Refine& ref, Params& rparams)
        : m_bvh(ref.getBVH()),
        m_refine(ref),
        m_platform(ref.getBVH().getPlatform()),
        m_rparams(rparams),
        m_treeletHeur(TreeletHeur::TREELET_CYCLE)
    {
    }

    TRefine::~TRefine(void)
    {
    }

    void TRefine::run()
    {
        // Allocate temporary stuff
        m_sahes.resize(1ull << m_rparams.nTrLeaves);
        m_copt.resize(1ull << m_rparams.nTrLeaves);
        m_popt.resize(1ull << m_rparams.nTrLeaves);

        m_collapseToLeaves = false;
        m_freezeThreshold = m_rparams.freezeThreshold;
        bool lastIter = false;
        int noProgressPasses = 0;
        BVHNode* root = m_bvh.getRoot();
        float oldSAH = root->m_sah;

        for (int i = 0; i < m_rparams.maxLoops; i++) {
            m_stats = Statistics();
            if (m_rparams.treeletHeuristic == TreeletHeur::TREELET_CYCLE)
                m_treeletHeur = static_cast<TreeletHeur>((m_treeletHeur + 1) % TreeletHeur::TREELET_CYCLE);
            else
                m_treeletHeur = m_rparams.treeletHeuristic;

            printf("TRBVH %d size=%d heur=%d freeze=%d ", i, m_rparams.nTrLeaves, m_treeletHeur, m_freezeThreshold);

            refineNode(root);

            float sah = root->m_sah;
            float impr = (oldSAH - sah) / sah;
            oldSAH = sah;

            m_stats.print();
            printf("dsah=%1.2f%% csah=%.6f tt=%f t=%f\n", 100.f * impr, sah, m_refine.getTimer().getTotal(), m_refine.getTimer().end());

            // Terminate
            if (lastIter)
                break;
            if (m_stats.optSuccess > 0)
                noProgressPasses = 0;
            else
                noProgressPasses++;

            if (noProgressPasses > m_rparams.maxNoProgressPasses || impr < m_rparams.perPassImprovement) {
                if (m_rparams.leafCollapsePass) {
                    m_freezeThreshold = 10000000;
                    m_collapseToLeaves = true;
                    lastIter = true;
                }
                else
                    break;
            }

            m_rparams.treeletEpsilon *= m_rparams.epsilonScale;
        }
    }

    // Form a treelet rooted at tRoot based on predicate Pred
    // When finished, internals contains all internal nodes, internals[0] is tRoot, and leaves contains all the leaves.
    template<class Pr>
    void TRefine::formTreeletPred(BVHNode* tRoot, int nTrLeaves, std::vector<BVHNode*>& internals, std::vector<BVHNode*>& leaves, Pr Pred)
    {
        FW_ASSERT(internals.size() == 0 && leaves.size() == 0);

        leaves.push_back(tRoot);

        while (leaves.size() < nTrLeaves) {
            // Add the children of the leaf that optimizes Pred

            BVHNode* node = leaves.back();
            leaves.pop_back();
            internals.push_back(node);

            if (node->isLeaf()) {
                //printf("Can't form treelet with %d leaves: nI=%d nL=%d\n", nTrLeaves, internals.size(), leaves.size());
                return;
            }

            for (int i = 0; i < node->getNumChildNodes(); i++) {
                BVHNode* ch = node->getChildNode(i);

                // Insert the child node after the first node for which the predicate is true
                auto it = std::find_if(leaves.begin(), leaves.end(), [ch, Pred](const BVHNode* a) { const BVHNode* b = ch; return Pred(a, b); });
                leaves.insert(it, ch);
            }
        }

        FW_ASSERT(nTrLeaves == leaves.size());
        FW_ASSERT(nTrLeaves - 1 == internals.size());
        FW_ASSERT(internals[0] == tRoot);
    }

    void TRefine::formTreelet(BVHNode* tRoot, int nTrLeaves, std::vector<BVHNode*>& internals, std::vector<BVHNode*>& leaves)
    {
        switch (m_treeletHeur) {
        case TreeletHeur::TREELET_GREATER:
            formTreeletPred(tRoot, nTrLeaves, internals, leaves,
                [](const void* aa, const void* bb) { BVHNode* a = (BVHNode*)aa; BVHNode* b = (BVHNode*)bb; return (a->isLeaf() && !b->isLeaf()) ? false : (b->isLeaf() && !a->isLeaf()) ? true : a->getArea() > b->getArea(); }); // This worked.
            break;
        case TreeletHeur::TREELET_LESS:
            formTreeletPred(tRoot, nTrLeaves, internals, leaves,
                [](const void* aa, const void* bb) { BVHNode* a = (BVHNode*)aa; BVHNode* b = (BVHNode*)bb; return (a->isLeaf() && !b->isLeaf()) ? false : (b->isLeaf() && !a->isLeaf()) ? true : a->getArea() < b->getArea(); }); // a is the one that's already there and b is the one it's inserting
            break;
        case TreeletHeur::TREELET_RANDOM:
            formTreeletPred(tRoot, nTrLeaves, internals, leaves,
                [](const void* aa, const void* bb) { BVHNode* a = (BVHNode*)aa; BVHNode* b = (BVHNode*)bb; return (a->isLeaf() && !b->isLeaf()) ? false : (b->isLeaf() && !a->isLeaf()) ? true : !(rand() % 3); }); // 1/3 true
            break;
        case TreeletHeur::TREELET_TRUE:
            formTreeletPred(tRoot, nTrLeaves, internals, leaves,
                [](const void* aa, const void* bb) { BVHNode* a = (BVHNode*)aa; BVHNode* b = (BVHNode*)bb; return (a->isLeaf() && !b->isLeaf()) ? false : (b->isLeaf() && !a->isLeaf()) ? true : true; });
            break;
        case TreeletHeur::TREELET_FALSE:
            formTreeletPred(tRoot, nTrLeaves, internals, leaves,
                [](const void* aa, const void* bb) { BVHNode* a = (BVHNode*)aa; BVHNode* b = (BVHNode*)bb; return (a->isLeaf() && !b->isLeaf()) ? false : (b->isLeaf() && !a->isLeaf()) ? true : false; });
            break;
        }
    }

    // Refine some treelet rooted at tRoot
    // Kensler 2008
    bool TRefine::refineTreeletKensler(BVHNode* tRoot)
    {
        std::vector<BVHNode*> internals;
        std::vector<BVHNode*> leaves;

        formTreelet(tRoot, m_rparams.nTrLeaves, internals, leaves);

        if (leaves.size() < m_rparams.nTrLeaves)
            return false;

        // Refine treelet
        // printf("Refining treelet with %d leaves: nI=%d nL=%d\n", m_rparams.nTrLeaves, internals.size(), leaves.size());
        // for (auto i : leaves)
        // printf("%c=>%f ", i->isLeaf() ? 'L' : 'I', i->getArea());
        // printf("\n");

        FW_ASSERT(m_rparams.nTrLeaves == 3);

        InnerNode* rt = dynamic_cast<InnerNode*>(internals[0]);
        InnerNode* in = dynamic_cast<InnerNode*>(internals[1]);

        float rootArea = m_bvh.getRoot()->getArea();
        float Ci = m_platform.getCost(rt->getNumChildNodes(), rt->getNumTriangles());

        AABB box12(leaves[1]->m_bounds); box12.grow(leaves[2]->m_bounds);
        float out0 = box12.area();
        AABB box02(leaves[0]->m_bounds); box02.grow(leaves[2]->m_bounds);
        float out1 = box02.area();
        AABB box01(leaves[0]->m_bounds); box01.grow(leaves[1]->m_bounds);
        float out2 = box01.area();

        rt->m_children[0] = internals[1];

        float out = min(out0, out1, out2);

        if (out == out0) {
            in->m_children[0] = leaves[1];
            in->m_children[1] = leaves[2];
            in->m_bounds = box12;
            rt->m_children[1] = leaves[0];
        }
        else if (out == out1) {
            in->m_children[0] = leaves[0];
            in->m_children[1] = leaves[2];
            in->m_bounds = box02;
            rt->m_children[1] = leaves[1];
        }
        else {
            in->m_children[0] = leaves[0];
            in->m_children[1] = leaves[1];
            in->m_bounds = box01;
            rt->m_children[1] = leaves[2];
        }

        in->m_sah = Ci * in->getArea() / rootArea + in->m_children[0]->m_sah + in->m_children[1]->m_sah;
        rt->m_sah = Ci * rt->getArea() / rootArea + rt->m_children[0]->m_sah + rt->m_children[1]->m_sah;

        return true;
    }

    // Return the index of the first set (one) bit
    inline U32 ffs(U32 x)
    {
        unsigned long ind;
        _BitScanForward(&ind, x);
        return ind;
    }

    BVHNode* TRefine::formNodes(std::vector<BVHNode*>& internals, std::vector<BVHNode*>& leaves, U32 s)
    {
        float rootArea = m_bvh.getRoot()->getArea();

        if (__popcnt(s) == 1)
            return leaves[ffs(s)];

        U32 p = m_popt[s] & ~Refine::LEAF_FLAG;

        BVHNode* l = formNodes(internals, leaves, p);
        BVHNode* r = formNodes(internals, leaves, s ^ p);

        InnerNode* in = dynamic_cast<InnerNode*>(internals.back());
        internals.pop_back();

        l->m_parent = r->m_parent = in;
        in->m_children[0] = l;
        in->m_children[1] = r;
        in->m_bounds = l->m_bounds;
        in->m_bounds.grow(r->m_bounds);
        in->m_probability = in->getArea() / rootArea;
        in->m_sah = m_copt[s];
        in->m_tris = l->m_tris + r->m_tris;
        in->m_frozen = 0;
        in->m_treelet = (m_popt[s] & Refine::LEAF_FLAG) && m_collapseToLeaves;

        return in;
    }

    // True if b is sufficiently less than a
    inline bool fuzzyDiff(const float a, const float b, const float eps)
    {
        float d = a - b;
        float s = d / a;
        return s > eps;
    }

    // Refine some treelet rooted at tRoot
    bool TRefine::refineTreelet(BVHNode* tRoot)
    {
        const int nL = m_rparams.nTrLeaves;

        std::vector<BVHNode*> internals;
        std::vector<BVHNode*> leaves;

        formTreelet(tRoot, nL, internals, leaves);

        if (leaves.size() < nL) {
            tRoot->m_frozen++;
            m_stats.optFailNoTreelet++;
            return false;
        }

        FW_ASSERT(tRoot == internals[0]);

        // Calculate surface areas of all subsets
        float rootArea = m_bvh.getRoot()->getArea();
        float invRootArea = 1.f / rootArea;
        for (U32 s = 0; s < (1ul << nL); s++) {
            AABB as;
            for (int i = 0; i < nL; i++) {
                if ((1 << i) & s)
                    as.grow(leaves[i]->m_bounds);
            }
            m_sahes[s] = as.area() * invRootArea;
        }

        // Initialize costs of leaves
        for (int i = 0; i < nL; i++)
            m_copt[1ull << i] = leaves[i]->m_sah;

        // Optimize each subset
        for (U32 k = 2; k <= (U32)nL; k++) { // Number of leaves in the subset
            for (U32 s = 0; s < (1ul << nL); s++) {
                if (__popcnt(s) == k) {
                    float bestC = FW_F32_MAX;
                    U32 bestP = 0;
                    S32 d = (s - 1) & s;
                    S32 p = (-d) & s;

                    // Find best partition of s
                    do {
                        float c = m_copt[p] + m_copt[s ^ p];
                        if (c < bestC) {
                            bestC = c;
                            bestP = p;
                        }
                        p = (p - d) & s; // Find the next valid partitioning with k bits set
                    } while (p > 0);

                    // Calculate final SAH cost for s
                    int sTris = 0;
                    for (int i = 0; i < nL; i++)
                        if ((1ul << i) & s)
                            sTris += leaves[i]->m_tris;
                    float SAHAsLeaf = m_platform.getCost(0, sTris) * m_sahes[s];
                    float SAHAsTreelet = m_platform.getCost(2, 0) * m_sahes[s] + bestC; // My cost plus the cost of my two children
                    m_copt[s] = min(SAHAsLeaf, SAHAsTreelet);
                    if (SAHAsLeaf < SAHAsTreelet) {
                        // Don't collapse the leaf now; just treat the SAH as though we were collapsing it.
                        // Setting bestP to 0 should be fine, but we don't want to flatten leaves until after, so we preserve the partitioning.
                        bestP = bestP | Refine::LEAF_FLAG; // Force leaf
                        // printf("forceLeaf 0x%x 0x%x %f < %f\n", s, p, SAHAsLeaf, SAHAsTreelet);
                    }
                    m_popt[s] = bestP;
                }
            }
        }

        m_stats.optAttempts++;

        // Construct treelet
        U32 rti = (1ul << nL) - 1;
        if (m_collapseToLeaves || fuzzyDiff(tRoot->m_sah, m_copt[rti], m_rparams.treeletEpsilon)) {
            //if (m_rparams.treeletEpsilon == 1e-2f) {
            //	float diff = tRoot->m_sah - m_copt[rti];
            //	printf("Treelet: 0x%08x %.8g - %.8g = %.8g %.8g %f\n", tRoot, tRoot->m_sah, m_copt[rti], diff, m_rparams.treeletEpsilon, diff / tRoot->m_sah);
            //	//m_bvh.printTree(tRoot);
            //}

            m_stats.optSuccess++;

            // Because tRoot == internals[0] it properly attaches the subtree root
            formNodes(internals, leaves, rti);

            FW_ASSERT(internals.size() == 0);

            return true;
        }

        tRoot->m_frozen++;

        m_stats.optFailNoImprovement++;

        return false;
    }

    bool TRefine::refineNode(BVHNode* node)
    {
        m_stats.optVisits++;

        if (node->isLeaf()) {
            m_stats.optFailIsLeaf++;
            return false;
        }

        // Abort if this node isn't big enough to form the treelet
        if (node->m_tris < m_rparams.nTrLeaves) {
            m_stats.optFailTooSmall++;
            return false;
        }

        // Abort if this subtree has not improved in too long
        if (node->m_frozen > m_freezeThreshold) {
            m_stats.optFailFrozen++;
            return false;
        }

        bool childSucc = false;
        for (int i = 0; i < node->getNumChildNodes(); i++)
            childSucc |= refineNode(node->getChildNode(i));

        bool succ = refineTreelet(node);

        if (childSucc && !succ) {
            // Compute my SAH if refineTreelet was unsuccessful (to keep it up to date in case children updated)
            node->m_sah = m_platform.getCost(node->getNumChildNodes(), 0) * node->getArea() / m_bvh.getRoot()->getArea();
            for (int i = 0; i < node->getNumChildNodes(); i++)
                node->m_sah += node->getChildNode(i)->m_sah;
            node->m_frozen = 0;
        }

        return succ || childSucc;
    }

};
