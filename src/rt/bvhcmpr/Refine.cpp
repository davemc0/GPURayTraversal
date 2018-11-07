
// XXX
#define FW_ENABLE_ASSERT

#include "bvhcmpr/Refine.hpp"
#include "bvhcmpr/TRefine.hpp"
#include "bvhcmpr/BRefine.hpp"

#include <algorithm>

#define SHOWFUNC() printf("Optimizing with %s algorithm\n", __func__)

namespace FW
{

    Refine::Refine(BVH& bvh)
        : m_bvh(bvh),
        m_platform(bvh.getPlatform())
    {
    }

    Refine::~Refine(void)
    {
    }

    float Refine::checkTree(bool recomputeBounds, bool resetFrozen)
    {
        m_bvh.getRoot()->computeSubtreeValues(m_platform, m_bvh.getRoot()->getArea(), recomputeBounds, resetFrozen);
        float ta = m_progressTimer.end();
        printf("sah=%.6f tt=%f t=%f\n", m_bvh.getRoot()->m_sah, m_progressTimer.getTotal(), ta);
        return ta;
    }

    void Refine::run()
    {
        runBestAdversarial();
        // runExtremeTRBVH();
        // runBestSplitsLeafCollapse();
        // runExtremeBittner();
        // runTraditionalBittner();
        // runBestNoSplitsPrimPerLeaf();
        // runBestSplitsPrimPerLeaf();
        // runQuickAndClean();
        // runTest();
    }

    void Refine::runBestAdversarial()
    {
        SHOWFUNC();
        TRefine::Params tparams;
        tparams.nTrLeaves = 6;
        tparams.freezeThreshold = 2;
        tparams.maxLoops = 1;
        tparams.treeletEpsilon = 1e-4f;
        tparams.treeletHeuristic = TRefine::TreeletHeur::TREELET_GREATER;
        TRefine TRef(*this, tparams);

        BRefine::Params bparams;
        bparams.maxLoops = 10;
        bparams.batchSize = 0.01f;
        BRefine BRef(*this, bparams);

        float nTrLeaves = (float)tparams.nTrLeaves;
        float sah = m_bvh.getRoot()->m_sah;
        for (int x = 0; x < 10000; x++)
        {
            float ta = 0;

            {
                tparams.nTrLeaves = (int)nTrLeaves;
                TRef.run();

                ta = checkTree(false, false);
                if (sah / m_bvh.getRoot()->m_sah < 1.1f)
                    nTrLeaves += 0.2f;
                sah = m_bvh.getRoot()->m_sah;

                if (m_bvh.getRoot()->m_sah < 200.0f)
                    tparams.treeletHeuristic = TRefine::TreeletHeur::TREELET_CYCLE;
            }

            if (m_bvh.getRoot()->m_sah < 1000.0f) {
                bparams.timeBudget = max(10.f, ta);

                BRef.run();
                bparams.batchSize *= 1.2f;
                bparams.priorityMetric = static_cast<BRefine::PriorityHeur>((bparams.priorityMetric + 1) % BRefine::PriorityHeur::PRIORITY_CYCLE);

                checkTree(false, false);
            }

            if (x % 10 == 9) {
                collapseLeaves();
            }

            if (m_progressTimer.getTotal() > 3600.0f)
                break;
        }
    }

    void Refine::runBestOrderedRandom()
    {
        SHOWFUNC();
    }

    void Refine::runBestNoSplitsPrimPerLeaf()
    {
        SHOWFUNC();
        TRefine::Params tparams;
        tparams.nTrLeaves = 7;
        tparams.freezeThreshold = 5;
        tparams.maxLoops = 1;
        tparams.treeletEpsilon = 1e-4f;
        tparams.treeletHeuristic = TRefine::TreeletHeur::TREELET_GREATER;
        TRefine TRef(*this, tparams);

        BRefine::Params bparams;
        bparams.maxLoops = 18;
        bparams.batchSize = 0.02f;
        bparams.timeBudget = 10.0f;
        BRefine BRef(*this, bparams);

        for (int x = 0; x < 10000; x++) {
            TRef.run();
            TRef.run();
            checkTree(false, false);
            bparams.maxLoops = 5;

            BRef.run();
            checkTree(false, false);
            TRef.run();
            TRef.run();
            checkTree(true, true);
            bparams.maxLoops = 4;

            BRef.run();
            checkTree(true, true);
            TRef.run();
            TRef.run();

            if (m_progressTimer.getTotal() > 30.0f)
                break;
        }

        collapseLeaves();
    }

    void Refine::runBestNoSplitsLeafCollapse()
    {
        SHOWFUNC();
    }

    void Refine::runBestSplitsPrimPerLeaf()
    {
        SHOWFUNC();
        float timeBudget = max(7.f, getTimer().getElapsed() * 2.f); // Spend as much on refinement as on build

        TRefine::Params tparams;
        tparams.nTrLeaves = 10;
        tparams.freezeThreshold = 3;
        tparams.maxLoops = 10;
        tparams.treeletEpsilon = 1e-4f;
        tparams.treeletHeuristic = TRefine::TreeletHeur::TREELET_GREATER;
        TRefine TRef(*this, tparams);

        BRefine::Params bparams;
        bparams.maxLoops = 18;
        bparams.batchSize = 0.02f;
        bparams.timeBudget = 10.0f;
        bparams.nodesToRemove = 2;
        BRefine BRef(*this, bparams);

        for (int x = 0; x < 10000; x++) {
            TRef.run();
            checkTree(false, true);

            bparams.maxLoops = 1;
            bparams.removeRandomChild = false;
            BRef.run();

            TRef.run();
            checkTree(false, true);

            bparams.maxLoops = 1;
            bparams.removeRandomChild = true;
            BRef.run();

            TRef.run();
            checkTree(false, true);

            if (m_progressTimer.getTotal() > timeBudget)
                break;
        }

        collapseLeaves();
    }

    void Refine::runBestSplitsLeafCollapse()
    {
        SHOWFUNC();
        // Ignore the function name.
        // This is all Bittner, followed by one TRBVH to show that Bittner hits a local minimum.
        BRefine::Params bparams;
        bparams.maxLoops = 3;
        bparams.batchSize = 0.02f;
        bparams.timeBudget = 0.0f;
        bparams.nodesToRemove = 2;
        bparams.priorityMetric = BRefine::PriorityHeur::PRIORITY_MIN_AND_SUM;
        BRefine BRef(*this, bparams);

        for (int x = 0; x < 10000; x++) {
            BRef.run();
            bparams.batchSize *= 1.1f;
            if (bparams.batchSize > 0.1f) bparams.batchSize = 0.1f;

            // checkTree(false, false);

            //if (bparams.priorityMetric == BRefine::PriorityHeur::PRIORITY_CYCLE)
            bparams.priorityMetric = static_cast<BRefine::PriorityHeur>((bparams.priorityMetric + 1) % BRefine::PriorityHeur::PRIORITY_CYCLE);

            if (m_progressTimer.getTotal() > 300.0f)
                break;
        }

        TRefine::Params tparams;
        tparams.nTrLeaves = 7;
        tparams.freezeThreshold = 3;
        tparams.maxLoops = 1;
        tparams.treeletEpsilon = 1e-4f;
        tparams.treeletHeuristic = TRefine::TreeletHeur::TREELET_GREATER;
        TRefine TRef(*this, tparams);

        TRef.run();
        checkTree(false, false);
        collapseLeaves();
    }

    void Refine::runExtremeBittner()
    {
        SHOWFUNC();
        BRefine::Params bparams;
        bparams.maxLoops = 5;
        bparams.batchSize = 0.001f;
        bparams.timeBudget = 0.0f;
        bparams.nodesToRemove = 2;
        bparams.priorityMetric = BRefine::PriorityHeur::PRIORITY_ALL_THREE;
        BRefine BRef(*this, bparams);

        for (int x = 0; x < 10000; x++) {
            BRef.run();
            bparams.batchSize *= 1.1f;
            if (bparams.batchSize > 0.1f) bparams.batchSize = 0.1f;

            checkTree(false, false);

            //if (bparams.priorityMetric == BRefine::PriorityHeur::PRIORITY_CYCLE)
            bparams.priorityMetric = static_cast<BRefine::PriorityHeur>((bparams.priorityMetric + 1) % BRefine::PriorityHeur::PRIORITY_CYCLE);

            if (m_progressTimer.getTotal() > 600.0f)
                break;
        }

        collapseLeaves();
    }

    void Refine::runTraditionalBittner()
    {
        SHOWFUNC();
        BRefine::Params bparams;
        bparams.maxLoops = 100000;
        bparams.batchSize = 0.01f;
        bparams.timeBudget = 100.0f;
        BRefine BRef(*this, bparams);

        for (int x = 0; x < 10000; x++) {
            BRef.run();

            if (m_progressTimer.getTotal() > bparams.timeBudget)
                break;
        }

        collapseLeaves();
    }

    void Refine::runExtremeTRBVH()
    {
        SHOWFUNC();
        TRefine::Params tparams;
        tparams.nTrLeaves = 7;
        tparams.freezeThreshold = 7;
        tparams.maxLoops = 100;
        tparams.treeletEpsilon = 1e-4f;
        tparams.treeletHeuristic = TRefine::TreeletHeur::TREELET_CYCLE;
        TRefine TRef(*this, tparams);

        TRef.run();

        collapseLeaves();
    }

    void Refine::runGrowingTRBVH()
    {
        SHOWFUNC();
    }

    void Refine::runQuickAndClean()
    {
        SHOWFUNC();
        TRefine::Params tparams;
        tparams.nTrLeaves = 7;
        tparams.freezeThreshold = 4;
        tparams.maxLoops = 1;
        tparams.treeletEpsilon = 1e-5f;
        tparams.treeletHeuristic = TRefine::TreeletHeur::TREELET_GREATER;
        TRefine TRef(*this, tparams);

        BRefine::Params bparams;
        bparams.maxLoops = 3;
        bparams.batchSize = 0.02f;
        bparams.timeBudget = 0.0f;
        bparams.nodesToRemove = 2;
        bparams.removeRandomChild = false;
        BRefine BRef(*this, bparams);

        TRef.run();

        checkTree(false, true);

        BRef.run();

        TRef.run();

        checkTree(false, true);

        BRef.run();

        TRef.run();

        collapseLeaves();
    }

    void Refine::runTest()
    {
        SHOWFUNC();
    }

    // Returns treelet root node so it can be attached to parent
    BVHNode* Refine::collapseLeavesRecursive(BVHNode* node, Array<S32>& newTriIndices)
    {
        if (node->isLeaf()) {
            Array<S32>& oldTriIndices = m_bvh.getTriIndices();
            LeafNode* oldL = dynamic_cast<LeafNode*>(node);

            // Copy the indices
            int lo = newTriIndices.getSize();
            FW_ASSERT(oldL->m_hi - oldL->m_lo == node->m_tris);
            for (int i = oldL->m_lo; i < oldL->m_hi; i++) {
                newTriIndices.add(oldTriIndices[i]);
            }

            // Update the leaf node
            oldL->m_lo = lo;
            oldL->m_hi = newTriIndices.getSize();
        }
        else {
            InnerNode* oldI = dynamic_cast<InnerNode*>(node);

            // Find treelet to collapse
            float SAHAsLeaf = m_platform.getCost(0, node->m_tris) * node->m_probability;

            S32 lo = newTriIndices.getSize();

            // Recompute SAH of all nodes as it goes up
            node->m_sah = m_platform.getCost(node->getNumChildNodes(), 0) * node->m_probability;

            for (int i = 0; i < node->getNumChildNodes(); i++) {
                oldI->m_children[i] = collapseLeavesRecursive(node->getChildNode(i), newTriIndices);
                node->m_sah += oldI->m_children[i]->m_sah;
            }

            if (SAHAsLeaf < node->m_sah && node->m_tris <= m_platform.getMaxLeafSize()) {
                // Not using the m_treelet flag, which indicates a node that the algorithm indicated should be collapsed.
                // The algorithm doesn't take MaxLeafSize into account, etc. And SAH is better this way.

                // Remove duplicate triangle references
                std::sort(&newTriIndices[0] + lo, &newTriIndices[0] + newTriIndices.getSize());

                for (S32 t = lo; t < newTriIndices.getSize() - 1; t++) {
                    if (newTriIndices[t] == newTriIndices[t + 1]) {
                        newTriIndices.remove(t);
                        t--;
                    }
                }

                // Make the new leaf
                BVHNode* newL = new LeafNode(node->m_bounds, lo, newTriIndices.getSize());
                newL->m_tris = newTriIndices.getSize() - lo;
                newL->m_probability = node->m_probability;
                newL->m_sah = SAHAsLeaf;

                node->deleteSubtree();

                return newL;
            }
        }

        return node;
    }

    void Refine::collapseLeaves()
    {
        printf("Collapse ");
        float oldSAH = m_bvh.getRoot()->m_sah;

        Array<S32>& oldTriIndices = m_bvh.getTriIndices();
        Array<S32> newTriIndices;

        // Depth-first traversal to insert triIndices into new triIndices array
        BVHNode* newRoot = collapseLeavesRecursive(m_bvh.getRoot(), newTriIndices);

        m_bvh.setRoot(newRoot);

        float sah = m_bvh.getRoot()->m_sah;
        float impr = (oldSAH - sah) / sah;

        printf("dsah=%1.2f%% sah=%.6f tt=%f t=%f\n", 100.f * impr, sah, m_progressTimer.getTotal(), m_progressTimer.end());

        // Move these in and delete the old ones
        oldTriIndices = newTriIndices;
    }
};
