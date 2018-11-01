// XXX
#define FW_ENABLE_ASSERT

#include "bvhcmpr/RefineBittner.hpp"

#include <algorithm>

#include <intrin.h>

namespace FW
{

    BRefine::BRefine(BVH& bvh, const BVH::BuildParams& params, BRefineParams& rparams)
        : m_bvh(bvh),
        m_platform(bvh.getPlatform()),
        m_params(params),
        m_rparams(rparams)
    {
    }

    BRefine::~BRefine(void)
    {
    }

    void BRefine::computePriority(BVHNode* node)
    {
        if (node->isLeaf())
            return;

        float a = node->getArea();
        float sumChArea = 0, minChArea = FW_F32_MAX;

        for (int i = 0; i < node->getNumChildNodes(); i++) {
            computePriority(node->getChildNode(i));
            float na = node->getChildNode(i)->getArea();
            sumChArea += na;
            minChArea = min(minChArea, na);
        }

        //float metric = a;
        //float metric = a / sumChArea;
        //float metric = a / minChArea;
        float metric = a * a * a / (sumChArea * minChArea);

        OptPair_T pr(metric, node);
        m_toOptimize.push_back(pr);
    }

#if 1
    // Original Bittner algorithm
    // Remove a high cost node (because its children don't belong together)
    void BRefine::removeForOpt(BVHNode* node)
    {
        FW_ASSERT(!node->isLeaf());

        if (node->m_parent == nullptr || node->m_parent->m_parent == nullptr)
            return;

        FW_ASSERT(node->m_parent);
        FW_ASSERT(node->m_parent->m_parent);

        // Put this node's children into m_toInsert
        for (int i = 0; i < node->getNumChildNodes(); i++) {
            BVHNode* ch = node->getChildNode(i);
            ch->m_parent = nullptr;
            OptPair_T pr(ch->getArea(), ch);
            m_toInsert.push_back(pr);
        }

        // Fix up the tree with this node gone
        BVHNode* dad = node->m_parent;
        BVHNode* gpa = dad->m_parent;
        FW_ASSERT(gpa);
        InnerNode* gpai = dynamic_cast<InnerNode*>(gpa);

        int dadInd = 0;
        for (dadInd = 0; dadInd < gpa->getNumChildNodes(); dadInd++)
            if (gpa->getChildNode(dadInd) == dad)
                break;

        for (int i = 0; i < dad->getNumChildNodes(); i++) {
            BVHNode* sib = dad->getChildNode(i);
            if (sib != node) {
                gpai->m_children[dadInd] = sib;
                sib->m_parent = gpa;
            }
        }

        // Tag the nodes as unused and toss them.
        // Don't delete them because they may be in the toOptimize list and will get referenced later.
        node->m_parent = nullptr;
        dad->m_parent = nullptr;
        m_toRecycle.push_back(OptPair_T(0.f, node));
        m_toRecycle.push_back(OptPair_T(0.f, dad));

        // Recompute its ancestors
        BVHNode* cur = gpa;
        while (cur) {
            //float oldArea = cur->getArea();
            cur->computeValues(m_platform, m_rootArea, true, true);
            //float newArea = cur->getArea();
            cur = cur->m_parent;
            //printf("%c", (newArea < oldArea) ? '<' : (newArea > oldArea) ? '>' : '=');
        }
        //printf("\n");
    }

#else
    // My algorithm
    // Find a treelet that's in the wrong place
    void BRefine::removeForOpt(BVHNode* node)
    {
        FW_ASSERT(!node->isLeaf());

        if (node->m_parent == nullptr || node->m_parent->m_parent == nullptr)
            return;

        FW_ASSERT(node->m_parent);
        FW_ASSERT(node->m_parent->m_parent);

        // Try pairing each child with uncle

        BVHNode* dad = node->m_parent;
        BVHNode* gpa = dad->m_parent;
        FW_ASSERT(gpa);
        InnerNode* gpai = dynamic_cast<InnerNode*>(gpa);

        int dadInd = 0;
        for (dadInd = 0; dadInd < gpa->getNumChildNodes(); dadInd++)
            if (gpa->getChildNode(dadInd) == dad)
                break;

        for (int i = 0; i < dad->getNumChildNodes(); i++) {
            BVHNode* sib = dad->getChildNode(i);
            if (sib != node) {
                gpai->m_children[dadInd] = sib;
                sib->m_parent = gpa;
            }
        }

        // Fix up the tree with this node gone

        // Put this node's children into m_toInsert
        for (int i = 0; i < node->getNumChildNodes(); i++) {
            BVHNode* ch = node->getChildNode(i);
            ch->m_parent = nullptr;
            OptPair_T pr(ch->getArea(), ch);
            m_toInsert.push_back(pr);
        }

        // Tag the nodes as unused and toss them.
        // Don't delete them because they may be in the toOptimize list and will get referenced later.
        node->m_parent = nullptr;
        dad->m_parent = nullptr;
        m_toRecycle.push_back(OptPair_T(0.f, node));
        m_toRecycle.push_back(OptPair_T(0.f, dad));

        // Recompute its ancestors
        BVHNode* cur = gpa;
        while (cur) {
            //float oldArea = cur->getArea();
            cur->computeValues(m_platform, m_rootArea, true, true);
            //float newArea = cur->getArea();
            cur = cur->m_parent;
            //printf("%c", (newArea < oldArea) ? '<' : (newArea > oldArea) ? '>' : '=');
        }
        //printf("\n");
    }
#endif

    BVHNode* BRefine::findInsertTarget(BVHNode* node)
    {
        FW_ASSERT(m_toSearch.size() == 0);
        float nArea = node->getArea();
        OptPair_T pr(0.f, m_bvh.getRoot());
        m_toSearch.push_back(pr);

        float cBest = FW_F32_MAX; // Want to set this to the induced cost saved by removing it.
        BVHNode* xBest = nullptr;

        while (m_toSearch.size() > 0) {
            BVHNode* x = m_toSearch.back().second;
            float cIx = m_toSearch.back().first; // Induced cost of placing node as sibling of x
            m_toSearch.pop_back();
            FW_ASSERT(x);
            if (cIx + nArea >= cBest) // Our best branch isn't good enough => abort
                break;

            // Compute total cost of merging node with x
            float cD = (x->m_bounds + node->m_bounds).area();
            float cT = cIx + cD;
            if (cT < cBest) {
                cBest = cT;
                xBest = x;
            }

            // Calculate the induced cost for children of x
            float xArea = x->m_bounds.area();
            float cI = cT - xArea;

            // Are children of x candidates?
            if (cI + nArea < cBest && !x->isLeaf()) {
                // Can't hoist because iterator becomes invalid after insert
                auto insBefore = std::find_if(m_toSearch.begin(), m_toSearch.end(), [cI](const OptPair_T& a) { return cI < a.first; });
                size_t iB = insBefore - m_toSearch.begin();
                for (int i = 0; i < x->getNumChildNodes(); i++) {
                    OptPair_T pc(cI, x->getChildNode(i));
                    m_toSearch.insert(m_toSearch.begin() + iB, pc);
                }
            }
        }

        FW_ASSERT(xBest);
        // FW_ASSERT(xBest != m_bvh.getRoot());

        m_toSearch.resize(0);

        return xBest;
    }

    void BRefine::insertForOpt(BVHNode* n)
    {
        FW_ASSERT(n);

        BVHNode* x = findInsertTarget(n); // Insert n as a sibling of x

        InnerNode* in = dynamic_cast<InnerNode*>(m_toRecycle.back().second);
        m_toRecycle.pop_back();

        InnerNode* gpa = dynamic_cast<InnerNode*>(x->m_parent);
        if (gpa) {
            for (int i = 0; i < gpa->getNumChildNodes(); i++)
                if (gpa->getChildNode(i) == x)
                    gpa->m_children[i] = in;
        }
        else {
            // Parent of x is root
            //float oldRootArea = m_bvh.getRoot()->getArea();
            m_bvh.setRoot(in);
            // printf("Replacing root. Old area=%f New area=%f Actual area=%f\n", oldRootArea, m_bvh.getRoot()->getArea(), m_rootArea);
        }
        in->m_parent = gpa;

        x->m_parent = n->m_parent = in;
        in->m_children[0] = x;
        in->m_children[1] = n;

        // Recompute its ancestors
        BVHNode* cur = in;
        while (cur) {
            //float oldArea = cur->getArea();
            cur->computeValues(m_platform, m_rootArea, true, true);
            //float newArea = cur->getArea();
            cur = cur->m_parent;
            //printf("%c", (newArea < oldArea) ? '<' : (newArea > oldArea) ? '>' : '=');
        }
        //printf("\n");
        m_nodesInserted++;
    }

    // Declare this with non-templated args so that std::sort can find it instead of getting ambiguous result
    void swap(OptPair_T& lhs, OptPair_T& rhs)
    {
        std::swap(lhs, rhs);
    }

    void BRefine::run()
    {
        m_progressTimer.start();
        bool timeOut = false;

        m_rootArea = m_bvh.getRoot()->getArea();
        for (int iter = 0; iter < m_rparams.maxLoops; iter++) {
            m_nodesInserted = 0;

            // Compute node priorities
            m_toOptimize.resize(0);
            computePriority(m_bvh.getRoot());

            // Sort nodes by priority
            std::sort(m_toOptimize.begin(), m_toOptimize.end(), [](OptPair_T& a, OptPair_T& b) { return a.first > b.first; });

            size_t batchSize = min(size_t(m_rparams.batchSize * (float)m_toOptimize.size()) + 1, m_toOptimize.size()/2);
            for (int i = 0; i < batchSize; i++) {
                // Remove a node
                BVHNode* node = m_toOptimize[i].second;
                //printf("Removing node 0x%08x weight=%f area=%f\n", (U64)node, m_toOptimize[i].first, node->getArea());
                removeForOpt(node);

                // Insert a node
                std::sort(m_toInsert.begin(), m_toInsert.end(), [](OptPair_T& a, OptPair_T& b) { return a.first > b.first; });
                for (auto nn : m_toInsert) {
                    // printf("m_toOptimize=%d m_toInsert=%d m_toRecycle=%d m_toSearch=%d\n", m_toOptimize.size(), m_toInsert.size(), m_toRecycle.size(), m_toSearch.size());
                    //printf("Inserting node 0x%08x weight=%f area=%f\n", (U64)nn.second, nn.first, nn.second->getArea());
                    insertForOpt(nn.second);
                }
                m_toInsert.resize(0);

                if ((i % 32) == 0 && m_rparams.timeBudget > 0 && m_progressTimer.getElapsed() > m_rparams.timeBudget) {
                    timeOut = true;
                    break;
                }
            }
            
            printf("Bitnr %d new sah=%.6f insert=%d batch=%d worst100=%f worst90=%f\n", iter, m_bvh.getRoot()->m_sah, m_nodesInserted, batchSize, m_toOptimize[0].first, m_toOptimize[size_t(m_toOptimize.size()*0.1f)].first);
            if (timeOut)
                break;
        }
    }

};

// Compute refinement heuristic for all nodes
// Take highest batch
// Remove a node and its children
// Find optimal insertion point
// Insert node


// Parallelize by:
// For all nodes in batch:
// Find optimal insertion location and store it
// Serialize insertions

// Do odd-even depths to avoid conflicts
// For every node in whole tree find its best location

// When finding optimal location of every node without deleting it:
// * Compute the incremental cost of removing it from where it is; the optimal location must be better than this.
//   * Track the highest ancestor that has incremental cost because of this node
//   * Don't traverse into the subtree rooted at that highest ancestor.
//   * Obviously can't get a win inserting the node into a child of itself.
// * Can there be an optimal insertion point inside the highest affected ancestor's subtree?

// To show the 7-leaf treelets get stuck in a local minimum, run 7 to completion, then run 8 and look at the biggest transformation that it does.
// Repeatedly choosing the same treelet also contributes to staying in a local minimum.
// Experiment with making a totally random BVH, then refining it
// Study Morton, Hilbert, tiled, raster, buestro, etc. 
// Compute Morton code of upper-left corners and of lower-right corners. The difference tells how big of a treelet they straddle.
// What else can we do with this?

// All refinement algorithms, regardless of parameters, get stuck in local minima. Evidence:
// Starting with a bad BVH never yields the best BVH
// Larger treelet sizes give better results.
// More treelet passes give better results.
// Bittner can beat treelet and treelet can beat Bittner.
// Once TRBVH bottoms out, using a random treelet formation gives improvement

// Bittner problematic node is a treelet in the wrong place, rather than two nodes that don't belong together

// On San Miguel, starting from non-randomized RandomBVH, no leaf collapsing, the actual SAH values: (was 1390 before opt) nTr=7 1% batchSize
// BBBB 691 387 235 149
// BTBT 691 271 112 80
// TBTB 440 196 129 53
// TTTT 432 273 225 162

// Does this pattern hold for HQ BVHes? Does it hold for other models?

// Optimization goes MUCH faster after the leaf collapse.
// Use the TRBVH optimization pass to compute the priority metric for Bittner

// TRBVH with random treelet shape is worse than good treelets on really poor BVHes (like totally random)
// With totally random on San Miguel, Bittner doesn't help AT ALL in the early stages.
// Read Warren's paper on scene hierarchy BVH

// With MSBVH if splitAlpha is infinity it does NOT spend time making splits, so it is a legit SweepBVH implementation.

// Bittner makes things worse in an almost optimized tree.
// Try splits in Bittner
// Frozen not working unless leaf collapse is done.
