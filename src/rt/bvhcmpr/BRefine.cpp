// XXX
#define FW_ENABLE_ASSERT

#include "bvhcmpr/Refine.hpp"
#include "bvhcmpr/BRefine.hpp"
#include "base/Timer.hpp"

#include <algorithm>

#include <intrin.h>

namespace FW
{

    BRefine::BRefine(Refine& ref, Params& rparams)
        : m_bvh(ref.getBVH()),
        m_refine(ref),
        m_platform(ref.getBVH().getPlatform()),
        m_rparams(rparams)
    {
    }

    BRefine::~BRefine(void)
    {
    }

#if 0
    //
    // The unfinished prototype of my parallel algorithm
    //
    
    // Compute the SAH of the subtree that will change size from adding or removing a node
    // node - a node whose AABB will change
    // modBounds - the proposed new AABB of that node
    // modSAH - The SAH of node (whole treelet) if we do the change
    // Returns a pair containing:
    //   the highest ancestor whose area will change if the proposed change were done
    //   and the new SAH of that ancestor's whole treelet
    OptPair_T BRefine::computeModSAH(BVHNode* node, AABB modBounds, float modSAH, bool isShrink)
    {
        FW_ASSERT(!node->isLeaf());
        FW_ASSERT(node->m_parent);

        // modSAH and modBounds now represent modified node's subtree
        BVHNode* mod = node;
        BVHNode* dad = mod->m_parent;
        float lastModSAH = modSAH;
        while (dad) { // Generalize this test for insert / delete
            int ni = dad->getChildIndex(mod);
            FW_ASSERT(ni == 0 || ni == 1);
            int si = 1 - ni;
            BVHNode* sib = dad->getChildNode(si);

            lastModSAH = modSAH;

            // modBounds will become dad's new bounds
            // Add in sib
            modBounds.grow(sib->m_bounds);
            modSAH += sib->m_sah;

            // dad's own cost
            modSAH += m_platform.getCost(dad->getNumChildNodes(), 0) * modBounds.area() / m_rootArea;

            printf("%c", (modBounds.area() < dad->m_bounds.area()) ? '<' : (modBounds.area() > dad->m_bounds.area()) ? '>' : '=');
            printf("newA=%f oldA=%f newSAH=%f oldSAH=%f\n", modBounds.area(), dad->m_bounds.area(), modSAH, dad->m_sah);

            if(isShrink)
                FW_ASSERT(modBounds.area() <= dad->m_bounds.area());
            else
                FW_ASSERT(modBounds.area() >= dad->m_bounds.area());

            if (!((isShrink && modBounds.area() < dad->m_bounds.area()) ||
                 (!isShrink && modBounds.area() > dad->m_bounds.area())))
                // Do the compare before moving the pointers up so that we can return the highest changed node, rather than its parent
                break;

            mod = dad;
            dad = mod->m_parent;
        }

        OptPair_T treeletTopPr(lastModSAH, mod);
        return treeletTopPr;
    }

    OptPair_T BRefine::newFindInsertTarget(BVHNode* node, BVHNode* treeletTop)
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
            if (cIx + nArea >= cBest) // Our best branch isn't good enough so we're done.
                // cIx + nArea is the lower bound of inserting at or below x. nArea is lower bound of x U n area.
                break;

            if (x == treeletTop) {
                BVHNode* nextBest = m_toSearch.size() > 1 ? m_toSearch[1].second : nullptr;
                float nextBestC = m_toSearch.size() > 1 ? m_toSearch[1].first : FW_F32_MAX;
                printf("Best place is home: 0x%016llx 0x%016llx %lld %f; next best is 0x%016llx at %f\n", (U64)node, (U64)treeletTop, m_toSearch.size(), cIx, (U64)nextBest, nextBestC);
                continue;
            }

            // Compute total cost of merging node with x
            float cD = (x->m_bounds + node->m_bounds).area(); // Convert to SAH by multiplying by traversal cost and divide by rootArea
            float cT = cIx + cD; // Total cost to make node a sibling of x
            if (cT < cBest) {
                cBest = cT;
                xBest = x;
            }

            // Calculate the induced cost for children of x
            float cI = cT - x->m_bounds.area();

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

        // If xBest is null it means that treeletTop is root; have to find a way to insert in treeletTop.
        FW_ASSERT(xBest);

        m_toSearch.resize(0);

        OptPair_T rt(cBest, xBest);
        return rt;
    }

    // Find a treelet that's in the wrong place
    float BRefine::planRemoveReplace(BVHNode* node)
    {
        // If this node were removed how would the SAH change?
        if (node->m_parent == nullptr || node->m_parent->m_parent == nullptr)
            return 0;

        FW_ASSERT(!node->isLeaf());
        FW_ASSERT(node->m_parent);
        FW_ASSERT(node->m_parent->m_parent);

        InnerNode* dadi = dynamic_cast<InnerNode*>(node->m_parent);
        int ni = dadi->getChildIndex(node);
        int si = 1 - ni;
        FW_ASSERT(ni == 0 || ni == 1);
        FW_ASSERT(dadi->m_children[ni] == node);
        BVHNode* sib = dadi->m_children[si];

        // No changes have been made. We just know who the players are.
        // Replace dad with sib. Remove node and dad.
        OptPair_T result = computeModSAH(dadi, sib->m_bounds, sib->m_sah, true);

        printf("If we remove 0x%016llx treelet top is 0x%016llx SAH change: %g - %g = %g oldA=%f\n",
            (U64)node, (U64)result.second, result.first, result.second->m_sah, result.first - result.second->m_sah,
            result.second->getArea());

        OptPair_T fresult = newFindInsertTarget(node, result.second);

        // XXX Need to deal with unequal costs of root and internal nodes.
        float dSAH = m_platform.getCost(fresult.second->getNumChildNodes(), 0) * fresult.first / m_rootArea;

        printf("If we reinsert 0x%016llx as sibling of 0x%016llx @ %g SAH change: %g (%g)\n", (U64)node, (U64)fresult.second, fresult.second->m_sah, dSAH, fresult.first);

        // Place it next to x and recompute ancestors of x
        AABB xPlusN = fresult.second->m_bounds + node->m_bounds;
        float xPlusNSAH = fresult.second->m_sah + node->m_sah + m_platform.getCost(fresult.second->getNumChildNodes(), 0) * xPlusN.area() / m_rootArea;
        OptPair_T iresult = computeModSAH(fresult.second, xPlusN, xPlusNSAH, false);

        printf("Reinsert treelet top is 0x%016llx SAH change: SAH change: %g - %g = %g oldA=%f\n\n", 
            (U64)iresult.second, iresult.first, iresult.second->m_sah, iresult.first - iresult.second->m_sah, iresult.second->getArea());

        return result.first;
    }

    void BRefine::newComputePriority(BVHNode* node)
    {
        if (node->isLeaf())
            return;

        for (int i = 0; i < node->getNumChildNodes(); i++) {
            newComputePriority(node->getChildNode(i));
        }

        float dSAH = planRemoveReplace(node);
        OptPair_T pr(dSAH, node);
        m_toOptimize.push_back(pr);
    }

#endif















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

        float metric = 0;
        switch (m_rparams.priorityMetric) {
        case PRIORITY_AREA:
            metric = a;
            break;
        case PRIORITY_MIN_RATIO:
            metric = a / minChArea;
            break;
        case PRIORITY_SUM_RATIO:
            metric = a / sumChArea;
            break;
        case PRIORITY_MIN_AND_SUM:
            metric = a * a / (sumChArea * minChArea);
            break;
        case PRIORITY_ALL_THREE:
            metric = a * a * a / (sumChArea * minChArea);
            break;
        case PRIORITY_RANDOM:
            metric = (float)(rand() * rand());
            break;
        }

        OptPair_T pr(metric, node);
        m_toOptimize.push_back(pr);
    }

    // Original Bittner algorithm
    // Remove a high cost node (because its children don't belong together)
    void BRefine::remove2ForOpt(BVHNode* node)
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
            FW_ASSERT(cur->m_parent || cur == m_bvh.getRoot());
            // float oldArea = cur->getArea();
            cur->computeValues(m_platform, m_rootArea, true, true);
            // float newArea = cur->getArea();
            cur = cur->m_parent;
            // printf("%c", (newArea < oldArea) ? '<' : (newArea > oldArea) ? '>' : '=');
        }
        // printf("\n");
    }

    // My algorithm
    // Find a treelet that's in the wrong place
    void BRefine::remove1ForOpt(BVHNode* node)
    {
        FW_ASSERT(!node->isLeaf());

        if (node->m_parent == nullptr)
            return;

        FW_ASSERT(node->m_parent);

        // Try pairing each child with node's sibling

        BVHNode* dad = node->m_parent;
        InnerNode* dadi = dynamic_cast<InnerNode*>(dad);
        InnerNode* nodei = dynamic_cast<InnerNode*>(node);

        int ni = 0;
        for (ni = 0; ni < dad->getNumChildNodes(); ni++)
            if (dad->getChildNode(ni) == node)
                break;

        FW_ASSERT(ni == 0 || ni == 1);
        FW_ASSERT(dadi->m_children[ni] == node);

        BVHNode* sib = dadi->m_children[1 - ni];

        // Which is the best child to leave here? Choose the one with smallest area.
        // Is this the best? What about affect on ancestors? What about where we insert it?
        int bestI = rand() % 1;
        float bestA = FW_F32_MAX;
        if (!m_rparams.removeRandomChild)
            for (int i = 0; i < node->getNumChildNodes(); i++) {
                BVHNode* ch = node->getChildNode(i);
                AABB sibch = sib->m_bounds + ch->m_bounds;
                if (sibch.area() < bestA) {
                    bestI = i;
                    bestA = sibch.area();
                }
            }
        FW_ASSERT(bestI == 0 || bestI == 1);
        int badI = 1 - bestI;
        nodei->m_children[badI]->m_parent = nullptr;

        OptPair_T pr(nodei->m_children[badI]->getArea(), nodei->m_children[badI]);
        m_toInsert.push_back(pr);

        dadi->m_children[ni] = nodei->m_children[bestI];
        nodei->m_children[bestI]->m_parent = dad;

        // Tag the node as unused and toss it.
        // Don't delete it because it may be in the toOptimize list and will get referenced later.
        node->m_parent = nullptr;
        m_toRecycle.push_back(OptPair_T(0.f, node));

        // Recompute its ancestors
        BVHNode* cur = dad;
        while (cur) {
            FW_ASSERT(cur->m_parent || cur == m_bvh.getRoot());
            //float oldArea = cur->getArea();
            cur->computeValues(m_platform, m_rootArea, true, true);
            //float newArea = cur->getArea();
            cur = cur->m_parent;
            //printf("%c", (newArea < oldArea) ? '<' : (newArea > oldArea) ? '>' : '=');
        }
        //printf("\n");
    }

    // Branch-and-bound algorithm
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
            if (cIx + nArea >= cBest) // Our best branch isn't good enough so we're done.
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
            // x is root
            FW_ASSERT(x == m_bvh.getRoot());
            // float oldRootArea = m_bvh.getRoot()->getArea();
            // printf("%08x %08x %08x\n", m_bvh.getRoot(), x, in);
            m_bvh.setRoot(in);
            // printf("Replacing root. Old area=%f New area=%f Actual area=%f node area=%f\n", oldRootArea, m_bvh.getRoot()->getArea(), m_rootArea, n->getArea());
        }
        in->m_parent = gpa;

        x->m_parent = n->m_parent = in;
        in->m_children[0] = x;
        in->m_children[1] = n;

        // Recompute its ancestors
        BVHNode* cur = in;
        while (cur) {
            FW_ASSERT(cur->m_parent || cur == m_bvh.getRoot());
            // float oldArea = cur->getArea();
            cur->computeValues(m_platform, m_rootArea, true, true);
            // float newArea = cur->getArea();
            cur = cur->m_parent;
            // printf("%c", (newArea < oldArea) ? '<' : (newArea > oldArea) ? '>' : '=');
        }
        // printf("\n");
        m_nodesInserted++;
    }

    // Declare this with non-templated args so that std::sort can find it instead of getting ambiguous result
    void swap(OptPair_T& lhs, OptPair_T& rhs)
    {
        std::swap(lhs, rhs);
    }

    void BRefine::run()
    {
        Timer progressTimer(true);

        bool timeOut = false;

        m_rootArea = m_bvh.getRoot()->getArea();
        float oldSAH = m_bvh.getRoot()->m_sah;
        for (int iter = 0; iter < m_rparams.maxLoops; iter++) {
            m_nodesInserted = 0;

            // Compute node priorities
            m_toOptimize.resize(0);
            // newComputePriority(m_bvh.getRoot());
            computePriority(m_bvh.getRoot());

            // Sort nodes by priority
            std::sort(m_toOptimize.begin(), m_toOptimize.end(), [](OptPair_T& a, OptPair_T& b) { return a.first > b.first; });

            size_t batchSize = min(size_t(m_rparams.batchSize * (float)m_toOptimize.size()) + 1, m_toOptimize.size()/10);

            for (int i = 0; i < batchSize; i++) {
                // Remove a node
                BVHNode* node = m_toOptimize[i].second;

                // printf("%d Removing node 0x%016llx weight=%f area=%f\n", i, (U64)node, m_toOptimize[i].first, node->getArea());
                if (m_rparams.nodesToRemove == 1)
                    remove1ForOpt(node);
                else
                    remove2ForOpt(node);

                // Insert removed nodes
                std::sort(m_toInsert.begin(), m_toInsert.end(), [](OptPair_T& a, OptPair_T& b) { return a.first > b.first; });
                for (auto nn : m_toInsert) {
                    // printf("m_toOptimize=%d m_toInsert=%d m_toRecycle=%d m_toSearch=%d\n", m_toOptimize.size(), m_toInsert.size(), m_toRecycle.size(), m_toSearch.size());
                    // printf("%d Inserting node 0x%016llx weight=%f area=%f\n", i, (U64)nn.second, nn.first, nn.second->getArea());
                    insertForOpt(nn.second);
                }
                m_toInsert.resize(0);

                if ((i % 32) == 0 && m_rparams.timeBudget > 0 && progressTimer.getElapsed() > m_rparams.timeBudget) {
                    timeOut = true;
                    break;
                }
            }
            
            float sah = m_bvh.getRoot()->m_sah;
            float impr = (oldSAH - sah) / sah;
            oldSAH = sah;

            printf("Bitnr %d heur=%d insert=%d batch=%lld worst100=%f worst90=%f nr=%d rndch=%d ", iter,
                m_rparams.priorityMetric, m_nodesInserted, batchSize, m_toOptimize[0].first,
                m_toOptimize[size_t(m_toOptimize.size()*0.1f)].first, m_rparams.nodesToRemove, m_rparams.removeRandomChild);
            printf("dsah=%1.2f%% sah=%.6f tt=%f t=%f\n", 100.f * impr, sah, m_refine.getTimer().getTotal(), m_refine.getTimer().end());
            oldSAH = sah;

            //if (timeOut || (impr < m_rparams.perPassImprovement && impr >= 0.f))
            if (timeOut || (impr < m_rparams.perPassImprovement))
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

// My one-node deletion allows removing children of the root, but the original only allows grandchildren.

// Should try to get Bittner to respect the leaf collapse plan of TRBVH

// On Bittner, cycle through the different heuristics to get nodes that are bad in any way

// Once we know the best place for a node is inside its modified treelet we could do a TRBVH on that treelet.

// TRBVH puts lots of work into optimizing a whole treelet, then throws most of the work away as it steps up to the next level of the tree.

// SBVH alpha parameter claims to avoid blowing up splits, but this isn't true. It can still blow up. Alpha just dampens it to only blow up when useful to the SAH. Still no regard for memory.
// Mine provides a breadth-first build with a hard memory limit, so splits happen where they theoretically matter the most.

