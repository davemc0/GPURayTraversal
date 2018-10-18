
#include "bvhcmpr/Refine.hpp"

#include <algorithm>

#include <intrin.h>

namespace FW
{

    Refine::Refine(BVH& bvh, const BVH::BuildParams& params, const RefineParams& rparams)
        : m_bvh(bvh),
        m_platform(bvh.getPlatform()),
        m_params(params),
        m_rparams(rparams),
		m_optTreeletsDone(0),
		m_optTreeletsOutput(0)

    {
    }

    //------------------------------------------------------------------------

    Refine::~Refine(void)
    {
    }

    //------------------------------------------------------------------------

	// XXXTree depth-first treelet forming as a way of reaching across many layers of splits
	// !!!!!!!!!!!!!!!!!!!

    void Refine::run()
    {
        // Fill in necessary node stats - area and tri count

        // Allocate temporary stuff
        m_sahes.resize(1ull << m_rparams.nTrLeaves);
        m_copt.resize(1ull << m_rparams.nTrLeaves);
        m_popt.resize(1ull << m_rparams.nTrLeaves);

		m_treeletHeur = TreeletHeur::TREELET_CYCLE;
		TreeletHeur lastHeur = m_treeletHeur;

		for (int i = 0; i < m_rparams.maxLoops; i++) {
			if (m_rparams.treeletHeuristic == TreeletHeur::TREELET_CYCLE) {
				m_treeletHeur = static_cast<TreeletHeur>((m_treeletHeur + 1) % TreeletHeur::TREELET_CYCLE);
			}
			else if (m_rparams.treeletHeuristic == TreeletHeur::TREELET_CYCLEGR) {
				if (m_treeletHeur == TreeletHeur::TREELET_GREATER)
					m_treeletHeur = TreeletHeur::TREELET_RANDOM;
				else
					m_treeletHeur = TreeletHeur::TREELET_GREATER;
			}
			else {
				m_treeletHeur = m_rparams.treeletHeuristic;
			}

			printf("%d Forming nTr=%d treelets with heur=%d; ", i, m_rparams.nTrLeaves, m_treeletHeur);
			m_optTreeletsDone = 0;
			m_optTreeletsOutput = 0;
			BVHNode* root = m_bvh.getRoot();
			refineNode(root); // <-------------- THIS IS WHERE WE DO THE WORK

			float optSAH = root->m_sah; // The SAH calculated by the optimizer
			//if ((i % 100) == 0) root->computeSubtreeSAHValues(m_platform, root->getArea()); // The measured SAH; optimization may not touch the root
			if (m_params.enablePrints) {
				printf("optimized sah: %.6f %.6f opts: %d/%d\n", root->m_sah, optSAH, m_optTreeletsOutput, m_optTreeletsDone);
				// m_bvh.printTree(root);
			}

			// Terminate when same heuristic fails twice
			if (m_optTreeletsOutput > 0) {
				lastHeur = m_treeletHeur;
			}
			else if (lastHeur == m_treeletHeur) {
				//((Refine::RefineParams&)m_rparams).freezeThreshold = 100;
				break;
			}
		}
	}

    // Form a treelet rooted at tRoot based on predicate Pred
    // When finished, internals contains all internal nodes, internals[0] is tRoot, and leaves contains all the leaves.
    template<class Pr>
    void Refine::formTreeletPred(BVHNode* tRoot, int nTrLeaves, std::vector<BVHNode*>& internals, std::vector<BVHNode*>& leaves, Pr Pred)
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

	void Refine::formTreelet(BVHNode* tRoot, int nTrLeaves, std::vector<BVHNode*>& internals, std::vector<BVHNode*>& leaves)
	{
		switch (m_treeletHeur) {
		case TreeletHeur::TREELET_GREATER:
			formTreeletPred(tRoot, nTrLeaves, internals, leaves,
				[](const void* aa, const void* bb) { BVHNode* a = (BVHNode*)aa; BVHNode* b = (BVHNode*)bb;  return (a->isLeaf() && !b->isLeaf()) ? false : (b->isLeaf() && !a->isLeaf()) ? true : a->getArea() > b->getArea(); }); // This worked.
			break;
		case TreeletHeur::TREELET_LESS:
			formTreeletPred(tRoot, nTrLeaves, internals, leaves,
				[](const void* aa, const void* bb) { BVHNode* a = (BVHNode*)aa; BVHNode* b = (BVHNode*)bb;  return (a->isLeaf() && !b->isLeaf()) ? false : (b->isLeaf() && !a->isLeaf()) ? true : a->getArea() < b->getArea(); }); // a is the one that's already there and b is the one it's inserting
			break;
		case TreeletHeur::TREELET_RANDOM:
			formTreeletPred(tRoot, nTrLeaves, internals, leaves,
				[](const void* aa, const void* bb) { BVHNode* a = (BVHNode*)aa; BVHNode* b = (BVHNode*)bb;  return (a->isLeaf() && !b->isLeaf()) ? false : (b->isLeaf() && !a->isLeaf()) ? true : !(rand() % 3); }); // 1/3 true
			break;
		case TreeletHeur::TREELET_TRUE:
			formTreeletPred(tRoot, nTrLeaves, internals, leaves,
				[](const void* aa, const void* bb) { BVHNode* a = (BVHNode*)aa; BVHNode* b = (BVHNode*)bb;  return (a->isLeaf() && !b->isLeaf()) ? false : (b->isLeaf() && !a->isLeaf()) ? true : true; });
			break;
		case TreeletHeur::TREELET_FALSE:
			formTreeletPred(tRoot, nTrLeaves, internals, leaves,
				[](const void* aa, const void* bb) { BVHNode* a = (BVHNode*)aa; BVHNode* b = (BVHNode*)bb;  return (a->isLeaf() && !b->isLeaf()) ? false : (b->isLeaf() && !a->isLeaf()) ? true : false; });
			break;
		}
	}

#if 0
    // Refine some treelet rooted at tRoot
    // Kensler 2008
    bool Refine::refineTreelet(BVHNode* tRoot)
    {
        std::vector<BVHNode*> internals;
        std::vector<BVHNode*> leaves;

		formTreelet(tRoot, m_rparams.nTrLeaves, internals, leaves);

        if (leaves.size() < m_rparams.nTrLeaves)
            return;

        // Refine treelet
        // printf("Refining treelet with %d leaves: nI=%d nL=%d\n", m_rparams.nTrLeaves, internals.size(), leaves.size());
        // for (auto i : leaves)
        //     printf("%c=>%f  ", i->isLeaf() ? 'L' : 'I', i->getArea());
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

#else

	// Return the index of the first set (one) bit
	inline U32 ffs(U32 x)
	{
		unsigned long ind;
		_BitScanForward(&ind, x);
		return ind;
	}

	BVHNode* Refine::formNodes(std::vector<BVHNode*>& internals, std::vector<BVHNode*>& leaves, U32 s)
	{
		float rootArea = m_bvh.getRoot()->getArea();

		if (__popcnt(s) == 1)
			return leaves[ffs(s)];

		U32 p = m_popt[s];

		if (p == 0) {
			// XXX Flatten all leaves in s into one leaf
			printf("Epic fail\n");
		}

		BVHNode* l = formNodes(internals, leaves, p);
		BVHNode* r = formNodes(internals, leaves, s ^ p);

		InnerNode* in = dynamic_cast<InnerNode*>(internals.back());
		internals.pop_back();

		in->m_children[0] = l;
		in->m_children[1] = r;
		in->m_bounds = l->m_bounds;
		in->m_bounds.grow(r->m_bounds);
		in->m_probability = in->getArea() / rootArea;
		in->m_sah = m_copt[s];
		in->m_tris = l->m_tris + r->m_tris;
		in->m_frozen = 0;

		return in;
	}

	// True if b sufficiently less than a
	inline bool fuzzyDiff(const float a, const float b, const float eps = 1e-7)
	{
		float d = a - b;
		float s = d / a;
		return s > eps;
	}

	// Refine some treelet rooted at tRoot
    bool Refine::refineTreelet(BVHNode* tRoot)
    {
        const int nL = m_rparams.nTrLeaves;

        std::vector<BVHNode*> internals;
        std::vector<BVHNode*> leaves;

		formTreelet(tRoot, nL, internals, leaves);

		if (leaves.size() < nL) {
			tRoot->m_frozen++;
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
						if((1ul << i) & s)
							sTris += leaves[i]->m_tris;
					//float SAHAsLeaf = m_platform.getCost(0, sTris) * m_sahes[s];
					float SAHAsLeaf = 9999999999.0f; // XXX
					float SAHAsTreelet = m_platform.getCost(2, 0) * m_sahes[s] + bestC; // My cost plus the cost of my two children
					m_copt[s] = min(SAHAsTreelet, SAHAsLeaf);
					if (SAHAsLeaf < SAHAsTreelet) {
						bestP = 0; // Force leaf
						printf("forceLeaf 0x%x 0x%x %f < %f\n", s, p, SAHAsLeaf, SAHAsTreelet);
					}
					m_popt[s] = bestP;
				}
			}
		}

		m_optTreeletsDone++;

		// Construct treelet
		U32 rti = (1ul << nL) - 1;
		if (fuzzyDiff(tRoot->m_sah, m_copt[rti])) {
			//float diff = tRoot->m_sah - m_copt[rti];
			//printf("\nTreelet: 0x%08x %.8g - %.8g = %.8g  %.8g\n", tRoot, tRoot->m_sah, m_copt[rti], diff, m_copt[rti]);
			//m_bvh.printTree(tRoot);

			m_optTreeletsOutput++;

			// Because tRoot == internals[0] it properly attaches the subtree root
			formNodes(internals, leaves, rti);

			FW_ASSERT(internals.size() == 0);
			
			//printf("\n");
			//m_bvh.printTree(tRoot);

			return true;
		}

		tRoot->m_frozen++;

		return false;
	}

#endif

    bool Refine::refineNode(BVHNode* node)
    {
		// Abort if this node isn't big enough to form the treelet
		if (node->m_tris < m_rparams.nTrLeaves || node->isLeaf())
			return false;

		// Abort if this subtree has not improved in too long
		if (node->m_frozen > m_rparams.freezeThreshold) {
			//printf("f");
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

	void Refine::collapseLeaves()
	{
		// Depth-first traversal to insert triIndices into new triIndices array
		//Array<S32>& tris = m_bvh.getTriIndices();
		//for (int i = 0; i < spec.numRef; i++)
		//	tris.add(m_refStack.removeLast().triIdx);
		//return new LeafNode(spec.bounds, tris.getSize() - spec.numRef, tris.getSize());

	}
};
