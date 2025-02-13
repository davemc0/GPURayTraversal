/*
 *  Copyright (c) 2009-2011, NVIDIA Corporation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *      * Neither the name of NVIDIA Corporation nor the
 *        names of its contributors may be used to endorse or promote products
 *        derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// XXX
#define FW_ENABLE_ASSERT

#include "bvh/GPUSplitBVHBuilder.hpp"
#include "base/Sort.hpp"

#include <algorithm>

#if !FW_CUDA10
#include <execution>
const auto parseq = std::execution::par_unseq;
#define NOPAR 0
#else
#define NOPAR 1
#endif

//------------------------------------------------------------------------

FW::GPUSplitBVHBuilder::GPUSplitBVHBuilder(FW::BVH& bvh, const FW::BVH::BuildParams& params)
:   m_bvh           (bvh),
    m_platform      (bvh.getPlatform()),
    m_params        (params),
    m_minOverlap    (0.0f),
    m_sortDim       (-1)
{
}

//------------------------------------------------------------------------

FW::GPUSplitBVHBuilder::~GPUSplitBVHBuilder(void)
{
}

//------------------------------------------------------------------------

FW::BVHNode* FW::GPUSplitBVHBuilder::run(void)
{
    if (m_params.enablePrints)
        printf("SBVH alpha=%g minLeafSize=%d maxLeafSize=%d\n", m_params.splitAlpha, m_platform.getMinLeafSize(), m_platform.getMaxLeafSize());

    // Initialize reference stack and determine root bounds.

    const Vec3i* tris = (const Vec3i*)m_bvh.getScene()->getTriVtxIndexBuffer().getPtr();
    const Vec3f* verts = (const Vec3f*)m_bvh.getScene()->getVtxPosBuffer().getPtr();

    NodeSpec rootSpec;
    rootSpec.numRef = m_bvh.getScene()->getNumTriangles();
    m_refStack.resize(rootSpec.numRef);
    m_reqIndices.resize(rootSpec.numRef);
    FW_ASSERT(m_reqIndices.getSize() == m_refStack.getSize());

#if NOPAR
    for (int i = 0; i < rootSpec.numRef; i++)
    {
        m_refStack[i].triIdx = i;
        for (int j = 0; j < 3; j++)
            m_refStack[i].bounds.grow(verts[tris[i][j]]);
        rootSpec.bounds.grow(m_refStack[i].bounds);
    }
#else
    rootSpec.bounds = std::transform_reduce(parseq, &m_refStack[0], &m_refStack[0] + rootSpec.numRef, AABB(), [](AABB a, AABB b) {return a + b; }, [&](Reference & ir) {
        S32 i = (S32)(&(ir)-&m_refStack[0]);
        if (i >= rootSpec.numRef)
            printf("!!!!!!!!!!!! %d %d\n", i, rootSpec.numRef);

        m_refStack[i].triIdx = i;
        for (int j = 0; j < 3; j++)
            m_refStack[i].bounds.grow(verts[tris[i][j]]);

        return m_refStack[i].bounds;
    });
#endif

    // Initialize rest of the members.

    m_minOverlap = rootSpec.bounds.area() * m_params.splitAlpha;
    m_rightBounds.reset(max(rootSpec.numRef, (int)NumSpatialBins) - 1);
    m_intersectReqs.resize(rootSpec.numRef); // XXX I have no idea if this is a good number.
    m_numDuplicates = 0;
    m_progressTimer.start();

    // Build recursively.
    FW_ASSERT(m_reqIndices.getSize() == m_refStack.getSize());

    BVHNode* root = buildNode(rootSpec, 0, 0.0f, 1.0f);
    m_bvh.getTriIndices().compact(); // Delete the BVH's m_triIndices

    // Done.

    if (m_params.enablePrints)
        printf("GPUSplitBVHBuilder: progress %.0f%%, duplicates %.0f%%\n",
            100.0f, (F32)m_numDuplicates / (F32)m_bvh.getScene()->getNumTriangles() * 100.0f);
    return root;
}

//------------------------------------------------------------------------

bool FW::GPUSplitBVHBuilder::sortCompare(void* data, int idxA, int idxB)
{
    const GPUSplitBVHBuilder* ptr = (const GPUSplitBVHBuilder*)data;
    int dim = ptr->m_sortDim;
    const Reference& ra = ptr->m_refStack[idxA];
    const Reference& rb = ptr->m_refStack[idxB];
    F32 ca = ra.bounds.min()[dim] + ra.bounds.max()[dim];
    F32 cb = rb.bounds.min()[dim] + rb.bounds.max()[dim];
    return (ca < cb || (ca == cb && ra.triIdx < rb.triIdx));
}

//------------------------------------------------------------------------

void FW::GPUSplitBVHBuilder::sortSwap(void* data, int idxA, int idxB)
{
    GPUSplitBVHBuilder* ptr = (GPUSplitBVHBuilder*)data;
    swap(ptr->m_refStack[idxA], ptr->m_refStack[idxB]);
}

//------------------------------------------------------------------------

FW::BVHNode* FW::GPUSplitBVHBuilder::buildNode(NodeSpec spec, int level, F32 progressStart, F32 progressEnd)
{
    // Display progress.

    if (m_params.enablePrints && m_progressTimer.getElapsed() >= 1.0f)
    {
        printf("GPUSplitBVHBuilder: progress %.0f%%, duplicates %.0f%%\r",
            progressStart * 100.0f, (F32)m_numDuplicates / (F32)m_bvh.getScene()->getNumTriangles() * 100.0f);
        m_progressTimer.start();
    }

    // Remove degenerates.
    // XXX I think this is only necessary at the root. Below that the splitter should be able to not make degenerates.
    {
        int firstRef = m_refStack.getSize() - spec.numRef;
        for (int i = m_refStack.getSize() - 1; i >= firstRef; i--)
        {
            Vec3f size = m_refStack[i].bounds.max() - m_refStack[i].bounds.min();
            if (min(size) < 0.0f || sum(size) == max(size))
                m_refStack.removeSwap(i);
        }
        spec.numRef = m_refStack.getSize() - firstRef;
    }

    // Small enough or too deep => create leaf.

    if (spec.numRef <= m_platform.getMinLeafSize() || level >= MaxDepth) {
        m_params.stats->forcedLeaves++;
        //printf("spec.numRef=%d m_platform.getMinLeafSize()=%d\n", spec.numRef, m_platform.getMinLeafSize());
        return createLeaf(spec);
    }

    // Find split candidates.

    F32 area = spec.bounds.area();
    F32 leafSAH = area * m_platform.getTriangleCost(spec.numRef);

    F32 nodeSAH = area * m_platform.getNodeCost(2); // num children
    ObjectSplit object = findObjectSplit(spec, nodeSAH);

    SpatialSplit spatial;
    if (level < MaxSpatialDepth)
    {
        AABB overlap = object.leftBounds;
        overlap.intersect(object.rightBounds);
        if (overlap.area() >= m_minOverlap)
            spatial = findSpatialSplit(spec, nodeSAH);
    }

    // Leaf SAH is the lowest => create leaf.

    F32 minSAH = min(leafSAH, object.sah, spatial.sah);
    if (minSAH == leafSAH && spec.numRef <= m_platform.getMaxLeafSize())
        return createLeaf(spec);

    // Perform split.

    NodeSpec left, right;
    if (minSAH == spatial.sah)
        performSpatialSplit(left, right, spec, spatial);
    if (!left.numRef || !right.numRef)
        performObjectSplit(left, right, spec, object);

    // Create inner node.

    m_numDuplicates += left.numRef + right.numRef - spec.numRef;
    F32 progressMid = lerp(progressStart, progressEnd, (F32)right.numRef / (F32)(left.numRef + right.numRef));
    BVHNode* rightNode = buildNode(right, level + 1, progressStart, progressMid);
    BVHNode* leftNode = buildNode(left, level + 1, progressMid, progressEnd);
    return new InnerNode(spec.bounds, leftNode, rightNode);
}

//------------------------------------------------------------------------

FW::BVHNode* FW::GPUSplitBVHBuilder::createLeaf(const NodeSpec& spec)
{
	// Remove the node's triangles from refStack and add them to tris
	//printf("%d", spec.numRef);
    Array<S32>& tris = m_bvh.getTriIndices();
    for (int i = 0; i < spec.numRef; i++)
        tris.add(m_refStack.removeLast().triIdx);
    return new LeafNode(spec.bounds, tris.getSize() - spec.numRef, tris.getSize());
}

//------------------------------------------------------------------------

FW::GPUSplitBVHBuilder::ObjectSplit FW::GPUSplitBVHBuilder::findObjectSplit(const NodeSpec& spec, F32 nodeSAH)
{
    ObjectSplit split;
    const Reference* refPtr = m_refStack.getPtr(m_refStack.getSize() - spec.numRef);
    F32 bestTieBreak = FW_F32_MAX;

    // Sort along each dimension.

    for (m_sortDim = 0; m_sortDim < 3; m_sortDim++)
    {
#if NOPAR
        sort(this, m_refStack.getSize() - spec.numRef, m_refStack.getSize(), sortCompare, sortSwap, m_params.doMulticore);
#else
        // PAR: Sort by centroid
        std::sort(parseq, &m_refStack[m_refStack.getSize() - spec.numRef], &m_refStack[0] + m_refStack.getSize(), [&](const Reference& ra, const Reference& rb) {
            int dim = m_sortDim;
            F32 ca = ra.bounds.min()[dim] + ra.bounds.max()[dim];
            F32 cb = rb.bounds.min()[dim] + rb.bounds.max()[dim];
            return (ca < cb || (ca == cb && ra.triIdx < rb.triIdx));
        });
#endif

        // Sweep right to left and determine bounds.
        // PAR: Is an inclusive scan
        AABB rightBounds;
        for (int i = spec.numRef - 1; i > 0; i--)
        {
            rightBounds.grow(refPtr[i].bounds);
            m_rightBounds[i - 1] = rightBounds;
        }

        // Sweep left to right and select lowest SAH.
        // PAR: Is an inclusive scan
        AABB leftBounds;
#if 1
        for (int i = 1; i < spec.numRef; i++)
        {
            leftBounds.grow(refPtr[i - 1].bounds);
            F32 sah = nodeSAH + leftBounds.area() * m_platform.getTriangleCost(i) + m_rightBounds[i - 1].area() * m_platform.getTriangleCost(spec.numRef - i);
            F32 tieBreak = sqr((F32)i) + sqr((F32)(spec.numRef - i));
            if (sah < split.sah || (sah == split.sah && tieBreak < bestTieBreak))
            {
                split.sah = sah;
                split.sortDim = m_sortDim;
                split.numLeft = i;
                split.leftBounds = leftBounds;
                split.rightBounds = m_rightBounds[i - 1];
                bestTieBreak = tieBreak;
            }
        }
#else

#endif
    }
    return split;
}

//------------------------------------------------------------------------

void FW::GPUSplitBVHBuilder::performObjectSplit(NodeSpec& left, NodeSpec& right, const NodeSpec& spec, const ObjectSplit& split)
{
    m_sortDim = split.sortDim;
#if NOPAR
    sort(this, m_refStack.getSize() - spec.numRef, m_refStack.getSize(), sortCompare, sortSwap, m_params.doMulticore);
#else
    std::sort(parseq, &m_refStack[m_refStack.getSize() - spec.numRef], &m_refStack[0] + m_refStack.getSize(), [&](const Reference& ra, const Reference& rb) {
        int dim = m_sortDim;
        F32 ca = ra.bounds.min()[dim] + ra.bounds.max()[dim];
        F32 cb = rb.bounds.min()[dim] + rb.bounds.max()[dim];
        return (ca < cb || (ca == cb && ra.triIdx < rb.triIdx));
    });
#endif

    left.numRef = split.numLeft;
    left.bounds = split.leftBounds;
    right.numRef = spec.numRef - split.numLeft;
    right.bounds = split.rightBounds;
}

//------------------------------------------------------------------------

FW::GPUSplitBVHBuilder::SpatialSplit FW::GPUSplitBVHBuilder::findSpatialSplit(const NodeSpec& spec, F32 nodeSAH)
{
    // Initialize bins.

    Vec3f origin = spec.bounds.min();
    Vec3f binSize = (spec.bounds.max() - origin) * (1.0f / (F32)NumSpatialBins);
    Vec3f invBinSize = 1.0f / binSize;

#if NOPAR
    for (int dim = 0; dim < 3; dim++)
    {
        for (int i = 0; i < NumSpatialBins; i++)
        {
            SpatialBin& bin = m_bins[dim][i];
            bin.bounds = AABB();
            bin.enter = 0;
            bin.exit = 0;
        }
    }
#else
    std::for_each(parseq, &m_bins[0][0], &m_bins[0][0] + 3 * NumSpatialBins, [](SpatialBin& bin) {
        bin.bounds = AABB();
        bin.enter = 0;
        bin.exit = 0;
    });
#endif

    m_reqIndices.resize(m_refStack.getSize());

    // Chop references into bins.
    S32 st = m_refStack.getSize() - spec.numRef, end = m_refStack.getSize();

    if (spec.numRef > 10000000) {
        printf("findSpatialSplit(%d)", spec.numRef);
        m_progressTimer.start();
    }

    //if (spec.numRef < 20)
#if NOPAR
    {
        for (int refIdx = st; refIdx < end; refIdx++)
        {
            const Reference& ref = m_refStack[refIdx];
            Vec3i firstBin = clamp(Vec3i((ref.bounds.min() - origin) * invBinSize), 0, NumSpatialBins - 1);
            Vec3i lastBin = clamp(Vec3i((ref.bounds.max() - origin) * invBinSize), firstBin, NumSpatialBins - 1);

            for (int dim = 0; dim < 3; dim++)
            {
                Reference currRef = ref;
                for (int i = firstBin[dim]; i < lastBin[dim]; i++)
                {
                    Reference leftRef, rightRef;
                    splitReference(leftRef, rightRef, currRef, dim, origin[dim] + binSize[dim] * (F32)(i + 1));
                    m_bins[dim][i].bounds.grow(leftRef.bounds);
                    currRef = rightRef;
                }
                m_bins[dim][lastBin[dim]].bounds.grow(currRef.bounds);
                m_bins[dim][firstBin[dim]].enter++;
                m_bins[dim][lastBin[dim]].exit++;
            }
        }
    }
#else
    else if (spec.numRef < 2000000000) {
        int bleh[NumHostThreads];

        std::for_each(parseq, &bleh[0], &bleh[0] + NumHostThreads, [&](int& threadIt) {
            S32 th = static_cast<S32>(&threadIt - &bleh[0]);
            // printf("Big thread %d\n", th);

            std::for_each(&m_parBins[th][0][0], &m_parBins[th][0][0] + 3 * NumSpatialBins, [](SpatialBin& bin) {
                bin.bounds = AABB();
                bin.enter = 0;
                bin.exit = 0;
            });

            int itemsPerTh = (spec.numRef + NumHostThreads  - 1) / NumHostThreads;
            int myst = st + itemsPerTh * th;
            int myend = std::min(end, st + itemsPerTh * (th + 1));

            for (int refIdx = myst; refIdx < myend; refIdx++)
            {
                const Reference& ref = m_refStack[refIdx];
                Vec3i firstBin = clamp(Vec3i((ref.bounds.min() - origin) * invBinSize), 0, NumSpatialBins - 1);
                Vec3i lastBin = clamp(Vec3i((ref.bounds.max() - origin) * invBinSize), firstBin, NumSpatialBins - 1);

                for (int dim = 0; dim < 3; dim++)
                {
                    Reference currRef = ref;
                    for (int i = firstBin[dim]; i < lastBin[dim]; i++)
                    {
                        Reference leftRef, rightRef;
                        splitReference(leftRef, rightRef, currRef, dim, origin[dim] + binSize[dim] * (F32)(i + 1));
                        m_parBins[th][dim][i].bounds.grow(leftRef.bounds);
                        currRef = rightRef;
                    }
                    m_parBins[th][dim][lastBin[dim]].bounds.grow(currRef.bounds);
                    m_parBins[th][dim][firstBin[dim]].enter++;
                    m_parBins[th][dim][lastBin[dim]].exit++;
                }
            }
            // printf("Bye thread %d\n", th);
        });

        for (int dim = 0; dim < 3; dim++) {
            for (int bin = 0; bin < NumSpatialBins; bin++) {
                for (int th = 0; th < NumHostThreads; th++) {
                    m_bins[dim][bin].bounds.grow(m_parBins[th][dim][bin].bounds);
                    m_bins[dim][bin].enter += m_parBins[th][dim][bin].enter;
                    m_bins[dim][bin].exit += m_parBins[th][dim][bin].exit;
                }
            }
        }
    }
    else {
        exit(0);
        // PAR: GPU
        // for each tri in parallel:
        //   compute firstBin and lastBin
        // Prefix sum(lastBin-firstBin+1) gives indices into an array of all intersection requests
        std::transform_inclusive_scan(parseq, &m_refStack[0] + st, &m_refStack[0] + end,
            &m_reqIndices[0] + st, std::plus<S32>(), [&](const Reference& ref) {
            // Count how many bins the ref touches in all dimensions
            Vec3i firstBin = clamp(Vec3i((ref.bounds.min() - origin) * invBinSize), 0, NumSpatialBins - 1);
            Vec3i lastBin = clamp(Vec3i((ref.bounds.max() - origin) * invBinSize), firstBin, NumSpatialBins - 1);
            Vec3i nBins = lastBin - firstBin;
            return (S32)(nBins.sum() + 3);
        });

        S32 reqInds = m_reqIndices[end - 1];
        m_intersectReqs.resize(reqInds);

        // For each ref in parallel:
        //   Fill span of this ref's intersection requests (BinTriIntReqs)
        std::for_each(parseq, &m_reqIndices[0] + st, &m_reqIndices[0] + end, [&](auto& cnt) {
            S32 i = static_cast<S32>(&cnt - &m_reqIndices[0]);
            S32 ofs = i > st ? m_reqIndices[i - 1] : 0;
            // S32 opl = m_reqIndices[i]; Don't need one-past-last anymore

            const Reference& ref = m_refStack[i];
            Vec3i firstBin = clamp(Vec3i((ref.bounds.min() - origin) * invBinSize), 0, NumSpatialBins - 1);
            Vec3i lastBin = clamp(Vec3i((ref.bounds.max() - origin) * invBinSize), firstBin, NumSpatialBins - 1);

            for (S8 dim = 0; dim < 3; dim++) {
                for (int b = firstBin[dim]; b <= lastBin[dim]; b++) {
                    BinTriIntReq& req = m_intersectReqs[ofs++];
                    req.ref = i;
                    req.dim = dim;
                    req.bin = b;
                    req.enters = (S8)(b == firstBin[dim] ? 1 : 0);
                    req.exits = (S8)(b == lastBin[dim]) ? 1 : 0;
                }
            }
        });

        // Sort all intersection requests by bin
        std::sort(parseq, &m_intersectReqs[0], &m_intersectReqs[0] + reqInds, [&](auto& a, auto& b) {
            return (a.dim == b.dim) ? (a.bin < b.bin) : (a.dim < b.dim);
        });

        // For each bin in parallel:
        //   Binary search to find start of this bin's intersection requests
        //   For each intersection request
        //     Compute a tri-bin intersection
        //     Accumulate tri-bin intersection into the bin
        std::for_each(parseq, &m_bins[0][0], &m_bins[0][0] + 3 * NumSpatialBins, [&](SpatialBin& binRec) {
            S32 ir = static_cast<S32>(&binRec - &m_bins[0][0]);
            S8 dim = static_cast<S8>(ir / NumSpatialBins);
            S32 bin = ir % NumSpatialBins;
            BinTriIntReq qreq;
            qreq.dim = dim;
            qreq.bin = bin;

            auto it = std::lower_bound(&m_intersectReqs[0], &m_intersectReqs[0] + reqInds, qreq, [&](auto& a, auto& b) {
                return (a.dim == b.dim) ? (a.bin < b.bin) : (a.dim < b.dim);
            });

            S32 i = static_cast<S32>(it - &m_intersectReqs[0]);
            while (i < reqInds && m_intersectReqs[i].dim == dim && m_intersectReqs[i].bin == bin) {
                Reference& ref = m_refStack[m_intersectReqs[i].ref];
                Reference leftRef, midRef, rightRef;
                splitReference(leftRef, midRef, ref, dim, origin[dim] + binSize[dim] * (F32)(bin)); // split on left of bin
                splitReference(leftRef, rightRef, midRef, dim, origin[dim] + binSize[dim] * (F32)(bin + 1)); // split on right of bin
                m_bins[dim][bin].bounds.grow(leftRef.bounds);
                m_bins[dim][bin].enter += m_intersectReqs[i].enters;
                m_bins[dim][bin].exit += m_intersectReqs[i].exits;
                i++;
            }
        });
    }
#endif

    if (spec.numRef > 10000000) {
        printf(" t=%f\n", m_progressTimer.end());
    }

    // for (int dim = 0; dim < 3; dim++) {
    //     for (int bin = 0; bin < NumSpatialBins; bin++) {
    //         printf("dim=%d bin=%d enter=%d exit=%d\n", dim, bin, m_bins[dim][bin].enter, m_bins[dim][bin].exit);
    //     }
    // }
    // exit(0);

    // Select best split plane.

    SpatialSplit split;
    for (int dim = 0; dim < 3; dim++)
    {
        // Sweep right to left and determine bounds.

        AABB rightBounds;
        for (int i = NumSpatialBins - 1; i > 0; i--)
        {
            rightBounds.grow(m_bins[dim][i].bounds);
            m_rightBounds[i - 1] = rightBounds;
        }

        // Sweep left to right and select lowest SAH.

        AABB leftBounds;
        int leftNum = 0;
        int rightNum = spec.numRef;

        for (int i = 1; i < NumSpatialBins; i++)
        {
            leftBounds.grow(m_bins[dim][i - 1].bounds);
            leftNum += m_bins[dim][i - 1].enter;
            rightNum -= m_bins[dim][i - 1].exit;

            F32 sah = nodeSAH + leftBounds.area() * m_platform.getTriangleCost(leftNum) + m_rightBounds[i - 1].area() * m_platform.getTriangleCost(rightNum);
            if (sah < split.sah)
            {
                split.sah = sah;
                split.dim = dim;
                split.pos = origin[dim] + binSize[dim] * (F32)i;
            }
        }
    }
    return split;
}

//------------------------------------------------------------------------

void FW::GPUSplitBVHBuilder::performSpatialSplit(NodeSpec& left, NodeSpec& right, const NodeSpec& spec, const SpatialSplit& split)
{
    // Categorize references and compute bounds.
    //
    // Left-hand side:      [leftStart, leftEnd[
    // Uncategorized/split: [leftEnd, rightStart[
    // Right-hand side:     [rightStart, refs.getSize()[

    Array<Reference>& refs = m_refStack;
    int leftStart = refs.getSize() - spec.numRef;
    int leftEnd = leftStart;
    int rightStart = refs.getSize();
    left.bounds = right.bounds = AABB();

#if NOPAR
    for (int i = leftEnd; i < rightStart; i++)
    {
        // Entirely on the left-hand side?

        if (refs[i].bounds.max()[split.dim] <= split.pos)
        {
            left.bounds.grow(refs[i].bounds);
            swap(refs[i], refs[leftEnd++]);
        }

        // Entirely on the right-hand side?

        else if (refs[i].bounds.min()[split.dim] >= split.pos)
        {
            right.bounds.grow(refs[i].bounds);
            swap(refs[i--], refs[--rightStart]);
        }
    }
#else
    auto lE = std::partition(parseq, &refs[leftEnd], &refs[0] + refs.getSize(), [&](const Reference& r) {return r.bounds.max()[split.dim] <= split.pos; });
    auto rS = std::partition(parseq, lE, &refs[0] + refs.getSize(), [&](const Reference& r) {return r.bounds.min()[split.dim] < split.pos; });
    auto L = std::reduce(parseq, &refs[leftStart], lE, Reference(), [](const Reference& a, const Reference& b) {Reference r; r.bounds = a.bounds + b.bounds; return r; });
    left.bounds = L.bounds;
    auto R = std::reduce(parseq, rS, &refs[0] + refs.getSize(), Reference(), [](const Reference& a, const Reference& b) {Reference r; r.bounds = a.bounds + b.bounds; return r; });
    right.bounds = R.bounds;
#endif

    // Duplicate or unsplit references intersecting both sides.

    while (leftEnd < rightStart)
    {
        // Split reference.

        Reference lref, rref;
        splitReference(lref, rref, refs[leftEnd], split.dim, split.pos);

        // Compute SAH for duplicate/unsplit candidates.

        // Unsplit is the bounds if we don't split this reference
        // Duplicate is the bounds if we do split this reference
        AABB lub = left.bounds;  // Unsplit to left:     new left-hand bounds.
        AABB rub = right.bounds; // Unsplit to right:    new right-hand bounds.
        AABB ldb = left.bounds;  // Duplicate:           new left-hand bounds.
        AABB rdb = right.bounds; // Duplicate:           new right-hand bounds.
        lub.grow(refs[leftEnd].bounds);
        rub.grow(refs[leftEnd].bounds);
        ldb.grow(lref.bounds);
        rdb.grow(rref.bounds);

        F32 lac = m_platform.getTriangleCost(leftEnd - leftStart);
        F32 rac = m_platform.getTriangleCost(refs.getSize() - rightStart);
        F32 lbc = m_platform.getTriangleCost(leftEnd - leftStart + 1);
        F32 rbc = m_platform.getTriangleCost(refs.getSize() - rightStart + 1);

        F32 unsplitLeftSAH = lub.area() * lbc + right.bounds.area() * rac;
        F32 unsplitRightSAH = left.bounds.area() * lac + rub.area() * rbc;
        F32 duplicateSAH = ldb.area() * lbc + rdb.area() * rbc;
        F32 minSAH = min(unsplitLeftSAH, unsplitRightSAH, duplicateSAH);

        // Unsplit to left?

        if (minSAH == unsplitLeftSAH)
        {
            left.bounds = lub;
            leftEnd++;
        }

        // Unsplit to right?

        else if (minSAH == unsplitRightSAH)
        {
            right.bounds = rub;
            swap(refs[leftEnd], refs[--rightStart]);
        }

        // Duplicate?

        else
        {
            left.bounds = ldb;
            right.bounds = rdb;
            refs[leftEnd++] = lref;
            refs.add(rref);
        }
    }

    left.numRef = leftEnd - leftStart;
    right.numRef = refs.getSize() - rightStart;
}

//------------------------------------------------------------------------

void FW::GPUSplitBVHBuilder::splitReference(Reference& left, Reference& right, const Reference& ref, int dim, F32 pos)
{
    // Initialize references.

    left.triIdx = right.triIdx = ref.triIdx;
    left.bounds = right.bounds = AABB();

    // Loop over vertices/edges.

    const Vec3i* tris = (const Vec3i*)m_bvh.getScene()->getTriVtxIndexBuffer().getPtr();
    const Vec3f* verts = (const Vec3f*)m_bvh.getScene()->getVtxPosBuffer().getPtr();
    const Vec3i& inds = tris[ref.triIdx];
    const Vec3f* v1 = &verts[inds.z];

    for (int i = 0; i < 3; i++)
    {
        const Vec3f* v0 = v1;
        v1 = &verts[inds[i]];
        F32 v0p = v0->get(dim);
        F32 v1p = v1->get(dim);

        // Insert vertex to the boxes it belongs to.

        if (v0p <= pos)
            left.bounds.grow(*v0);
        if (v0p >= pos)
            right.bounds.grow(*v0);

        // Edge intersects the plane => insert intersection to both boxes.

        if ((v0p < pos && v1p > pos) || (v0p > pos && v1p < pos))
        {
            Vec3f t = lerp(*v0, *v1, clamp((pos - v0p) / (v1p - v0p), 0.0f, 1.0f));
            left.bounds.grow(t);
            right.bounds.grow(t);
        }
    }

    // Intersect with original bounds.

    left.bounds.max()[dim] = pos;
    right.bounds.min()[dim] = pos;
    left.bounds.intersect(ref.bounds);
    right.bounds.intersect(ref.bounds);
}

//------------------------------------------------------------------------
