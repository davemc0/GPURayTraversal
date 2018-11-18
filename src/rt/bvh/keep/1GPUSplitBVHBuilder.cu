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

#include "bvh/GPUSplitBVHBuilder.hpp"
#include "base/Sort.hpp"

namespace {
    float myrandf()
    {
        int randi = (rand() | (rand() << 15)) & 0x3fffffff;
        float randf = randi / float(0x3fffffff);

        return randf;
    }
};

#if 0
//------------------------------------------------------------------------
FW::GPUSplitBVHBuilder::GPUSplitBVHBuilder(BVH& bvh, const BVH::BuildParams& params)
    : m_bvh(bvh),
    m_platform(bvh.getPlatform()),
    m_params(params),
    m_minOverlap(0.0f),
    m_sortDim(-1)
{
}
#endif
#if 1

//------------------------------------------------------------------------

FW::GPUSplitBVHBuilder::~GPUSplitBVHBuilder(void)
{
}
#endif
#if 0

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

    for (int i = 0; i < rootSpec.numRef; i++)
    {
        m_refStack[i].triIdx = i;
        for (int j = 0; j < 3; j++)
            m_refStack[i].bounds.grow(verts[tris[i][j]]);
        rootSpec.bounds.grow(m_refStack[i].bounds);
    }

    Benchy();

    // Initialize rest of the members.

    m_minOverlap = rootSpec.bounds.area() * m_params.splitAlpha;
    m_rightBounds.reset(max(rootSpec.numRef, (int)NumSpatialBins) - 1);
    m_numDuplicates = 0;
    m_progressTimer.start();

    // Build recursively.

    BVHNode* root = buildNode(rootSpec, 0, 0.0f, 1.0f);
    m_bvh.getTriIndices().compact();

    // Done.

    if (m_params.enablePrints)
        printf("GPUSplitBVHBuilder: progress %.0f%%, duplicates %.0f%%\n",
            100.0f, (F32)m_numDuplicates / (F32)m_bvh.getScene()->getNumTriangles() * 100.0f);
    return root;
}
#endif
#if 1

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
#endif
#if 0

//------------------------------------------------------------------------

void FW::GPUSplitBVHBuilder::sortSwap(void* data, int idxA, int idxB)
{
    GPUSplitBVHBuilder* ptr = (GPUSplitBVHBuilder*)data;
    swap(ptr->m_refStack[idxA], ptr->m_refStack[idxB]);
}
#endif
#if 1

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

    if (spec.numRef <= m_platform.getMinLeafSize() || level >= MaxDepth)
        return createLeaf(spec);

    // Find split candidates.

    F32 area = spec.bounds.area();
    F32 leafSAH = area * m_platform.getTriangleCost(spec.numRef);
    F32 nodeSAH = area * m_platform.getNodeCost(2);
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
#endif
#if 0

//------------------------------------------------------------------------

FW::BVHNode* FW::GPUSplitBVHBuilder::createLeaf(const NodeSpec& spec)
{
    Array<S32>& tris = m_bvh.getTriIndices();
    for (int i = 0; i < spec.numRef; i++)
        tris.add(m_refStack.removeLast().triIdx);
    return new LeafNode(spec.bounds, tris.getSize() - spec.numRef, tris.getSize());
}
#endif
#if 1

//------------------------------------------------------------------------

FW::GPUSplitBVHBuilder::ObjectSplit FW::GPUSplitBVHBuilder::findObjectSplit(const NodeSpec& spec, F32 nodeSAH)
{
    ObjectSplit split;
    const Reference* refPtr = m_refStack.getPtr(m_refStack.getSize() - spec.numRef);
    F32 bestTieBreak = FW_F32_MAX;

    // Sort along each dimension.

    for (m_sortDim = 0; m_sortDim < 3; m_sortDim++)
    {
        sort(this, m_refStack.getSize() - spec.numRef, m_refStack.getSize(), sortCompare, sortSwap);

        // Sweep right to left and determine bounds.

        AABB rightBounds;
        for (int i = spec.numRef - 1; i > 0; i--)
        {
            rightBounds.grow(refPtr[i].bounds);
            m_rightBounds[i - 1] = rightBounds;
        }

        // Sweep left to right and select lowest SAH.

        AABB leftBounds;
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
    }
    return split;
}
#endif
#if 0

//------------------------------------------------------------------------

void FW::GPUSplitBVHBuilder::performObjectSplit(NodeSpec& left, NodeSpec& right, const NodeSpec& spec, const ObjectSplit& split)
{
    m_sortDim = split.sortDim;
    sort(this, m_refStack.getSize() - spec.numRef, m_refStack.getSize(), sortCompare, sortSwap);

    left.numRef = split.numLeft;
    left.bounds = split.leftBounds;
    right.numRef = spec.numRef - split.numLeft;
    right.bounds = split.rightBounds;
}
#endif
#if 1

//------------------------------------------------------------------------

FW::GPUSplitBVHBuilder::SpatialSplit FW::GPUSplitBVHBuilder::findSpatialSplit(const NodeSpec& spec, F32 nodeSAH)
{
    // Initialize bins.

    if (spec.numRef > 100000) {
        printf("findSpatialSplit(%d)", spec.numRef);
        m_progressTimer.start();
    }

    Vec3f origin = spec.bounds.min();
    Vec3f binSize = (spec.bounds.max() - origin) * (1.0f / (F32)NumSpatialBins);
    Vec3f invBinSize = 1.0f / binSize;

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

    // Chop references into bins.

    for (int refIdx = m_refStack.getSize() - spec.numRef; refIdx < m_refStack.getSize(); refIdx++)
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

    if (spec.numRef > 100000) {
        printf(" t=%f\n", m_progressTimer.end());
    }

    return split;
}
#endif
#if 0

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

    // Duplicate or unsplit references intersecting both sides.

    while (leftEnd < rightStart)
    {
        // Split reference.

        Reference lref, rref;
        splitReference(lref, rref, refs[leftEnd], split.dim, split.pos);

        // Compute SAH for duplicate/unsplit candidates.

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
#endif
#if 1

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
#endif
#if 0

//------------------------------------------------------------------------

float BenchyMcBenchfaceG(size_t N)
{
    FW::Timer tim;
    tim.start();

    unsigned long long cnt = 0;

    for (unsigned long long i = 0; i < N; i++) {
        float x = myrandf();
        float y = myrandf();
        float r = sqrtf(x * x + y * y);

        if (r < 1.0f)
            cnt++;
    }

    float pi = 4.0f * (float)cnt / (float)N;

    printf("G pi=%f t=%f\n", pi, tim.end());

    return pi;
}

float BenchyBench2(size_t N)
{
    FW::Timer tim;
    tim.start();

    for (int i = 0; i < N; i++) {
        void* ptr;
        // Allocates a 64KB page on every call (no sub-allocator).
        // 4.14 sec to do 64k allocs of 16KB each (but it provides them using 64KB pages)
        // Sometimes uses 96KB, 128KB, 192KB
        // Requesting 64KB allocs provides them on 2MB pages.
        cudaMallocManaged(&ptr, (1<<10), cudaMemAttachGlobal);
        //if ((i % 10000) == 0)
            printf("i=%d t=%f 0x%016llx\n", i, tim.getElapsed(), (unsigned long long)ptr);
    }

    printf("G t=%f\n", tim.end());

    return 0.f;
}
#endif
#if 0

void FW::GPUSplitBVHBuilder::Benchy()
{
    FW::Timer tim;
    tim.start();

    const size_t N = 1000001;
    AABB final;

    for (unsigned long long i = 0; i < N; i++) {
        Reference rr = m_refStack[rand() % m_refStack.getSize()];

        for (int s = 0; s < 10; s++) {
            int dim = rand() % 3;
            float split = rr.bounds.min()[dim] + myrandf() * (rr.bounds.max() - rr.bounds.min())[dim];

            Reference leftRef, rightRef;
            splitReference(leftRef, rightRef, rr, dim, split);
            rr = leftRef.bounds.area() > rightRef.bounds.area() ? leftRef : rightRef;
            final.grow(rr.bounds);
        }
        if ((i % 1000000) == 0)
            printf("G i=%lld s=%f t=%f\n", i, final.area(), tim.getElapsed());
    }

    exit(0);
}
#endif
