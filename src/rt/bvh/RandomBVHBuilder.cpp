
#include "bvh/RandomBVHBuilder.hpp"
#include "bvh/BVHNode.hpp"

using namespace FW;

//------------------------------------------------------------------------

RandomBVHBuilder::RandomBVHBuilder(BVH& bvh, const BVH::BuildParams& params, bool randomize)
:   m_bvh           (bvh),
    m_platform      (bvh.getPlatform()),
    m_params        (params),
    m_randomizeLeaves(randomize)
{
}

//------------------------------------------------------------------------

RandomBVHBuilder::~RandomBVHBuilder(void)
{
}

BVHNode* RandomBVHBuilder::run(void)
{
    printf("Random BVH randomize=%d\n", m_randomizeLeaves);

    m_progressTimer.start();

    const Vec3i* tris = (const Vec3i*)m_bvh.getScene()->getTriVtxIndexBuffer().getPtr();
    const Vec3f* verts = (const Vec3f*)m_bvh.getScene()->getVtxPosBuffer().getPtr();

    U32 numRef = m_bvh.getScene()->getNumTriangles();
    m_refStack.resize(numRef);
    Array<S32>& triIndices = m_bvh.getTriIndices();
    triIndices.resize(numRef);

    // Fill in the triIndices in order
    U32 k = 0;
    for (U32 i = 0; i < numRef; i++)  {
        triIndices[k] = i;
        m_refStack[i].bounds = AABB();
        for (int j = 0; j < 3; j++)
            m_refStack[i].bounds.grow(verts[tris[i][j]]);
        // Remove degenerates from triIndices but keep them in m_refStack
        Vec3f size = m_refStack[i].bounds.max() - m_refStack[i].bounds.min();
        if (min(size) < 0.0f || sum(size) == max(size))
            k--;
        k++;
    }

    triIndices.resize(k);

    // Randomize triIndices order
    if(m_randomizeLeaves)
        randomizeArray(triIndices);

    // Build nodes over randomly ordered tri indices
    BVHNode* root = buildNode(0, triIndices.getSize());
    triIndices.compact();

    if (m_params.enablePrints)
        printf("RandomBVHBuilder: progress %.0f%%\n", 100.0f);

    return root;
}

void RandomBVHBuilder::randomizeArray(Array<S32>& ar)
{
    for (int i = 0; i < ar.getSize(); i++) {
        int t = rand() % ar.getSize();
        swap(ar[i], ar[t]);
    }
}

BVHNode* RandomBVHBuilder::buildNode(int left, int right)
{
    Array<S32>& triIndices = m_bvh.getTriIndices();
    FW_ASSERT(left < triIndices.getSize());
    FW_ASSERT(right <= triIndices.getSize());

    if (left + 1 == right) {
        m_params.stats->forcedLeaves++;
        U32 i = triIndices[left];
        return new LeafNode(m_refStack[i].bounds, left, right);
    }

    int mid = (right + left + 1) / 2;
    FW_ASSERT(mid > left);
    FW_ASSERT(right > mid);
    BVHNode* rightNode = buildNode(mid, right);
    BVHNode* leftNode = buildNode(left, mid);
    AABB bounds = leftNode->m_bounds + rightNode->m_bounds;
    return new InnerNode(bounds, leftNode, rightNode);
}
