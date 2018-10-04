
#pragma once

#include <bvh/BVH.hpp>

// Pass in a BVH and do this:
// Make treelets of branching factor N
// Compress each treelet
// Compute compression statistics
// Decompress each treelet
// Ray trace it

class SGPUBVHCompressor
{

public:
	SGPUBVHCompressor(BVH& bvh, const BVH::BuildParams& params);
	~SGPUBVHCompressor(void);

	BVHNode*                run(void);

private:
	SGPUBVHCompressor(const SGPUBVHCompressor&); // forbidden
	SGPUBVHCompressor&        operator=           (const SGPUBVHCompressor&); // forbidden

private:
	BVH&                    m_bvh;
	const Platform&         m_platform;
	const BVH::BuildParams& m_params;

	Timer                   m_progressTimer;

};
