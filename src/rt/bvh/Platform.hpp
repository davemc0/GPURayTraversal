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

#pragma once
#include "base/String.hpp"
#include "base/Hash.hpp"

namespace FW
{

class LeafNode;
class BVHNode;

class Platform
{
public:
#if !FW_CUDA_DEVICE
    Platform() { m_name = String("Default"); m_SAHNodeCost = 1.f; m_SAHTriangleCost = 1.f; m_nodeBatchSize = 1; m_triBatchSize = 1; m_minLeafSize = 1; m_maxLeafSize = 0x7FFFFFF; }
    Platform(const String& name, float nodeCost = 1.f, float triCost = 1.f, S32 nodeBatchSize = 1, S32 triBatchSize = 1)
               { m_name = name; m_SAHNodeCost = nodeCost; m_SAHTriangleCost = triCost; m_nodeBatchSize = nodeBatchSize; m_triBatchSize = triBatchSize; m_minLeafSize = 1; m_maxLeafSize = 0x7FFFFFF; }

    const String&   getName() const                     { return m_name; }
#endif

    // SAH weights
    float getSAHTriangleCost() const                    { return m_SAHTriangleCost; }
    float getSAHNodeCost() const                        { return m_SAHNodeCost; }

    // SAH costs, raw and batched
    float getCost(int numChildNodes,int numTris) const  { return getNodeCost(numChildNodes) + getTriangleCost(numTris); }
    float getTriangleCost(S32 n) const                  { return roundToTriangleBatchSize(n) * m_SAHTriangleCost; }
    float getNodeCost(S32 n) const                      { return roundToNodeBatchSize(n) * m_SAHNodeCost; }

    // batch processing (how many ops at the price of one)
    S32   getTriangleBatchSize() const                  { return m_triBatchSize; }
    S32   getNodeBatchSize() const                      { return m_nodeBatchSize; }
    void  setTriangleBatchSize(S32 triBatchSize)        { m_triBatchSize = triBatchSize; }
    void  setNodeBatchSize(S32 nodeBatchSize)           { m_nodeBatchSize= nodeBatchSize; }
    S32   roundToTriangleBatchSize(S32 n) const         { return ((n+m_triBatchSize-1)/m_triBatchSize)*m_triBatchSize; }
    S32   roundToNodeBatchSize(S32 n) const             { return ((n+m_nodeBatchSize-1)/m_nodeBatchSize)*m_nodeBatchSize; }

    // leaf preferences
    void  setLeafPreferences(S32 minSize,S32 maxSize)   { m_minLeafSize=minSize; m_maxLeafSize=maxSize; }
    S32   getMinLeafSize() const                        { return m_minLeafSize; }
    S32   getMaxLeafSize() const                        { return m_maxLeafSize; }

#if !FW_CUDA_DEVICE
    U32   computeHash() const                           { return hashBits(hash<String>(m_name), floatToBits(m_SAHNodeCost), floatToBits(m_SAHTriangleCost), hashBits(m_triBatchSize, m_nodeBatchSize, m_minLeafSize, m_maxLeafSize)); }
#endif

private:
    float   m_SAHNodeCost;
    float   m_SAHTriangleCost;
    S32     m_triBatchSize;
    S32     m_nodeBatchSize;
    S32     m_minLeafSize;
    S32     m_maxLeafSize;
#if !FW_CUDA_DEVICE
    String  m_name;
#endif
};


} //