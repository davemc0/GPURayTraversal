// A representation of a compressed wide BVH node for SGPU

#pragma once

namespace FW
{

    const int MAX_LEAVES = 32;
    const int MAX_INTR_LVL = MAX_LEAVES / 2 / 2; // How many internal node pairs on the widest layer of a compressed treelet
    const int MAX_LEAF_PAIRS = MAX_LEAVES / 2;

    // The parameters that represent a floating point number format
    struct FloatRep
    {
        int signbits;
        int expobits;
        int mantbits;
        int excess;
        bool zeroone;
    };

    // All the parameters of the compressed representation
    struct CmpNodeParams
    {
        int numLeaves;
        FloatRep parentRep, childRep, leafRep;
    };

    struct CmpNodeDim
    {
        // Root
        int rootMinSign, rootMinExpo, rootMinMant;
        int rootMaxSign, rootMaxExpo, rootMaxMant;

        // Internal Nodes
        struct InternalNodePair
        {
            int lftExpo, lftMant, rgtExpo, rgtMant, ctrl;
        };

        InternalNodePair intr[MAX_INTR_LVL][MAX_LEAVES];

        // Leaves
        struct LeafNodePair
        {
            int lftExpo, lftMant, rgtExpo, rgtMant, ctrl;
        };

        LeafNodePair leaves[MAX_LEAF_PAIRS];
    };

    // A complete compressed node that has numLeaves leaves and one root node

    struct CmpNode
    {
        CmpNodeDim dimX, dimY, dimZ;

        int indicesAndOffsetsGoHere;
    };

};
