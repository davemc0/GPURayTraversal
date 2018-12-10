doGeneration()
{
    // Remove degenerates
    // OPT: For Sweep builder move this out of the loop. If so, for speed, change it to not be a stable_partition. Split builder makes new refs and some are degenerate if alpha == 0.
    // OPT: Could make this part of sort predicate and use custom iterator to count how many are rejected or an atomic
    N <= thrust::stable_partition();

    // Compute refSegIdx
    refSegIdx <= thrust::transform_inclusive_scan();
    
    // Compute segRefIdx
    segRefIdx <= 

    // Compute each segment's leaf SAH
    segCostBest, segStrat <=
    
    // Try object split in each dimension
    for (int dim = 0; dim < 3; dim++) {
        // Sort in given dimension
        thrust::sort(TGBK);

        // Sweep right to left and determine bounds
        // OPT: Use transform_output iterator to just store area instead of whole AABB; how to get AABB for later?
        refRightBounds <= thrust::inclusive_scan_by_key(refBounds);

        // Sweep left to right and determine bounds
        refLeftBounds <= thrust::exclusive_scan_by_key(refBounds);

        // Select lowest SAH
        // OPT: Only need to write the keys out once. Could use discard_iterator on the other two dimensions. refSegIdx is unneeded; should use a discard_iterator to get rid of refRightIdx output, but was getting errors. Store a segment's full AABB into its final BVHNode, since we know its location now
        segCostNew, segIdxNew <= thrust::reduce_by_key(refKeys, refRightBounds, refRightIdx, refLeftBounds, refLeftIdx);

        // OPT: Would rather do this as a conditional_iterator as part of reduce_by_key.
        segCostBest, segIdxBest, segStrat <= thrust::for_each_n(segCostBest, segIdxBest, segCostNew, segIdxNew);
    }

    // Try spatial split in each dimension
    for (int dim = 0; dim < 3; dim++) {
        segCostBest, segIdxBest <= 
        // XXX Will spatial splits screw up gamma by inserting nodes between index and what it points to?
    }
    
    // Count how many refs want each kind of strategy to give me indices to them after they're sorted
    // thrust::inclusive_scan with an output tuple with a value per strategy. Could fold it into the for_each_n and use atomic counters?
    
    // XXX Need to make sure that for multi-reference leaves the right ref sorts to the ends to make gamma work.

    // OPT: Sort by strat to give good spans for doing separate algorithms in next phase; maybe lets us keep a sorted array per dim; Could store strat in 3 msbs of segKeys instead of segRefIdx
    thrust::sort();

    // Update Nactive here so only the active ones get their keys updated
    // Try to get rid of keys and just use segIdx. Have to be able to put them back in order to make gamma work.

    // Update keys to partition each segment at the best location
    refKeys <= thrust::for_each(segIdxBest, segStrat);
}

makeNodes()
{
    // Fill leaf node i
    thrust::for_each();

    // Fill inner node i
    thrust::for_each();
}

batchRun()
{
    // Determine triangle and root bounds
    rootBounds = thrust::transform_reduce();

    for (S32 level = 0; level < 64; level++) {
        doGeneration(N, nSegments, level); // Modifies N
    }

    BVHNode* root = makeNodes(N);
}




        // OPT: Store a segment's full AABB into its final BVHNode, since we know its location now

        // Select lowest SAH.
        //BBIZipIt BoundsIt(thrust::make_tuple(refRightBounds, refLeftBounds, thrust::counting_iterator<S32>((S32)0)));

        // OPT: Only need to write the keys out once. Could use discard_iterator on the other two dimensions.
        // OPT: refSegIdx is unneeded; should use a discard_iterator to get rid of refRightIdx output, but was getting errors.
        auto segValues = thrust::make_zip_iterator(thrust::make_tuple(dim == 0 ? segCostBest : segCostNew, dim == 0 ? segIdxBest : segIdxNew, discard)); // FII

        auto segEnd = thrust::reduce_by_key(thrust::device,
            refKeys, refKeys + N, thrust::make_transform_iterator(BoundsIt, BoundsToCost()),
            segKeys, segValues, // OPT: I don't use segKeys. Should use discard_iterator, but need nSegments.
            [] BHD(U64 ka, U64 kb) { return ka == kb; },
            [] BHD(FIITuple a, FIITuple b) { return get<0>(a) < get<0>(b) ? a :
                (get<0>(a) > get<0>(b) ? b :
                (abs(get<1>(a) - get<2>(a)) < abs(get<1>(b) - get<2>(b)) ? a : b)); });

        nSegments = segEnd.first - segKeys;
        U64 demoK = segKeys[0];
        S32 thisStrategy = stratObjectSplit | (dim << stratBitOffset);
