/* 
 * Copyright (c) 2010, Dejan Pangercic <dejan.pangercic@cs.tum.edu>,
 Vladimir Haltakov <vladimir.haltakov@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef ODU_FINDER_COMMON_H_
#define ODU_FINDER_COMMON_H_
#include <ANN/ANN.h>
#include <vocabulary_tree/simple_kmeans.h>
#include <vocabulary_tree/vocabulary_tree.h>
#include <vocabulary_tree/database.h>
#include <vocabulary_tree/tree_builder.h>
#include <vocabulary_tree/simple_kmeans.h>

#include <siftfast/siftfast.h>

/*
 * Define standard C methods and C++ classes that are common to all methods
 */
namespace odu_finder
{

inline double logsig(double x) 
{
    return 1.0 / (1.0 + exp(-x));
}

/**
 * \brief cluster keypoints in 2D image according to the logsig criterion
 * using KdTree
 * \param points 2D coordinate of every point
 * \param points_count number of keypoints
 * \param membership membership vector
 * \param r_max max radius for adaptive radius calculation
 * \param r_min min radius for adaptive radius calculation
 * \param A
 * \param K
 */
inline int cluster_points(ANNpointArray points, int points_count,
                          std::vector<int>& membership, float r_max =
        600.0f, float r_min = 200.0f, float A = 800.0f, float K = 0.02f)
{
    // adaptive radius calculation
    // (see paper Fast and Robust Object Detection in Household Environments
    // Using Vocabulary Trees with SIFT Descriptors by Pangercic and Haltakov)
    double radius = (1 - logsig((points_count - A) * K)) * (r_max - r_min) + r_min;
    //radius = (1-logsig((points_count-800.0)/50.0))*300 + 300;
    //printf("\n\n NEW RADIUS = %f\n\n", radius);
    ANNidxArray nnIdx = new ANNidx[points_count];
    //	ANNdistArray dists = new ANNdist[points_count];
    ANNkd_tree* kdTree = new ANNkd_tree(points, points_count, 2);
    membership.assign(points_count, -1);
    int last_unassigned_id = 0;
    int current_cluster = 0;

    while (last_unassigned_id < points_count)
    {
        std::vector<int> points_stack;
        points_stack.push_back(last_unassigned_id);
        while (points_stack.size() > 0)
        {
            //printf("STACK SIZE = %d\n", points_stack.size());
            int current_point_id = points_stack.back();
            points_stack.pop_back();
            membership[current_point_id] = current_cluster;
            int points_found = kdTree->annkFRSearch(points[current_point_id],
                                                    radius, points_count, nnIdx);
            //printf("Neighbours found = %d\n", points_found);
            int newPointsCount = 0;
            for (int i = 0; i < points_found; ++i)
                if (membership[nnIdx[i]] == -1)
                    ++newPointsCount;

            if (newPointsCount > 3)
            {
                for (int i = 0; i < points_found; ++i)
                    if (membership[nnIdx[i]] == -1)
                        points_stack.push_back(nnIdx[i]);
            }
        }

        ++current_cluster;
        ++last_unassigned_id;
        while (last_unassigned_id < points_count
               && membership[last_unassigned_id] != -1)
            ++last_unassigned_id;
    }

    delete[] nnIdx;
    //delete [] dists;
    delete kdTree;
    annClose();
    return current_cluster;
}

/**
 * \brief Keypoints extension class
 */
class KeypointExt 
{
public:
    Keypoint keypoint;
    vt::Word word;
    unsigned int cluster;

    KeypointExt(Keypoint keypoint, vt::Word word, unsigned int cluster = 0):
        keypoint(keypoint),
        word(word),
        cluster(cluster)
    {}
};

inline bool compare_keypoint_ext(KeypointExt* k1, KeypointExt* k2) 
{
    return (k1->word < k2->word);
}

inline bool compare_pairs(std::pair<uint32_t, float> p1, std::pair<uint32_t, float> p2) 
{
    return (p1.second > p2.second);
}

inline bool compare_pairs2(std::pair<std::string, int> p1, std::pair<std::string, int> p2) 
{
    return (p1.second > p2.second);
}

}

#endif  //#ifndef ODU_FINDER_COMMON_H_
