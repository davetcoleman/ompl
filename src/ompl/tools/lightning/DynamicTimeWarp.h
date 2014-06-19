/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK, The University of Tokyo.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK, The University of Tokyo nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Compute Dynamic Time Warping
*/

#ifndef OMPL_TOOLS_LIGHTNING_DYNAMIC_TIME_WARP_
#define OMPL_TOOLS_LIGHTNING_DYNAMIC_TIME_WARP_

#include <ompl/geometric/PathGeometric.h>
#include <ompl/baes/SpaceInformation.h>

namespace ompl
{
namespace tools
{

/// @cond IGNORE
OMPL_CLASS_FORWARD(DynamicTimeWarp);
/// @endcond

/** \class ompl::geometric::DynamicTimeWarpPtr
    \brief A boost shared pointer wrapper for ompl::tools::DynamicTimeWarp */

class DynamicTimeWarp
{
public:
    DynamicTimeWarp(base::SpaceInformationPtr &si)
        : si_(si)
    {
        infinity_ = std::numeric_limits<double>::infinity();
    }

    /**
     * \brief Use Dynamic Timewarping to score two paths
     * \param path1
     * \param path2
     * \return score
     */
    double calcDTWDistance(const og::PathGeometric &path1, const og::PathGeometric &path2 )
    {
        // Get lengths
        std::size_t n = path1.getStateCount();
        std::size_t m = path2.getStateCount();

        // Intialize table to have all values of infinity
        std::vector<std::vector<double> > table(n,std::vector<double>(m,infinity_)); // TODO reuse this memory by allocating it in the constructor!

        // Set first value to zero
        table[0][0] = 0;

        // Do calculations
        double cost;
        for (std::size_t i = 1; i < n; ++i)
        {
            for (std::size_t j = 1; j < m; ++j)
            {
                cost = si_->distance(path1.getState(i), path2.getState(j));
                table[i][j] = cost + min(table[i-1][j], table[i][j-1], table[i-1][j-1]);
            }
        }

        return table[n-1][m-1];
    }

    /**
     * \brief Calculate min for 3 numbers
     */
    double min(double n1, double n2, double n3)
    {
        return std::min(n1, std::min(n2, n3));
    }

    /**
     * \brief If path1 and path2 have a better start/goal match when reverse, then reverse path2
     * \param path to test against
     * \param path to reverse
     * \return true if reverse was necessary
     */
    bool reversePathIfNecessary(og::PathGeometric &path1, og::PathGeometric &path2)
    {
        // Reverse path2 if it matches better
        const ob::State* s1 = path1.getState(0);
        const ob::State* s2 = path2.getState(0);
        const ob::State* g1 = path1.getState(path1.getStateCount()-1);
        const ob::State* g2 = path2.getState(path2.getStateCount()-1);

        double regularDistance  = si_->distance(s1,s2) + si_->distance(g1,g2);
        double reversedDistance = si_->distance(s1,g2) + si_->distance(s2,g1);

        // Check if path is reversed from normal [start->goal] direction
        if ( regularDistance > reversedDistance )
        {
            // needs to be reversed
            path2.reverse();
            return true;
        }

        return false;
    }

    double getPathsScoreConst(const og::PathGeometric &path1, const og::PathGeometric &path2)
    {
        // Copy the path but not the states
        og::PathGeometric newPath1 = path1;
        og::PathGeometric newPath2 = path2;
        getPathsScoreNonConst(newPath1, newPath2);
    }

    /**
     * \brief Use dynamic time warping to compare the similarity of two paths
     *        Note: this will interpolate the second path and it returns the change by reference
     * \param path1 - const, will not interpolate
     * \param path2 - will interpolate
     * \return score
     */
    double getPathsScoreHalfConst(const og::PathGeometric &path1, og::PathGeometric &path2)
    {
        // Copy the path but not the states
        og::PathGeometric newPath = path1;
        getPathsScoreNonConst(newPath, path2);
    }

    /**
     * \brief Use dynamic time warping to compare the similarity of two paths
     *        Note: this will interpolate both of the paths and it returns the change by reference
     * \param path1 - will interpolate
     * \param path2 - will interpolate
     * \return score
     */
    double getPathsScoreNonConst(og::PathGeometric &path1, og::PathGeometric &path2)
    {
        // Reverse path2 if it matches better
        reversePathIfNecessary(path1, path2);

        //std::size_t path1Count = path1.getStateCount();
        //std::size_t path2Count = path2.getStateCount();

        // Interpolate both paths so that we have an even discretization of samples
        path1.interpolate();
        path2.interpolate();

        // debug
        //OMPL_INFORM("Path 1 interpolated with an increase of %ld states", path1.getStateCount() - path1Count);
        //OMPL_INFORM("Path 2 interpolated with an increase of %ld states", path2.getStateCount() - path2Count);

        // compute the DTW between two vectors and divide by total path length of the longer path
        double score = calcDTWDistance(path1, path2) / std::max(path1.getStateCount(),path2.getStateCount());

        return score;
    }

protected:

    double infinity_;

    /** \brief The created space information */
    base::SpaceInformationPtr     si_;

}; // end of class

} // namespace
} // namespace

#endif
