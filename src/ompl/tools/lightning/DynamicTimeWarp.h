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
#include <ompl/base/SpaceInformation.h>

namespace og = ompl::geometric;
namespace ob = ompl::base;
//namespace ot = ompl::tools;

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
    DynamicTimeWarp(base::SpaceInformationPtr &si);

    /**
     * \brief Use Dynamic Timewarping to score two paths
     * \param path1
     * \param path2
     * \return score
     */
    double calcDTWDistance(const og::PathGeometric &path1, const og::PathGeometric &path2 );

    /**
     * \brief Calculate min for 3 numbers
     */
    double min(double n1, double n2, double n3);

    /**
     * \brief If path1 and path2 have a better start/goal match when reverse, then reverse path2
     * \param path to test against
     * \param path to reverse
     * \return true if reverse was necessary
     */
    bool reversePathIfNecessary(og::PathGeometric &path1, og::PathGeometric &path2);

    double getPathsScoreConst(const og::PathGeometric &path1, const og::PathGeometric &path2);

    /**
     * \brief Use dynamic time warping to compare the similarity of two paths
     *        Note: this will interpolate the second path and it returns the change by reference
     * \param path1 - const, will not interpolate
     * \param path2 - will interpolate
     * \return score
     */
    double getPathsScoreHalfConst(const og::PathGeometric &path1, og::PathGeometric &path2);

    /**
     * \brief Use dynamic time warping to compare the similarity of two paths
     *        Note: this will interpolate both of the paths and it returns the change by reference
     * \param path1 - will interpolate
     * \param path2 - will interpolate
     * \return score
     */
    double getPathsScoreNonConst(og::PathGeometric &path1, og::PathGeometric &path2);

protected:

    double infinity_;

    /** \brief The created space information */
    base::SpaceInformationPtr     si_;

}; // end of class

} // namespace
} // namespace

#endif
