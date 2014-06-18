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

#include <ompl/datastructures/NearestNeighbors.h> // distance funciton

namespace ompl
{
namespace tools
{
//template<typename _T>
class DynamicTimeWarp
{
public:
    DynamicTimeWarp() //const ompl::NearestNeighbors<_T>::DistanceFunction &distFun)
    {
    }

    /**
     * \brief Use Dynamic Timewarping to score two paths
     * \param path1
     * \param path2
     * \return score
     */
    double calcDTWDistance(ob::PlannerDataPtr &path1, ob::PlannerDataPtr &path2 )
    {
        double INF = 100000000; // TODO

        // Get lengths
        std::size_t n = path1->numVertices();
        std::size_t m = path2->numVertices();

        // Intialize table to have all values of infinity
        std::vector<std::vector<double> > table(n,std::vector<double>(m,INF)); // TODO reuse this memory
        //table = [[0 for i in xrange(m+1)] for j in xrange(n+1)];
        //for i in xrange(1, n+1):
        //table[i][0] = float('inf');
        //for i in xrange(1, m+1):
        //table[0][i] = float('inf');

        // Set first value to zero
        table[0][0] = 0;

        // Do calculations
        double cost;
        for (std::size_t i = 0; i < n; ++i)
        {
            for (std::size_t j = 0; j < m; ++j)
            {
                //                cost = distFun_(path1.getState(i), path2.getState(j));
                //                table[i+1][j+1] = cost + std::min(table[i][j+1], table[i+1][j], table[i][j]);                
            }
        }

        return table[n][m];
    }

protected:

    /** \brief The used distance function */
    //ompl::NearestNeighbors::DistanceFunction distFun_;

}; // end of class

} // namespace
} // namespace

#endif
