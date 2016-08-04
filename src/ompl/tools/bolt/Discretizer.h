/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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

/* Author: Dave Coleman <dave@dav.ee>
   Desc:   Utility to discretize a space into a uniform grid
*/

#ifndef OMPL_TOOLS_BOLT_DISCRETIZER_
#define OMPL_TOOLS_BOLT_DISCRETIZER_

// OMPL
#include <ompl/base/StateSpace.h>
#include <ompl/tools/bolt/Visualizer.h>
#include <ompl/tools/bolt/BoltGraph.h>
#include <ompl/tools/bolt/DenseDB.h>

// Boost
#include <boost/thread/mutex.hpp>

namespace ompl
{
namespace tools
{
namespace bolt
{
/// @cond IGNORE
OMPL_CLASS_FORWARD(Discretizer);
/// @endcond

/** \class ompl::tools::bolt::DiscretizerPtr
    \brief A boost shared pointer wrapper for ompl::tools::bolt::Discretizer */

class Discretizer
{
  public:
    /** \brief Constructor needs the state space used for planning.
     *  \param space - state space
     */
    Discretizer(base::SpaceInformationPtr si, DenseDBPtr denseDB, base::VisualizerPtr visual);

    /** \brief Deconstructor */
    virtual ~Discretizer(void);

    /**
     * \brief Discretize the space into a simple grid
     */
    void generateGrid();

    /** \brief Helper function to calculate connectivity based on dimensionality */
    static std::size_t getEdgesPerVertex(base::SpaceInformationPtr si);

  private:
    void generateVertices();

    void createVertexThread(std::size_t threadID, double startJointValue, double endJointValue,
                            base::SpaceInformationPtr si);

    void recursiveDiscretization(std::size_t threadID, std::vector<double>& values, std::size_t jointID,
                                 base::SpaceInformationPtr si, base::State* candidateState,
                                 std::size_t maxDiscretizationLevel);

    /**
     * \brief Connect vertices wherever possible
     */
    void generateEdges();

    /** \brief Collision check edges using threading */
    void checkEdgesThreaded(const std::vector<DenseEdge>& unvalidatedEdges);

    /** \brief Collision check edges in vector from [startEdge, endEdge] inclusive */
    void checkEdgesThread(std::size_t threadID, std::size_t startEdge, std::size_t endEdge,
                          base::SpaceInformationPtr si, const std::vector<DenseEdge>& unvalidatedEdges);

    void getVertexNeighborsPreprocess();

    void getVertexNeighbors(std::size_t v1, std::vector<DenseVertex>& graphNeighborhood);

    /** \brief The created space information */
    base::SpaceInformationPtr si_;

    /** \brief Class for storing the dense graph */
    DenseDBPtr denseDB_;

    /** \brief Class for managing various visualization features */
    base::VisualizerPtr visual_;

    // Prevent two vertices from being added to graph at same time
    boost::mutex vertexMutex_;

    /** \brief Constant used by k-PRM* to determine now many neighbors to connect to milestones */
    double findNearestKNeighbors_;
double radiusNeighbors_;

    std::size_t edgeConnectionStrategy_;

  public:
    /** \brief Various options for visualizing the algorithmns performance */
    bool visualizeGridGeneration_;

    /** \brief Distance between grid points (discretization level) */
    double discretization_;

};  // end of class Discretizer

}  // namespace bolt
}  // namespace tools
}  // namespace ompl
#endif
