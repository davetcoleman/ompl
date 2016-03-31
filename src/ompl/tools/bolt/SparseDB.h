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
   Desc:   Sparse experience database for storing and reusing past path plans
*/

#ifndef OMPL_TOOLS_BOLT_SPARSEDB_
#define OMPL_TOOLS_BOLT_SPARSEDB_

#include <ompl/base/StateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/Planner.h>
#include <ompl/base/State.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/tools/bolt/Visualizer.h>
#include <ompl/tools/bolt/BoltDB.h>
#include <ompl/tools/bolt/BoltGraph.h>

// Boost
// #include <boost/range/adaptor/map.hpp>
// #include <boost/unordered_map.hpp>
// #include <boost/graph/graph_traits.hpp>
// #include <boost/graph/adjacency_list.hpp>
// #include <boost/graph/filtered_graph.hpp>
// #include <boost/graph/graph_utility.hpp>
// #include <boost/graph/astar_search.hpp>
// #include <boost/graph/connected_components.hpp>
// #include <boost/property_map/property_map.hpp>
// #include <boost/pending/disjoint_sets.hpp>
#include <boost/function.hpp>

namespace ompl
{
namespace tools
{
namespace bolt
{
/**
   @anchor SparsDB
   @par Short description
   Database for storing and retrieving past plans
*/

/// @cond IGNORE
OMPL_CLASS_FORWARD(SparseDB);
OMPL_CLASS_FORWARD(BoltDB);
/// @endcond

/** \class ompl::tools::bolt::::SparseDBPtr
    \brief A boost shared pointer wrapper for ompl::tools::SparseDB */

/** \brief Save and load entire paths from file */
class SparseDB
{
  public:

    ////////////////////////////////////////////////////////////////////////////////////////
    /**
     * Used to artifically supress edges during A* search.
     * \implements ReadablePropertyMapConcept
     */
    class edgeWeightMap
    {
      private:
        const SparseGraph& g_;  // Graph used
        const SparseEdgeCollisionStateMap& collisionStates_;
        const double popularityBias_;
        const bool popularityBiasEnabled_;

      public:
        /** Map key type. */
        typedef SparseEdge key_type;
        /** Map value type. */
        typedef double value_type;
        /** Map auxiliary value type. */
        typedef double& reference;
        /** Map type. */
        typedef boost::readable_property_map_tag category;

        /**
         * Construct map for certain constraints.
         * \param graph         Graph to use
         */
        edgeWeightMap(const SparseGraph& graph, const SparseEdgeCollisionStateMap& collisionStates, const double& popularityBias,
                      const bool popularityBiasEnabled);

        /**
         * Get the weight of an edge.
         * \param e the edge
         * \return infinity if \a e lies in a forbidden neighborhood; actual weight of \a e otherwise
         */
        double get(SparseEdge e) const;
    };

    ////////////////////////////////////////////////////////////////////////////////////////
    /**
     * Thrown to stop the A* search when finished.
     */
    class foundGoalException
    {
    };

    ////////////////////////////////////////////////////////////////////////////////////////
    /**
     * Vertex visitor to check if A* search is finished.
     * \implements AStarVisitorConcept
     * See http://www.boost.org/doc/libs/1_58_0/libs/graph/doc/AStarVisitor.html
     */
    class CustomAstarVisitor : public boost::default_astar_visitor
    {
      private:
        SparseVertex goal_;  // Goal Vertex of the search
        SparseDB* parent_;

      public:
        /**
         * Construct a visitor for a given search.
         * \param goal  goal vertex of the search
         */
        CustomAstarVisitor(SparseVertex goal, SparseDB* parent);

        /**
         * \brief Invoked when a vertex is first discovered and is added to the OPEN list.
         * \param v current Vertex
         * \param g graph we are searching on
         */
        void discover_vertex(SparseVertex v, const SparseGraph& g) const;

        /**
         * \brief Check if we have arrived at the goal.
         * This is invoked on a vertex as it is popped from the queue (i.e., it has the lowest
         * cost on the OPEN list). This happens immediately before examine_edge() is invoked on
         * each of the out-edges of vertex u.
         * \param v current vertex
         * \param g graph we are searching on
         * \throw foundGoalException if \a u is the goal
         */
        void examine_vertex(SparseVertex v, const SparseGraph& g) const;
    };

    ////////////////////////////////////////////////////////////////////////////////////////
    // SparseDB MEMBER FUNCTIONS
    ////////////////////////////////////////////////////////////////////////////////////////

    /** \brief Constructor needs the state space used for planning.
     *  \param space - state space
     */
    SparseDB(base::SpaceInformationPtr si, BoltDB* boltDB, VisualizerPtr visual);

    /** \brief Deconstructor */
    virtual ~SparseDB(void);

    /** \brief Initialize database */
    bool setup();

    /** \brief Given two milestones from the same connected component, construct a path connecting them and set it as
     * the solution
     *  \param start
     *  \param goal
     *  \param vertexPath
     *  \return true if candidate solution found
     */
    bool astarSearch(const SparseVertex start, const SparseVertex goal,
                     std::vector<SparseVertex>& vertexPath);

    /** \brief Print info to screen */
    void debugVertex(const ompl::base::PlannerDataVertex& vertex);
    void debugState(const ompl::base::State* state);

    /** \brief Retrieve the computed roadmap. */
    const SparseGraph& getRoadmap() const
    {
        return g_;
    }

    /** \brief Get the number of vertices in the sparse roadmap. */
    unsigned int getNumVertices() const
    {
        return boost::num_vertices(g_);
    }

    /** \brief Get the number of edges in the sparse roadmap. */
    unsigned int getNumEdges() const
    {
        return boost::num_edges(g_);
    }

    /** \brief Free all the memory allocated by the database */
    void freeMemory();

    /** \brief Compute distance between two milestones (this is simply distance between the states of the milestones) */
    double distanceFunction(const SparseVertex a, const SparseVertex b) const;

    /** \brief Check that the query vertex is initialized (used for internal nearest neighbor searches) */
    void initializeQueryState();

    /** \brief Clear all past edge state information about in collision or not */
    void clearEdgeCollisionStates();

    /** \brief Create a SPARS graph from the discretized dense graph and its popularity metric */
    void createSPARS();

    /* ----------------------------------------------------------------------------------------*/
    /** \brief SPARS-related functions */
    bool addStateToRoadmap(const base::PlannerTerminationCondition& ptc);  // DenseVertex denseVertex);
    SparseVertex addVertex(base::State* state, const GuardType& type);
    std::size_t getVizVertexType(const GuardType& type);
    void addEdge(SparseVertex v1, SparseVertex v2, std::size_t visualColor, std::size_t coutIndent);
    bool checkAddCoverage(const base::State* qNew, std::vector<SparseVertex>& visibleNeighborhood,
                          std::size_t coutIndent);
    bool checkAddConnectivity(const base::State* qNew, std::vector<SparseVertex>& visibleNeighborhood,
                              std::size_t coutIndent);
    bool checkAddInterface(const base::State* qNew, std::vector<SparseVertex>& graphNeighborhood,
                           std::vector<SparseVertex>& visibleNeighborhood, std::size_t coutIndent);
    // bool checkAsymptoticOptimal(DenseVertex denseVertex, std::size_t coutIndent);
    // void getInterfaceNeighborhood(DenseVertex q, std::vector<DenseVertex> &interfaceNeighborhood);
    void findGraphNeighbors(base::State* state, std::vector<SparseVertex>& graphNeighborhood,
                            std::vector<SparseVertex>& visibleNeighborhood, std::size_t coutIndent);
    bool sameComponent(SparseVertex m1, SparseVertex m2);

  protected:
    /** \brief The created space information */
    base::SpaceInformationPtr si_;

    /** \brief The database of motions to search through */
    BoltDB* boltDB_;

    /** \brief Class for managing various visualization features */
    VisualizerPtr visual_;

    /** \brief Nearest neighbors data structure */
    boost::shared_ptr<NearestNeighbors<SparseVertex> > nn_;

    /** \brief Connectivity graph */
    SparseGraph g_;

    /** \brief Vertex for performing nearest neighbor queries. */
    SparseVertex queryVertex_;

    /** \brief Access to the weights of each Edge */
    boost::property_map<SparseGraph, boost::edge_weight_t>::type edgeWeightPropertySparse_;

    /** \brief Access to the collision checking state of each Edge */
    SparseEdgeCollisionStateMap edgeCollisionStatePropertySparse_;

    /** \brief Access to the internal base::state at each Vertex */
    boost::property_map<SparseGraph, vertex_state_t>::type statePropertySparse_;

    /** \brief Access to the SPARS vertex type for the vertices */
    boost::property_map<SparseGraph, vertex_type_t>::type typePropertySparse_;

    /** \brief Data structure that maintains the connected components */
    boost::disjoint_sets<boost::property_map<SparseGraph, boost::vertex_rank_t>::type,
                         boost::property_map<SparseGraph, boost::vertex_predecessor_t>::type> disjointSets_;

    /** \brief Option to enable debugging output */
    bool verbose_;

  public:
    /** \brief Various options for visualizing the algorithmns performance */
    bool visualizeAstar_;

    bool visualizeSparsCreation_;

    /** \brief Amount of sub-optimality allowed */
    double sparseDelta_;

    /** \brief Visualization speed of astar search, num of seconds to show each vertex */
    double visualizeAstarSpeed_;

};  // end of class SparseDB

}  // namespace bolt
}  // namespace tools
}  // namespace ompl

#endif  // OMPL_TOOLS_BOLT_SPARSEDB_
