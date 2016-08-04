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
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/base/Planner.h>
#include <ompl/base/State.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/tools/bolt/Visualizer.h>
#include <ompl/tools/bolt/DenseDB.h>
#include <ompl/tools/bolt/BoltGraph.h>

// Boost
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
OMPL_CLASS_FORWARD(DenseDB);
/// @endcond

/** \class ompl::tools::bolt::::SparseDBPtr
    \brief A boost shared pointer wrapper for ompl::tools::SparseDB */

/** \brief Save and load entire paths from file */
class SparseDB
{
    friend class BoltRetrieveRepair;
    friend class DenseDB;

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
        edgeWeightMap(const SparseGraph& graph, const SparseEdgeCollisionStateMap& collisionStates,
                      const double& popularityBias, const bool popularityBiasEnabled);

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
    SparseDB(base::SpaceInformationPtr si, DenseDB* denseDB, base::VisualizerPtr visual);

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
    bool astarSearch(const SparseVertex start, const SparseVertex goal, std::vector<SparseVertex>& vertexPath);

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

    /**
     * \brief Check if anything has been loaded into DB
     * \return true if has no nodes
     */
    bool isEmpty()
    {
        return !getNumVertices();
    }

    /** \brief Distance between two states with special bias using popularity */
    double astarHeuristic(const SparseVertex a, const SparseVertex b) const;

    /** \brief Compute distance between two milestones (this is simply distance between the states of the milestones) */
    double distanceFunction(const SparseVertex a, const SparseVertex b) const;

    /** \brief Check that the query vertex is initialized (used for internal nearest neighbor searches) */
    void initializeQueryState();

    /** \brief Clear all past edge state information about in collision or not */
    void clearEdgeCollisionStates();

    /** \brief Create a SPARS graph from the discretized dense graph and its popularity metric */
    void createSPARS();
    bool findSparseRepresentatives();

    /** \brief Helper function for choosing the correct method for vertex insertion ordering */
    void getVertexInsertionOrdering(std::vector<WeightedVertex> &vertexInsertionOrder);

    /** \brief Add random samples until graph is fully connected */
    void eliminateDisjointSets();

    /** \brief Attempt to re-add neighbors from dense graph that have not been added yet */
    bool reinsertNeighborsIntoSpars(const SparseVertex& newVertex);

    /** \brief Helper for counting the number of disjoint sets in the sparse graph */
    std::size_t getDisjointSetsCount(bool verbose = false);

    bool getPopularityOrder(std::vector<WeightedVertex>& vertexInsertionOrder);
    bool getDefaultOrder(std::vector<WeightedVertex>& vertexInsertionOrder);
    bool getRandomOrder(std::vector<WeightedVertex>& vertexInsertionOrder);

    /** \brief Helper function for random integer creation */
    int iRand(int min, int max);

    /**
     * \brief Run various checks/criteria to determine if to keep DenseVertex in sparse graph
     * \param denseVertex - the original vertex to consider
     * \param newVertex - if function returns true, the newly generated sparse vertex
     * \param addReason - if function returns true, the reson the denseVertex was added to the sparse graph
     * \return true on success
     */
    bool addStateToRoadmap(DenseVertex denseVertex, SparseVertex& newVertex, GuardType& addReason);

    /* ----------------------------------------------------------------------------------------*/
    /** \brief SPARS-related functions */
    bool checkAddCoverage(const DenseVertex& denseV, std::vector<SparseVertex>& visibleNeighborhood,
                          SparseVertex& newVertex, std::size_t coutIndent);
    bool checkAddConnectivity(const DenseVertex& denseV, std::vector<SparseVertex>& visibleNeighborhood,
                              SparseVertex& newVertex, std::size_t coutIndent);
    bool checkAddInterface(const DenseVertex& denseV, std::vector<SparseVertex>& graphNeighborhood,
                           std::vector<SparseVertex>& visibleNeighborhood, SparseVertex& newVertex,
                           std::size_t coutIndent);
    void getInterfaceNeighborhood(const DenseVertex& denseV, std::vector<DenseVertex>& interfaceNeighborhood,
                                  std::size_t coutIndent);
    /**
     * \brief Get neighbors within sparseDelta radius
     * \param denseV - origin state to search from
     * \param graphNeighborhood - resulting nearby states
     * \param visibleNeighborhood - resulting nearby states that are visible
     * \param countIndent - debugging tool
     */
    void findGraphNeighbors(const DenseVertex& denseV, std::vector<SparseVertex>& graphNeighborhood,
                            std::vector<SparseVertex>& visibleNeighborhood, std::size_t coutIndent);

    DenseVertex getInterfaceNeighbor(DenseVertex q, SparseVertex rep);
    bool sameComponent(SparseVertex m1, SparseVertex m2);
    SparseVertex addVertex(DenseVertex denseV, const GuardType& type);
    std::size_t getVizVertexType(const GuardType& type);
    void addEdge(SparseVertex v1, SparseVertex v2, std::size_t visualColor, std::size_t coutIndent);

    /** \brief Show in visualizer the sparse graph */
    void displayDatabase(bool showVertices = false);

    /** \brief Custom A* visitor statistics */
    void recordNodeOpened() // discovered
    {
        numNodesOpened_++;
    }
    void recordNodeClosed() // examined
    {
        numNodesClosed_++;
    }


  public:
    /** \brief Shortcut function for getting the state of a vertex */
    base::State*& getSparseState(const SparseVertex& v);
    const base::State* getSparseStateConst(const SparseVertex& v) const;
    base::State*& getDenseState(const DenseVertex& denseV);

  protected:
    /** \brief The created space information */
    base::SpaceInformationPtr si_;

    /** \brief The database of motions to search through */
    DenseDB* denseDB_;

    /** \brief Class for managing various visualization features */
    base::VisualizerPtr visual_;

    /** \brief Nearest neighbors data structure */
    boost::shared_ptr<NearestNeighbors<SparseVertex> > nn_;

    /** \brief Connectivity graph */
    SparseGraph g_;

    /** \brief Vertex for performing nearest neighbor queries. */
    SparseVertex queryVertex_;

    /** \brief Geometric Path variable used for smoothing out paths. */
    geometric::PathGeometric smoothingGeomPath_;

    /** \brief Access to the weights of each Edge */
    boost::property_map<SparseGraph, boost::edge_weight_t>::type edgeWeightPropertySparse_;

    /** \brief Access to the collision checking state of each Edge */
    SparseEdgeCollisionStateMap edgeCollisionStatePropertySparse_;

    /** \brief Access to the internal base::state at each Vertex */
    boost::property_map<SparseGraph, vertex_dense_pointer_t>::type denseVertexProperty_;

    /** \brief Access to the SPARS vertex type for the vertices */
    boost::property_map<SparseGraph, vertex_type_t>::type typePropertySparse_;

    /** \brief Access to all non-interface supporting vertices of the sparse nodes */
    //boost::property_map<SparseGraph, vertex_list_t>::type nonInterfaceListsProperty_;

    /** \brief Access to the interface-supporting vertice hashes of the sparse nodes */
    //boost::property_map<SparseGraph, vertex_interface_list_t>::type interfaceListsProperty_;

    /** \brief Access to the popularity of each node */
    boost::property_map<SparseGraph, vertex_popularity_t>::type vertexPopularity_;

    /** \brief Data structure that maintains the connected components */
    boost::disjoint_sets<boost::property_map<SparseGraph, boost::vertex_rank_t>::type,
                         boost::property_map<SparseGraph, boost::vertex_predecessor_t>::type> disjointSets_;

    /** \brief A path simplifier used to simplify dense paths added to S */
    geometric::PathSimplifierPtr psimp_;

    /** \brief Special flag for tracking mode when inserting into sparse graph */
    bool secondSparseInsertionAttempt_;

    /** \brief Amount of sub-optimality allowed */
    double sparseDelta_;

    /** \brief For debugging */
    bool specialMode_;

    /** \brief Show what nodes are added on top of the regular SPARS graph */
    bool visualizeOverlayNodes_;

    /** \brief Cache the maximum extent for later re-use */
    double maxExtent_;

  public:

    /** \brief Astar statistics */
    std::size_t numNodesOpened_;
    std::size_t numNodesClosed_;

    /** \brief SPARS parameter for dense graph connection distance as a fraction of max. extent */
    double denseDeltaFraction_;

    /** \brief Maximum visibility range for nodes in the graph as a fraction of maximum extent. */
    double sparseDeltaFraction_;

    /** \brief The stretch factor in terms of graph spanners for SPARS to check against */
    double stretchFactor_;

    /** \brief SPARS parameter for dense graph connection distance */
    double denseDelta_;

    /** \brief How much the popularity of a node can cause its cost-to-go heuristic to be underestimated */
    double percentMaxExtentUnderestimate_;

    /** \brief Change verbosity levels */
    bool checksVerbose_;
    bool disjointVerbose_;
    bool fourthCheckVerbose_;

    /** \brief Various options for visualizing the algorithmns performance */
    bool visualizeAstar_;

    /** \brief Show the sparse graph being generated */
    bool visualizeSparsGraph_;
    double visualizeSparsGraphSpeed_;
    bool visualizeDenseRepresentatives_;
    bool visualizeNodePopularity_;

    /** \brief Visualization speed of astar search, num of seconds to show each vertex */
    double visualizeAstarSpeed_;

    /** \brief Method for ordering of vertex insertion */
    int sparseCreationInsertionOrder_;

    /** \brief For statistics */
    int numGraphGenerations_;
    int numSamplesAddedForDisjointSets_;
};  // end of class SparseDB

}  // namespace bolt
}  // namespace tools
}  // namespace ompl

#endif  // OMPL_TOOLS_BOLT_SPARSEDB_
