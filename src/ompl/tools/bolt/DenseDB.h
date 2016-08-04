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
   Desc:   Experience database for storing and reusing past path plans
*/

#ifndef OMPL_TOOLS_BOLT_DENSEDB_
#define OMPL_TOOLS_BOLT_DENSEDB_

// OMPL
#include <ompl/base/StateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/Planner.h>
//#include <ompl/base/PlannerData.h>
//#include <ompl/base/PlannerDataStorage.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/tools/bolt/Visualizer.h>
#include <ompl/tools/bolt/SparseDB.h>
#include <ompl/tools/bolt/BoltGraph.h>

// Boost
#include <boost/function.hpp>
#include <boost/thread.hpp>

// this package
#include <ompl/tools/bolt/BoltStorage.h>

namespace ompl
{
namespace tools
{
namespace bolt
{
/**
   @anchor DenseDB
   @par Short description
   Database for storing and retrieving past plans
*/

/// @cond IGNORE
OMPL_CLASS_FORWARD(DenseDB);
OMPL_CLASS_FORWARD(SparseDB);
/// @endcond

/** \class ompl::tools::bolt::DenseDBPtr
    \brief A boost shared pointer wrapper for ompl::tools::bolt::DenseDB */

/** \brief Save and load entire paths from file */
class DenseDB
{
    friend class BoltRetrieveRepair;
    friend class SparseDB;
    friend class BoltStorage;
    friend class Discretizer;

  public:
    ////////////////////////////////////////////////////////////////////////////////////////
    /**
     * Used to artifically supress edges during A* search.
     * \implements ReadablePropertyMapConcept
     */
    class edgeWeightMap
    {
      private:
        const DenseGraph& g_;  // Graph used
        const DenseEdgeCollisionStateMap& collisionStates_;
        const double popularityBias_;
        const bool popularityBiasEnabled_;

      public:
        /** Map key type. */
        typedef DenseEdge key_type;
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
        edgeWeightMap(const DenseGraph& graph, const DenseEdgeCollisionStateMap& collisionStates,
                      const double& popularityBias, const bool popularityBiasEnabled);

        /**
         * Get the weight of an edge.
         * \param e the edge
         * \return infinity if \a e lies in a forbidden neighborhood; actual weight of \a e otherwise
         */
        double get(DenseEdge e) const;
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
        DenseVertex goal_;  // Goal Vertex of the search
        DenseDB* parent_;

      public:
        /**
         * Construct a visitor for a given search.
         * \param goal  goal vertex of the search
         */
        CustomAstarVisitor(DenseVertex goal, DenseDB* parent);

        /**
         * \brief Invoked when a vertex is first discovered and is added to the OPEN list.
         * \param v current Vertex
         * \param g graph we are searching on
         */
        void discover_vertex(DenseVertex v, const DenseGraph& g) const;

        /**
         * \brief Check if we have arrived at the goal.
         * This is invoked on a vertex as it is popped from the queue (i.e., it has the lowest
         * cost on the OPEN list). This happens immediately before examine_edge() is invoked on
         * each of the out-edges of vertex u.
         * \param v current vertex
         * \param g graph we are searching on
         * \throw foundGoalException if \a u is the goal
         */
        void examine_vertex(DenseVertex v, const DenseGraph& g) const;
    };

    ////////////////////////////////////////////////////////////////////////////////////////
    // DenseDB MEMBER FUNCTIONS
    ////////////////////////////////////////////////////////////////////////////////////////

     /** \brief Constructor needs the state space used for planning.
      *  \param space - state space
      */
     DenseDB(base::SpaceInformationPtr si, base::VisualizerPtr visual);

     /** \brief Deconstructor */
     virtual ~DenseDB(void);

    /** \brief Initialize database */
    bool setup();

    /**
     * \brief Load database from file
     * \param fileName - name of database file
     * \return true if file loaded successfully
     */
    bool load(const std::string& fileName);

    /**
     * \brief Add a new solution path to our database. Des not actually save to file so
     *        experience will be lost if save() is not called
     * \param new path - must be non-const because will be interpolated
     * \return true on success
     */
    bool postProcessPath(geometric::PathGeometric& solutionPath);

    /** \brief Helper function for postProcessPath */
    bool postProcessPathWithNeighbors(geometric::PathGeometric& solutionPath,
                                      const std::vector<DenseVertex>& visibleNeighborhood, bool recurseVerbose,
                                      std::vector<DenseVertex>& roadmapPath);

    /** \brief Update edge weights of dense graph based on this a newly created path */
    bool updateEdgeWeights(const std::vector<DenseVertex>& roadmapPath);

    /**
     * \brief Call itself recursively for each point in the trajectory, looking for vertices on the graph to connect to
     * \param inputPath - smoothed trajectory that we want to now convert into a 'snapped' trajectory
     * \param roadmapPath - resulting path that is 'snapped' onto the pre-existing roadmap
     * \param currVertexIndex - index in inputPath (smoothed path) that we are trying to connect to
     * \param prevGraphVertex - vertex we are trying to connect to
     * \param allValid - flag to determine if any of the edges were never found to be valid
     * \param verbose - whether to show lots of debug output to console
     * \return true on success
     */
    bool recurseSnapWaypoints(ompl::geometric::PathGeometric& inputPath, std::vector<DenseVertex>& roadmapPath,
                              std::size_t currVertexIndex, const DenseVertex& prevGraphVertex, bool& allValid,
                              bool verbose);

    /**
     * \brief Save loaded database to file, except skips saving if no paths have been added
     * \param fileName - name of database file
     * \return true if file saved successfully
     */
    bool saveIfChanged(const std::string& fileName);

    /**
     * \brief Save loaded database to file
     * \param fileName - name of database file
     * \return true if file saved successfully
     */
    bool save(const std::string& fileName);

    /** \brief Given two milestones from the same connected component, construct a path connecting them and set it as
     * the solution
     *  \param start
     *  \param goal
     *  \param vertexPath
     *  \return true if candidate solution found
     */
    bool astarSearch(const DenseVertex start, const DenseVertex goal, std::vector<DenseVertex>& vertexPath);

    void computeDensePath(const DenseVertex start, const DenseVertex goal, DensePath& path);

    /**
     * \brief Get a vector of all the planner datas in the database
     */
    // TODO(davetcoleman): remove the vector of plannerdatas, because really its just one
    //void getAllPlannerDatas(std::vector<ompl::base::PlannerDataPtr>& plannerDatas) const;

    /** \brief Print info to screen */
    void debugVertex(const ompl::base::PlannerDataVertex& vertex);
    void debugState(const ompl::base::State* state);

    /** \brief Getter for enabling experience database saving */
    bool getSavingEnabled()
    {
        return savingEnabled_;
    }

    /** \brief Setter for enabling experience database saving */
    void setSavingEnabled(bool savingEnabled)
    {
        savingEnabled_ = savingEnabled;
    }

    /**
     * \brief Check if anything has been loaded into DB
     * \return true if has no nodes
     */
    bool isEmpty()
    {
        return !getNumVertices();
    }

    /** \brief Retrieve the computed roadmap. */
    const DenseGraph& getRoadmap() const
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

    /** \brief Hook for adding vertices from BoltStorage */
    //void addVertexFromFile(BoltStorage::PlannerDataVertex *v);
    void addVertexFromFile(BoltStorage::BoltVertexData v);

    /** \brief Hook for adding edges from BoltStorage */
    void addEdgeFromFile(BoltStorage::BoltEdgeData e);

    /**
     * \brief Set the sparse graph from file
     * \param a pre-built graph
     */
    void loadFromPlannerData(const base::PlannerData& data);

    /** \brief Clone the graph to have second and third layers for task planning then free space planning */
    void generateTaskSpace();

    /** \brief Free all the memory allocated by the database */
    void freeMemory();

    /** \brief Compute distance between two milestones (this is simply distance between the states of the milestones) */
    double distanceFunction(const DenseVertex a, const DenseVertex b) const;

    /** \brief Compute distance between two milestones (this is simply distance between the states of the milestones) */
    double distanceFunction2(const DenseVertex a, const DenseVertex b) const;

    /** \brief Compute the heuristic distance between the current node and the next goal */
    double distanceFunctionTasks(const DenseVertex a, const DenseVertex b) const;

    /** \brief Helper for getting the task level value from a state */
    std::size_t getTaskLevel(const DenseVertex& v) const;
    std::size_t getTaskLevel(const base::State* state) const;

    /** \brief Check that the query vertex is initialized (used for internal nearest neighbor searches) */
    void initializeQueryState();

    /**
     * \brief Get the sparse graph to save to file
     * \param data - where to convert the data into
     */
    //virtual void getPlannerData(base::PlannerData& data) const;

    /** \brief Clear all past edge state information about in collision or not */
    void clearEdgeCollisionStates();

    /** \brief Visualize the stored database in an external program using callbacks */
    void displayDatabase();

    /** \brief Keep graph evenly weighted */
    void normalizeGraphEdgeWeights();

    /** \brief Helper for creating/loading graph vertices */
    DenseVertex addVertex(base::State* state, const GuardType& type);

    /** \brief Helper for creating/loading graph edges */
    DenseEdge addEdge(const DenseVertex& v1, const DenseVertex& v2, const double weight);

    /** \brief Get whether to bias search using popularity of edges */
    bool getPopularityBiasEnabled()
    {
        return popularityBias_;
    }

    /** \brief Set whether to bias search using popularity of edges */
    void setPopularityBiasEnabled(bool enable)
    {
        popularityBiasEnabled_ = enable;
    }

    /** \brief Set how much to bias search using popularity of edges */
    void setPopularityBias(double popularityBias)
    {
        popularityBias_ = popularityBias;
    }

    /** \brief Remove parts of graph that were intended to be temporary */
    void cleanupTemporaryVerticies();

    /** \brief Testing code for integrating Decartes */
    bool addCartPath(std::vector<base::State*> path);

    /**
     * \brief Helper for connecting both sides of a cartesian path into a dual level graph
     * \param fromState - the endpoint (start or goal) we are connecting from the cartesian path to the graph
     * \param level - what task level we are connecting to - either 0 or 2 (bottom layer or top layer)
     * \param minConnectorVertex - the vertex on the main graph that has the shortest path to connecting to the
     * cartesian path
     * \return true on success
     */
    bool connectStateToNeighborsAtLevel(const DenseVertex& fromVertex, const std::size_t level,
                                        DenseVertex& minConnectorVertex);

    /**
     * \brief Get neighbors within radius
     * \param denseV - origin state to search from
     * \param graphNeighborhood - resulting nearby states
     * \param visibleNeighborhood - resulting nearby states that are visible
     * \param searchRadius - how far to search
     * \param countIndent - debugging tool
     */
    void findGraphNeighbors(const DenseVertex& denseV, std::vector<DenseVertex>& graphNeighborhood,
                            std::vector<DenseVertex>& visibleNeighborhood, double searchRadius, std::size_t coutIndent);

    void findGraphNeighbors(base::State* state, std::vector<DenseVertex>& graphNeighborhood,
                            std::vector<DenseVertex>& visibleNeighborhood, double searchRadius, std::size_t coutIndent);

    /** \brief Get k number of neighbors near a state at a certain level that have valid motions */
    void getNeighborsAtLevel(const base::State* origState, const std::size_t level, const std::size_t kNeighbors,
                             std::vector<DenseVertex>& neighbors);

    /** \brief Shortcut for visualizing an edge */
    void viz1Edge(DenseEdge& e);

    /** \brief Error checking function to ensure solution has correct task path/level changes */
    bool checkTaskPathSolution(geometric::PathGeometric& path, base::State* start, base::State* goal);

    /** \brief Check that all states are the correct type */
    void checkStateType();

    /** \brief Getter for using task planning flag */
    const bool& getUseTaskPlanning() const
    {
        return useTaskPlanning_;
    }

    /** \brief Setter for using task planning flag */
    void setUseTaskPlanning(const bool& useTaskPlanning)
    {
        useTaskPlanning_ = useTaskPlanning;
    }

    /** \brief Get class for managing various visualization features */
    base::VisualizerPtr getVisual()
    {
        return visual_;
    }

    /** \brief Get class that contains the sparse DB */
    SparseDBPtr getSparseDB()
    {
        return sparseDB_;
    }

    /**
     * \brief Sometimes the dense graph's discretization is not enough, so we have the ability to manually add
     *        samples and connect to the graph
     * \param denseV - a newly created vertex that needs to be connected to the dense graph
     *                 the vertex should cover a currently un-visible region of the configuration space
     */
    void connectNewVertex(DenseVertex denseV);

    /** \brief Helper for counting the number of disjoint sets in the sparse graph */
    std::size_t getDisjointSetsCount(bool verbose = false);

    bool checkConnectedComponents();

  protected:
    /** \brief The created space information */
    base::SpaceInformationPtr si_;

    /** \brief Class for managing various visualization features */
    base::VisualizerPtr visual_;

    /** \brief Class to store lighter version of graph */
    SparseDBPtr sparseDB_;

    /** \brief Sampler user for generating valid samples in the state space */
    base::ValidStateSamplerPtr sampler_;  // TODO(davetcoleman): remove this unused sampler

    /** \brief Determine if a save is required */
    bool graphUnsaved_;

    /** \brief Helper class for storing each plannerData instance */
    //ompl::base::PlannerDataStorage plannerDataStorage_;

    /** \brief Nearest neighbors data structure */
    boost::shared_ptr<NearestNeighbors<DenseVertex> > nn_;

    /** \brief Connectivity graph */
    DenseGraph g_;

    /** \brief Vertex for performing nearest neighbor queries. */
    DenseVertex queryVertex_;

    /** \brief Access to the weights of each Edge */
    boost::property_map<DenseGraph, boost::edge_weight_t>::type edgeWeightProperty_;

    /** \brief Access to the collision checking state of each Edge */
    DenseEdgeCollisionStateMap edgeCollisionStateProperty_;

    /** \brief Access to the internal base::state at each Vertex */
    boost::property_map<DenseGraph, vertex_state_t>::type stateProperty_;

    /** \brief Access to the SPARS vertex type for the vertices */
    boost::property_map<DenseGraph, vertex_type_t>::type typeProperty_;

    /** \brief Access to the representatives of the Dense vertices */
    boost::property_map<DenseGraph, vertex_sparse_rep_t>::type representativesProperty_;

    /** \brief Data structure that maintains the connected components */
    boost::disjoint_sets<boost::property_map<DenseGraph, boost::vertex_rank_t>::type,
                         boost::property_map<DenseGraph, boost::vertex_predecessor_t>::type> disjointSets_;


    /** \brief Track vertex for later removal if temporary */
    std::vector<DenseVertex> tempVerticies_;
    DenseVertex startConnectorVertex_;
    DenseVertex endConnectorVertex_;
    double distanceAcrossCartesian_;

  public:

    /** \brief Allow the database to save to file (new experiences) */
    bool savingEnabled_;

    /** \brief Whether to bias search using popularity of edges */
    bool popularityBiasEnabled_;

    /** \brief How much influence should the popularity costs have over the admissible heuristic */
    double popularityBias_;

    /** \brief Option to enable debugging output */
    bool verbose_;

    /** \brief Are we task planning i.e. for hybrid cartesian paths? */
    bool useTaskPlanning_;

    /** \brief Option to enable debugging output */
    bool snapPathVerbose_;

    /** \brief Various options for visualizing the algorithmns performance */
    bool visualizeAstar_;
    bool visualizeCartNeighbors_;
    bool visualizeCartPath_;
    bool visualizeSnapPath_;
    double visualizeSnapPathSpeed_;
    bool visualizeAddSample_;
    bool visualizeDatabaseVertices_;
    bool visualizeDatabaseEdges_;

    /** \brief Visualization speed of astar search, num of seconds to show each vertex */
    double visualizeAstarSpeed_;

    /** \brief Keep the average cost of the graph at this level */
    double desiredAverageCost_;

};  // end of class DenseDB

}  // namespace bolt
}  // namespace tools
}  // namespace ompl
#endif
