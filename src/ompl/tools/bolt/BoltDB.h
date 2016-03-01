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
*/

#ifndef OMPL_TOOLS_BOLT_BOLTDB_
#define OMPL_TOOLS_BOLT_BOLTDB_

#include <ompl/base/StateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/Planner.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataStorage.h>
#include <ompl/base/State.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/base/PlannerTerminationCondition.h>

// Boost
#include <boost/range/adaptor/map.hpp>
#include <boost/unordered_map.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/pending/disjoint_sets.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>

namespace ompl
{
namespace geometric
{
/**
   @anchor BoltDB
   @par Short description
   Database for storing and retrieving past plans
*/

/// @cond IGNORE
OMPL_CLASS_FORWARD(BoltDB);
// OMPL_CLASS_FORWARD(BoltRetrieveRepair);
/// @endcond

/** \class ompl::geometric::BoltDBPtr
    \brief A boost shared pointer wrapper for ompl::tools::BoltDB */

/** \brief Save and load entire paths from file */
class BoltDB
{
    friend class BoltRetrieveRepair;

  public:
    ////////////////////////////////////////////////////////////////////////////////////////
    // VERTEX PROPERTIES
    ////////////////////////////////////////////////////////////////////////////////////////

    /** \brief Enumeration which specifies the reason a guard is added to the spanner. */
    enum GuardType
    {
        START,
        GOAL,
        COVERAGE,
        CONNECTIVITY,
        INTERFACE,
        QUALITY,
    };

    ////////////////////////////////////////////////////////////////////////////////////////
    // BOOST GRAPH DETAILS
    ////////////////////////////////////////////////////////////////////////////////////////

    /** \brief The type used internally for representing vertex IDs */
    typedef unsigned long int VertexIndexType;  // TODO(davetcoleman): just use size_t?

    /** \brief Pair of vertices which support an interface. */
    // typedef std::pair< VertexIndexType, VertexIndexType > VertexPair;

    ////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Interface information storage class, which does bookkeeping for criterion four. */
    // struct InterfaceData
    // {
    //     /** \brief States which lie inside the visibility region of a vertex and support an interface. */
    //     base::State *pointA_;
    //     base::State *pointB_;

    //     /** \brief States which lie just outside the visibility region of a vertex and support an interface. */
    //     base::State *sigmaA_;
    //     base::State *sigmaB_;

    //     /** \brief Last known distance between the two interfaces supported by points_ and sigmas. */
    //     double       last_distance_;

    //     /** \brief Constructor */
    //     InterfaceData() :
    //         pointA_(NULL),
    //         pointB_(NULL),
    //         sigmaA_(NULL),
    //         sigmaB_(NULL),
    //         last_distance_(std::numeric_limits<double>::infinity())
    //     {
    //     }

    //     /** \brief Clears the given interface data. */
    //     void clear(const base::SpaceInformationPtr& si)
    //     {
    //         if (pointA_)
    //         {
    //             si->freeState(pointA_);
    //             pointA_ = NULL;
    //         }
    //         if (pointB_)
    //         {
    //             si->freeState(pointB_);
    //             pointB_ = NULL;
    //         }
    //         if (sigmaA_)
    //         {
    //             si->freeState(sigmaA_);
    //             sigmaA_ = NULL;
    //         }
    //         if (sigmaB_)
    //         {
    //             si->freeState(sigmaB_);
    //             sigmaB_ = NULL;
    //         }
    //         last_distance_ = std::numeric_limits<double>::infinity();
    //     }

    //     /** \brief Sets information for the first interface (i.e. interface with smaller index vertex). */
    //     void setFirst(const base::State *p, const base::State *s, const base::SpaceInformationPtr& si)
    //     {
    //         if (pointA_)
    //             si->copyState(pointA_, p);
    //         else
    //             pointA_ = si->cloneState(p);
    //         if (sigmaA_)
    //             si->copyState(sigmaA_, s);
    //         else
    //             sigmaA_ = si->cloneState(s);
    //         if (pointB_)
    //             last_distance_ = si->distance(pointA_, pointB_);
    //     }

    //     /** \brief Sets information for the second interface (i.e. interface with larger index vertex). */
    //     void setSecond(const base::State *p, const base::State *s, const base::SpaceInformationPtr& si)
    //     {
    //         if (pointB_)
    //             si->copyState(pointB_, p);
    //         else
    //             pointB_ = si->cloneState(p);
    //         if (sigmaB_)
    //             si->copyState(sigmaB_, s);
    //         else
    //             sigmaB_ = si->cloneState(s);
    //         if (pointA_)
    //             last_distance_ = si->distance(pointA_, pointB_);
    //     }
    // };

    /** \brief the hash which maps pairs of neighbor points to pairs of states */
    // typedef boost::unordered_map< VertexPair, InterfaceData, boost::hash< VertexPair > > InterfaceHash;

    ////////////////////////////////////////////////////////////////////////////////////////
    // The InterfaceHash structure is wrapped inside of this struct due to a compilation error on
    // GCC 4.6 with Boost 1.48.  An implicit assignment operator overload does not compile with these
    // components, so an explicit overload is given here.
    // Remove this struct when the minimum Boost requirement is > v1.48.
    // struct InterfaceHashStruct
    // {
    //     InterfaceHashStruct& operator=(const InterfaceHashStruct &rhs) { interfaceHash = rhs.interfaceHash; return
    //     *this; }
    //     InterfaceHash interfaceHash;
    // };

    ////////////////////////////////////////////////////////////////////////////////////////
    // Vertex properties

    struct vertex_state_t
    {
        typedef boost::vertex_property_tag kind;
    };

    struct vertex_type_t
    {
        typedef boost::vertex_property_tag kind;
    };

    // struct vertex_interface_data_t {
    //     typedef boost::vertex_property_tag kind;
    // };

    ////////////////////////////////////////////////////////////////////////////////////////
    // Edge properties

    struct edge_collision_state_t
    {
        typedef boost::edge_property_tag kind;
    };

    /** \brief Possible collision states of an edge */
    enum EdgeCollisionState
    {
        NOT_CHECKED,
        IN_COLLISION,
        FREE
    };

    ////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Struct for passing around partially solved solutions */
    struct CandidateSolution
    {
        base::PathPtr path_;
        // Edge 0 is edge from vertex 0 to vertex 1. Thus, there is n-1 edges for n vertices
        std::vector<EdgeCollisionState> edgeCollisionStatus_;
        // TODO save the collision state of the vertexes also?

        std::size_t getStateCount()
        {
            return static_cast<ompl::geometric::PathGeometric&>(*path_).getStateCount();
        }

        ompl::geometric::PathGeometric& getGeometricPath()
        {
            return static_cast<ompl::geometric::PathGeometric&>(*path_);
        }
    };

    ////////////////////////////////////////////////////////////////////////////////////////
    /**
     \brief The underlying roadmap graph.

     \par Any BGL graph representation could be used here. Because we
     expect the roadmap to be sparse (m<n^2), an adjacency_list is more
     appropriate than an adjacency_matrix. Edges are undirected.

     *Properties of vertices*
     - vertex_state_t: an ompl::base::State* is required for OMPL
     - vertex_predecessor_t: The incremental connected components algorithm requires it
     - vertex_rank_t: The incremental connected components algorithm requires it
     - vertex_type_t - TODO
     - vertex_interface_data_t - needed by PRM2 for maintainings its sparse properties

     Note: If boost::vecS is not used for vertex storage, then there must also
     be a boost:vertex_index_t property manually added.

     *Properties of edges*
     - edge_weight_t - cost/distance between two vertices
     - edge_collision_state_t - used for lazy collision checking, determines if an edge has been checked
          already for collision. 0 = not checked/unknown, 1 = in collision, 2 = free
     */

    /** Wrapper for the vertex's multiple as its property. */
    typedef boost::property<
        vertex_state_t, base::State*,
        boost::property<
            boost::vertex_predecessor_t, VertexIndexType,
            boost::property<boost::vertex_rank_t, VertexIndexType,
                            boost::property<vertex_type_t, GuardType  //,
                                            // boost::property < vertex_interface_data_t, InterfaceHashStruct >
                                            > > > > VertexProperties;

    /** Wrapper for the double assigned to an edge as its weight property. */
    typedef boost::property<boost::edge_weight_t, double,
                            boost::property<edge_collision_state_t, int> > EdgeProperties;


    /** The underlying boost graph type (undirected weighted-edge adjacency list with above properties). */
    typedef boost::adjacency_list<boost::vecS,  // store in std::vector
                                  boost::vecS,  // store in std::vector
                                  boost::undirectedS, VertexProperties, EdgeProperties> Graph;

    /** \brief Vertex in Graph */
    typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;

    /** \brief Edge in Graph */
    typedef boost::graph_traits<Graph>::edge_descriptor Edge;

    ////////////////////////////////////////////////////////////////////////////////////////
    // Typedefs for property maps

    /** \brief Access map that stores the lazy collision checking status of each edge */
    typedef boost::property_map<Graph, edge_collision_state_t>::type EdgeCollisionStateMap;

    ////////////////////////////////////////////////////////////////////////////////////////
    /**
     * Used to artifically supress edges during A* search.
     * \implements ReadablePropertyMapConcept
     */
    class edgeWeightMap
    {
      private:
        const Graph& g_;  // Graph used
        const EdgeCollisionStateMap& collisionStates_;

      public:
        /** Map key type. */
        typedef Edge key_type;
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
        edgeWeightMap(const Graph& graph, const EdgeCollisionStateMap& collisionStates);

        /**
         * Get the weight of an edge.
         * \param e the edge
         * \return infinity if \a e lies in a forbidden neighborhood; actual weight of \a e otherwise
         */
        double get(Edge e) const;
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
        Vertex goal_;  // Goal Vertex of the search
        BoltDB* parent_;

      public:
        /**
         * Construct a visitor for a given search.
         * \param goal  goal vertex of the search
         */
        CustomAstarVisitor(Vertex goal, BoltDB* parent);

        /**
         * \brief Invoked when a vertex is first discovered and is added to the OPEN list.
         * \param v current Vertex
         * \param g graph we are searching on
         */
        void discover_vertex(Vertex v, const Graph& g) const;

        /**
         * \brief Check if we have arrived at the goal.
         * This is invoked on a vertex as it is popped from the queue (i.e., it has the lowest
         * cost on the OPEN list). This happens immediately before examine_edge() is invoked on
         * each of the out-edges of vertex u.
         * \param v current vertex
         * \param g graph we are searching on
         * \throw foundGoalException if \a u is the goal
         */
        void examine_vertex(Vertex v, const Graph& g) const;
    };

    ////////////////////////////////////////////////////////////////////////////////////////
    // BoltDB MEMBER FUNCTIONS
    ////////////////////////////////////////////////////////////////////////////////////////

    /** \brief Constructor needs the state space used for planning.
     *  \param space - state space
     */
    BoltDB(base::SpaceInformationPtr si);

    /** \brief Deconstructor */
    virtual ~BoltDB(void);

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
     * \param returned insertion time to add to db
     * \return true on success
     */
    bool postProcessesPath(ompl::geometric::PathGeometric& solutionPath, double& insertionTime);

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
    bool astarSearch(const BoltDB::Vertex start, const BoltDB::Vertex goal, std::vector<BoltDB::Vertex>& vertexPath);

    /**
     * \brief Get a vector of all the planner datas in the database
     */
    void getAllPlannerDatas(std::vector<ompl::base::PlannerDataPtr>& plannerDatas) const;

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
    const Graph& getRoadmap() const
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

    /**
     * \brief Get the sparse graph to save to file
     * \param data - where to convert the data into
     */
    virtual void getPlannerData(base::PlannerData& data) const;

    /**
     * \brief Set the sparse graph from file
     * \param a pre-built graph
     */
    void setPlannerData(const base::PlannerData& data);

    /**
     * \brief Discretize the space into a simple grid
     */
    void generateGrid();

    /** \brief Recursively discretize
     *  \param values - the joint positions currently searching over
     *  \param joint_id - the dimension currently on
     *  \param desired_depth - how many dimensions to discretize to
     */
    void recursiveDiscretization(std::vector<double>& values, std::size_t joint_id, std::size_t desired_depth);

    /** \brief Faster method for collision checking vertices */
    void checkVerticesThreaded(const std::vector<Vertex> &unvalidatedVertices);
    void checkVerticesThread(std::size_t startVertex, std::size_t endVertex, base::SpaceInformationPtr si,
        const std::vector<Vertex> &unvalidatedVertices);

    /**
     * \brief Connect vertices wherever possible
     */
    void generateEdges();

    /** \brief Collision check edges */
    void checkEdges();

    /** \brief Collision check edges using threading */
    void checkEdgesThreaded(const std::vector<Edge> &unvalidatedEdges);

    /** \brief Collision check edges in vector from [startEdge, endEdge] inclusive */
    void checkEdgesThread(std::size_t startEdge, std::size_t endEdge, base::SpaceInformationPtr si, const std::vector<Edge> &unvalidatedEdges);

    /** \brief Free all the memory allocated by the database */
    void freeMemory();

    /** \brief Check that the query vertex is initialized (used for internal nearest neighbor searches) */
    void initializeQueryState();

    /** \brief Compute distance between two milestones (this is simply distance between the states of the milestones) */
    double distanceFunction(const Vertex a, const Vertex b) const;

    /** \brief Compute distance between two milestones (this is simply distance between the states of the milestones) */
    double distanceFunction2(const Vertex a, const Vertex b) const;

    /** \brief Clear all past edge state information about in collision or not */
    void clearEdgeCollisionStates();

    /** \brief Visualize the stored database in an external program using callbacks */
    void displayDatabase();

    /** \brief Visualize a planner's data during runtime, externally, using a function callback
     *         This could be called whenever the graph changes */
    void vizStateCallback(const ompl::base::State* state, std::size_t type, double neighborRadius)
    {
        if (vizStateCallback_)
            vizStateCallback_(state, type, neighborRadius);
    }

    /** \brief Visualize a planner's data during runtime, externally, using a function callback
     *         This could be called whenever the graph changes */
    void vizEdgeCallback(const ompl::base::State* stateA, const ompl::base::State* stateB, double cost)
    {
        if (vizEdgeCallback_)
            vizEdgeCallback_(stateA, stateB, cost);
    }

    /** \brief Trigger visualizer to publish graphics */
    void vizTriggerCallback()
    {
        if (vizTriggerCallback_)
            vizTriggerCallback_();
    }

    /** \brief Set the callback to visualize/publish a planner's progress */
    void setVizCallbacks(ompl::base::VizStateCallback vizStateCallback, ompl::base::VizEdgeCallback vizEdgeCallback,
                         ompl::base::VizTriggerCallback vizTriggerCallback);

    /** \brief Visualize a planner's data during runtime, externally, using a function callback
     *         This could be called whenever the graph changes */
    void viz2StateCallback(const ompl::base::State* state, std::size_t type, double neighborRadius)
    {
        if (viz2StateCallback_)
            viz2StateCallback_(state, type, neighborRadius);
    }

    /** \brief Visualize a planner's data during runtime, externally, using a function callback
     *         This could be called whenever the graph changes */
    void viz2EdgeCallback(const ompl::base::State* stateA, const ompl::base::State* stateB, double cost)
    {
        if (viz2EdgeCallback_)
            viz2EdgeCallback_(stateA, stateB, cost);
    }

    /** \brief Trigger visualizer to publish graphics */
    void viz2TriggerCallback()
    {
        if (viz2TriggerCallback_)
            viz2TriggerCallback_();
    }

    /** \brief Set the callback to visualize/publish a planner's progress */
    void setViz2Callbacks(ompl::base::VizStateCallback vizStateCallback, ompl::base::VizEdgeCallback vizEdgeCallback,
                          ompl::base::VizTriggerCallback vizTriggerCallback);

    /** \brief Keep graph evenly weighted */
    void normalizeGraphEdgeWeights();

    /** \brief Helper for creating/loading graph vertices */
    Vertex addVertex(base::State* state, const GuardType& type);

    /** \brief Helper for creating/loading graph edges */
    Edge addEdge(const Vertex& v1, const Vertex& v2, const double weight);

    /** \brief Get whether to bias search using popularity of edges */
    bool getPopularityBiasEnabled()
    {
        return popularityBias_;
    }

    /** \brief Set whether to bias search using popularity of edges */
    void setPopularityBiasEnabled(bool enable)
    {
        popularityBias_ = enable;
    }

  protected:
    /** \brief The created space information */
    base::SpaceInformationPtr si_;

    /** \brief Sampler user for generating valid samples in the state space */
    base::ValidStateSamplerPtr sampler_;

    /** \brief Determine if a save is required */
    bool graphUnsaved_;

    /** \brief Helper class for storing each plannerData instance */
    ompl::base::PlannerDataStorage plannerDataStorage_;

    /** \brief Allow the database to save to file (new experiences) */
    bool savingEnabled_;

    /** \brief Nearest neighbors data structure */
    boost::shared_ptr<NearestNeighbors<Vertex> > nn_;

    /** \brief Connectivity graph */
    Graph g_;

    /** \brief Vertex for performing nearest neighbor queries. */
    Vertex queryVertex_;

    /** \brief Access to the weights of each Edge */
    boost::property_map<Graph, boost::edge_weight_t>::type edgeWeightProperty_;

    /** \brief Access to the collision checking state of each Edge */
    EdgeCollisionStateMap edgeCollisionStateProperty_;

    /** \brief Access to the internal base::state at each Vertex */
    boost::property_map<Graph, vertex_state_t>::type stateProperty_;

    /** \brief Access to the SPARS vertex type for the vertices */
    boost::property_map<Graph, vertex_type_t>::type typeProperty_;

    /** \brief Whether to bias search using popularity of edges */
    bool popularityBias_;

    /** \brief Option to enable debugging output */
    bool verbose_;

    /** \brief Optional callbacks to allow easy introspection of a planner's search progress */
    ompl::base::VizStateCallback vizStateCallback_;
    ompl::base::VizEdgeCallback vizEdgeCallback_;
    ompl::base::VizTriggerCallback vizTriggerCallback_;

    /** \brief Optional secondary callbacks to allow easy introspection of database */
    ompl::base::VizStateCallback viz2StateCallback_;
    ompl::base::VizEdgeCallback viz2EdgeCallback_;
    ompl::base::VizTriggerCallback viz2TriggerCallback_;

    /** \brief Discretization helper */
    base::State* next_disc_state_;

  public:
    /** \brief Various options for visualizing the algorithmns performance */
    bool visualizeAstar_;
    bool visualizeGridGeneration_;

    /** \brief Distance between grid points (discretization level) */
    double sparseDelta_;

};  // end of class BoltDB

}  // namespace tools
}  // namespace ompl
#endif
