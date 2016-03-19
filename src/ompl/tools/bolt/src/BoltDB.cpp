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
   Desc:
*/

/* Author: Dave Coleman */

// OMPL
#include <ompl/tools/bolt/BoltDB.h>
#include <ompl/base/ScopedState.h>
#include <ompl/util/Time.h>
#include <ompl/util/Console.h>
#include <ompl/base/PlannerDataStorage.h>
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>  // TODO: remove, this is not space agnostic

// Boost
#include <boost/filesystem.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/foreach.hpp>
#include <boost/thread.hpp>

// C++
#include <limits>

// Allow hooks for visualizing planner
#define OMPL_BOLT_DEBUG

namespace og = ompl::geometric;
namespace ob = ompl::base;

// edgeWeightMap methods ////////////////////////////////////////////////////////////////////////////

BOOST_CONCEPT_ASSERT(
    (boost::ReadablePropertyMapConcept<ompl::geometric::BoltDB::edgeWeightMap, ompl::geometric::BoltDB::Edge>));

og::BoltDB::edgeWeightMap::edgeWeightMap(const Graph &graph, const EdgeCollisionStateMap &collisionStates)
    : g_(graph), collisionStates_(collisionStates)
{
}

double og::BoltDB::edgeWeightMap::get(Edge e) const
{
    // Get the status of collision checking for this edge
    if (collisionStates_[e] == IN_COLLISION)
        return std::numeric_limits<double>::infinity();

    // Method 1
    const double weight = boost::get(boost::edge_weight, g_, e);

    // Method 2
    // if (popularityBias_)
    // {
    //     static const double popularity_bias = 10;
    //     weight = boost::get(boost::edge_weight, g_, e) / 100.0 * popularity_bias;
    // }

    // Method 3 - less optimal but faster planning time
    // const double weighted_astar = 0.8;
    // const double weight = boost::get(boost::edge_weight, g_, e) * weighted_astar;

    // std::cout << "getting weight of edge " << e << " with value " << weight << std::endl;

    return weight;
}

namespace boost
{
    double get(const og::BoltDB::edgeWeightMap &m, const og::BoltDB::Edge &e)
    {
        return m.get(e);
    }
}

// CustomAstarVisitor methods ////////////////////////////////////////////////////////////////////////////

BOOST_CONCEPT_ASSERT((boost::AStarVisitorConcept<og::BoltDB::CustomAstarVisitor, og::BoltDB::Graph>));

og::BoltDB::CustomAstarVisitor::CustomAstarVisitor(Vertex goal, BoltDB *parent) : goal_(goal), parent_(parent)
{
}

void og::BoltDB::CustomAstarVisitor::discover_vertex(Vertex v, const Graph &) const
{
    if (parent_->visualizeAstar_)
        parent_->vizStateCallback(parent_->stateProperty_[v], 1, 1);
}

void og::BoltDB::CustomAstarVisitor::examine_vertex(Vertex v, const Graph &) const
{
    if (parent_->visualizeAstar_)
    {
        parent_->vizStateCallback(parent_->stateProperty_[v], 5, 1);
        parent_->vizTriggerCallback();
        usleep(parent_->visualizeAstarSpeed_ * 1000000);
    }

    if (v == goal_)
        throw foundGoalException();
}

// Actual class ////////////////////////////////////////////////////////////////////////////

og::BoltDB::BoltDB(base::SpaceInformationPtr si)
    : si_(si)
    , graphUnsaved_(false)
    , savingEnabled_(true)
      // Property accessors of edges
    , edgeWeightProperty_(boost::get(boost::edge_weight, g_))
    , edgeCollisionStateProperty_(boost::get(edge_collision_state_t(), g_))
      // Property accessors of vertices
    , stateProperty_(boost::get(vertex_state_t(), g_))
    , typeProperty_(boost::get(vertex_type_t(), g_))
      // interfaceDataProperty_(boost::get(vertex_interface_data_t(), g_)),
    , popularityBias_(false)
    , verbose_(true)
    , distanceAcrossCartesian_(0.0)
    , visualizeAstar_(false)
    , visualizeGridGeneration_(false)
    , visualizeCartNeighbors_(false)
    , visualizeCartPath_(false)
    , sparseDelta_(2.0)
    , visualizeAstarSpeed_(0.1)
{
    if (!nn_)
    {
        nn_.reset(new NearestNeighborsGNATNoThreadSafety<Vertex>());
    }
    nn_->setDistanceFunction(boost::bind(&og::BoltDB::distanceFunction, this, _1, _2));
}

og::BoltDB::~BoltDB(void)
{
    if (graphUnsaved_)
        OMPL_WARN("The database is being unloaded with unsaved experiences");
    freeMemory();
}

void og::BoltDB::freeMemory()
{
    BOOST_FOREACH (Vertex v, boost::vertices(g_))
    {
        // BOOST_FOREACH (InterfaceData &d, interfaceDataProperty_[v].interfaceHash | boost::adaptors::map_values)
        // d.clear(si_);
        if (stateProperty_[v] != NULL)
            si_->freeState(stateProperty_[v]);
        stateProperty_[v] = NULL;  // TODO(davetcoleman): is this needed??
    }
    g_.clear();

    if (nn_)
        nn_->clear();

    sampler_.reset();
}

bool og::BoltDB::setup()
{
    if (!sampler_)
        sampler_ = si_->allocValidStateSampler();
    return true;
}

bool og::BoltDB::load(const std::string &fileName)
{
    OMPL_INFORM("BoltDB: load()");

    // Error checking
    if (getNumEdges() || getNumVertices())
    {
        OMPL_INFORM("Database is not empty, unable to load from file");
        return true;
    }
    if (fileName.empty())
    {
        OMPL_ERROR("Empty filename passed to save function");
        return false;
    }
    if (!boost::filesystem::exists(fileName))
    {
        OMPL_INFORM("Database file does not exist: %s.", fileName.c_str());
        return false;
    }

    // Load database from file, track loading time
    time::point start = time::now();

    OMPL_INFORM("Loading database from file: %s", fileName.c_str());

    // Open a binary input stream
    std::ifstream iStream(fileName.c_str(), std::ios::binary);

    // Get the total number of paths saved
    double numPaths = 0;
    iStream >> numPaths;

    // Check that the number of paths makes sense
    if (numPaths < 0 || numPaths > std::numeric_limits<double>::max())
    {
        OMPL_WARN("Number of paths to load %d is a bad value", numPaths);
        return false;
    }

    if (numPaths > 1)
    {
        OMPL_ERROR("Currently more than one planner data is disabled from loading");
        return false;
    }

    // Create a new planner data instance
    ompl::base::PlannerDataPtr plannerData(new ompl::base::PlannerData(si_));

    // Note: the StateStorage class checks if the states match for us
    plannerDataStorage_.load(iStream, *plannerData.get());

    OMPL_INFORM("BoltDB: Loaded planner data with \n  %d vertices\n  %d edges\n  %d start states\n  %d goal states",
        plannerData->numVertices(), plannerData->numEdges(), plannerData->numStartVertices(),
        plannerData->numGoalVertices());

    // Add to db
    OMPL_INFORM("Adding plannerData to database.");
    loadFromPlannerData(*plannerData);

    // Close file
    iStream.close();

    double loadTime = time::seconds(time::now() - start);
    OMPL_INFORM("File load took **** %f sec ****", loadTime);
    return true;
}

bool og::BoltDB::postProcessesPath(og::PathGeometric &solutionPath, double &insertionTime)
{
    // Prevent inserting into database
    if (!savingEnabled_)
    {
        OMPL_WARN("BoltDB: Saving is disabled so not adding path");
        return false;
    }

    bool result = true;
    /*
      double seconds = 120;  // 10; // a large number, should never need to use this
      ompl::base::PlannerTerminationCondition ptc = ompl::base::timedPlannerTerminationCondition(seconds, 0.1);

      // Get starting state
      base::State *currentState = solutionPath.getStates()[0];

      // Find starting state's vertex
      stateProperty_[queryVertex_] = currentState;
      Vertex currentVertex = nn_->nearest(queryVertex_);

      // Walk through whole trajectory
      for (std::size_t i = 1; i < solutionPath.getStates().size(); ++i)
      {
      base::State *nextState = solutionPath.getStates()[i];

      vizStateCallback(currentState, 5, 1);
      vizStateCallback(nextState, 1, 1);
      vizTriggerCallback();
      usleep(1000000);

      // Check if we are close to where a new node is to be expected
      static const double percentSparse = 0.9;  // if within 90%
      double distance = si_->distance(currentState, nextState);
      double distThresh = sparseDelta_ * percentSparse;
      if (distance > distThresh)
      {
      std::cout << "---- FOUND CLOSE ENOUGH " << std::endl;
      // Find the nearest node
      stateProperty_[queryVertex_] = nextState;
      Vertex nearVertex = nn_->nearest(queryVertex_);

      base::State *nearState = stateProperty_[nearVertex];
      vizStateCallback(nearState, 2, 1);
      vizTriggerCallback();
      usleep(1000);

      // Check if path is not obstructed from nearest
      if (!si_->checkMotion(currentState, nearState))
      {
      OMPL_ERROR("Nearest vertex is in collision - have not accounted for this yet");
      }

      // Check if these vertices share an edge
      std::pair<BoltDB::Edge, bool> edge_result = boost::edge(currentVertex, nearVertex, g_);
      if (!edge_result.second)
      {
      OMPL_ERROR("Found two vertices that do not share an edge");
      }

      // reduce cost of this edge because it was just used
      if (popularityBias_)
      {
      static const double REDUCTION_AMOUNT = 10;
      edgeWeightProperty_[edge_result.first] =
      std::max(edgeWeightProperty_[edge_result.first] - REDUCTION_AMOUNT, 0.0);
      }

      // We have a valid near vertex
      // Visualize
      vizEdgeCallback(currentState, nearState, 100);
      vizTriggerCallback();
      usleep(500000);

      // Set the starting state to the new state
      currentState = nearState;
      currentVertex = nearVertex;
      }
      else
      {
      std::cout << "Not close enough: " << distance << " need " << distThresh << std::endl;
      }
      }

      if (popularityBias_)
      {
      OMPL_ERROR("popularity bias enabled");
      // Record this new addition
      graphUnsaved_ = true;
      }
    */
    return result;
}

bool og::BoltDB::saveIfChanged(const std::string &fileName)
{
    if (graphUnsaved_)
    {
        return save(fileName);
    }
    else
        OMPL_INFORM("Not saving because database has not changed");
    return true;
}

bool og::BoltDB::save(const std::string &fileName)
{
    if (!graphUnsaved_)
        OMPL_WARN("No need to save because graphUnsaved_ is false, but saving anyway because requested");

    // Disabled
    if (!savingEnabled_)
    {
        OMPL_WARN("Not saving because option disabled for ExperienceDB");
        return false;
    }

    // Error checking
    if (fileName.empty())
    {
        OMPL_ERROR("Empty filename passed to save function");
        return false;
    }

    // Save database from file, track saving time
    time::point start = time::now();

    OMPL_INFORM("Saving database to file: %s", fileName.c_str());

    // Open a binary output stream
    std::ofstream outStream(fileName.c_str(), std::ios::binary);

    // TODO: make this more than 1 planner data perhaps
    base::PlannerDataPtr data(new base::PlannerData(si_));
    getPlannerData(*data);
    OMPL_INFORM("Saving PlannerData with \n  %d vertices\n  %d edges\n  %d start states\n  %d goal states",
        data->numVertices(), data->numEdges(), data->numStartVertices(), data->numGoalVertices());

    // Write the number of paths we will be saving
    double numPaths = 1;
    outStream << numPaths;

    // Start saving each planner data object
    ompl::base::PlannerData &pd = *data.get();
    OMPL_INFORM("Saving graph with %d vertices and %d edges", pd.numVertices(), pd.numEdges());

    // Debug
    if (false)
        for (std::size_t i = 0; i < pd.numVertices(); ++i)
        {
            OMPL_INFORM("Vertex %d:", i);
            debugVertex(pd.getVertex(i));
        }

    // Save a single planner data
    plannerDataStorage_.store(pd, outStream);

    // Close file
    outStream.close();

    // Benchmark
    double loadTime = time::seconds(time::now() - start);
    OMPL_INFORM("Saved database to file in %f sec", loadTime);

    graphUnsaved_ = false;

    return true;
}

bool og::BoltDB::astarSearch(const Vertex start, const Vertex goal, std::vector<Vertex> &vertexPath)
{
    // Hold a list of the shortest path parent to each vertex
    Vertex *vertexPredecessors = new Vertex[getNumVertices()];
    // boost::vector_property_map<Vertex> vertexPredecessors(getNumVertices());

    bool foundGoal = false;

    double *vertexDistances = new double[getNumVertices()];

    // Error check
    if (getTaskLevel(start) != 0)
    {
        OMPL_ERROR("astarSearch: start level is %u", getTaskLevel(start));
        //exit(-1);
    }
    if (getTaskLevel(goal) != 2)
    {
        OMPL_ERROR("astarSearch: goal level is %u", getTaskLevel(goal));
        //exit(-1);
    }

    OMPL_INFORM("Beginning AStar Search");
    try
    {
        // Note: could not get astar_search to compile within BoltRetrieveRepair.cpp class because of namespacing issues
        boost::astar_search(g_,     // graph
            start,  // start state
            // boost::bind(&og::BoltDB::distanceFunction2, this, _1, goal),  // the heuristic
            //boost::bind(&og::BoltDB::distanceFunction, this, _1, goal),  // the heuristic
            boost::bind(&og::BoltDB::distanceFunctionTasks, this, _1, goal),  // the heuristic
            // ability to disable edges (set cost to inifinity):
            boost::weight_map(edgeWeightMap(g_, edgeCollisionStateProperty_))
            .predecessor_map(vertexPredecessors)
            .distance_map(&vertexDistances[0])
            .visitor(CustomAstarVisitor(goal, this)));
    }
    catch (foundGoalException &)
    {
        // the custom exception from CustomAstarVisitor
        OMPL_INFORM("astarSearch: Astar found goal vertex. distance to goal: %f", vertexDistances[goal]);

        if (vertexDistances[goal] > 1.7e+308)  // TODO(davetcoleman): fix terrible hack for detecting infinity
            // double diff = d[goal] - std::numeric_limits<double>::infinity();
            // if ((diff < std::numeric_limits<double>::epsilon()) && (-diff < std::numeric_limits<double>::epsilon()))
            // check if the distance to goal is inifinity. if so, it is unreachable
            // if (d[goal] >= std::numeric_limits<double>::infinity())
        {
            if (verbose_)
                OMPL_INFORM("Distance to goal is infinity");
            foundGoal = false;
        }
        else
        {
            // Only clear the vertexPath after we know we have a new solution, otherwise it might have a good
            // previous one
            vertexPath.clear();  // remove any old solutions

            // Trace back the shortest path in reverse and only save the states
            Vertex v;
            for (v = goal; v != vertexPredecessors[v]; v = vertexPredecessors[v])
            {
                vertexPath.push_back(v);
            }
            if (v != goal)  // TODO explain this because i don't understand
            {
                vertexPath.push_back(v);
            }

            foundGoal = true;
        }
    }

    if (!foundGoal)
        OMPL_WARN("        Did not find goal");

    // Show all predecessors
    if (visualizeAstar_)
    {
        OMPL_INFORM("        Show all predecessors");
        for (std::size_t i = 1; i < getNumVertices(); ++i)  // skip vertex 0 b/c that is the search vertex
        {
            const Vertex v1 = i;
            const Vertex v2 = vertexPredecessors[v1];
            // std::cout << "Edge " << v1 << " to " << v2 << std::endl;
            vizEdgeCallback(stateProperty_[v1], stateProperty_[v2], 10);
        }
        viz2TriggerCallback();
    }

    // Unload
    delete[] vertexPredecessors;
    delete[] vertexDistances;

    // No solution found from start to goal
    std::cout << "returning foundGoal " << std::endl;
    return foundGoal;
}

void og::BoltDB::getAllPlannerDatas(std::vector<ompl::base::PlannerDataPtr> &plannerDatas) const
{
    base::PlannerDataPtr data(new base::PlannerData(si_));
    getPlannerData(*data);
    plannerDatas.push_back(data);  // TODO(davetcoleman): don't make second copy of this?
}

void og::BoltDB::debugVertex(const ompl::base::PlannerDataVertex &vertex)
{
    debugState(vertex.getState());
}

void og::BoltDB::debugState(const ompl::base::State *state)
{
    si_->printState(state, std::cout);
}

double og::BoltDB::distanceFunction(const Vertex a, const Vertex b) const
{
    // const double dist = si_->distance(stateProperty_[a], stateProperty_[b]);
    // std::cout << "getting distance from " << a << " to " << b << " of value " << dist << std::endl;
    // return dist;
    return si_->distance(stateProperty_[a], stateProperty_[b]);
}

double og::BoltDB::distanceFunction2(const Vertex a, const Vertex b) const
{
    // const double dist = si_->getStateSpace()->distance2(stateProperty_[a], stateProperty_[b]);
    // std::cout << "getting distance from " << a << " to " << b << " of value " << dist << std::endl;
    // return dist;
    return si_->getStateSpace()->distance2(stateProperty_[a], stateProperty_[b]);
}

double og::BoltDB::distanceFunctionTasks(const Vertex a, const Vertex b) const
{
  const bool verbose = false;
    // Reorder a & b so that we are sure that a.level <= b.level
    std::size_t taskLevelA = getTaskLevel(a);
    std::size_t taskLevelB = getTaskLevel(b);
    if (taskLevelA > taskLevelB)
    {
        // Call itself again, this time switching ordering
      if (verbose)
        OMPL_INFORM("Switched ordering for distanceFunctionTasks()");
        return distanceFunctionTasks(b, a);
    }

    double dist; // the result
    static const double TASK_LEVEL_COST = 1.0; // cost to change levels/tasks

    // Error check
    assert(stateProperty_[a]);
    assert(stateProperty_[b]);
    assert(stateProperty_[startConnectorVertex_]);
    assert(stateProperty_[endConnectorVertex_]);

    if (taskLevelA == 0)
    {
        if (taskLevelB == 0) // regular distance for bottom level
        {
      if (verbose)
            std::cout << "a ";
            dist = si_->distance(stateProperty_[a], stateProperty_[b]);
        }
        else if (taskLevelB == 1)
        {
      if (verbose)
            std::cout << "b ";
            dist = si_->distance(stateProperty_[a], stateProperty_[startConnectorVertex_])
                + TASK_LEVEL_COST
                + si_->distance(stateProperty_[startConnectorVertex_], stateProperty_[b]);
        }
        else if (taskLevelB == 2)
        {
      if (verbose)
            std::cout << "c ";
            dist = si_->distance(stateProperty_[a], stateProperty_[startConnectorVertex_])
                + TASK_LEVEL_COST
                + distanceAcrossCartesian_
                + TASK_LEVEL_COST
                + si_->distance(stateProperty_[endConnectorVertex_], stateProperty_[b]);
        }
        else
        {
            OMPL_ERROR("Unknown task level mode");
            exit(-1);
        }
    }
    else if (taskLevelA == 1)
    {
        if (taskLevelB == 0)
        {
            OMPL_ERROR("Unknown task level mode");
            exit(-1);
        }
        else if (taskLevelB == 1)
        {
      if (verbose)
            std::cout << "d ";
            dist = si_->distance(stateProperty_[a], stateProperty_[b]);
        }
        else if (taskLevelB == 2)
        {
      if (verbose)
            std::cout << "e ";
            dist = si_->distance(stateProperty_[a], stateProperty_[endConnectorVertex_])
                + TASK_LEVEL_COST
                + si_->distance(stateProperty_[endConnectorVertex_], stateProperty_[b]);
        }
        else
        {
            OMPL_WARN("Unknown task level mode");
        }
    }
    else if (taskLevelA == 2)
    {
        if (taskLevelB == 0 || taskLevelB == 1)
        {
            OMPL_ERROR("Unknown task level mode");
            exit(-1);
        }
        else if (taskLevelB == 2)
        {
      if (verbose)
            std::cout << "f ";
            dist = si_->distance(stateProperty_[a], stateProperty_[b]);
        }
        else
        {
            OMPL_WARN("Unknown task level mode");
        }
    }
    else
    {
        OMPL_WARN("Unknown task level mode");
    }

      if (verbose)
    std::cout << "State " << a << " @level " << taskLevelA << " to State " << b << " @level " << taskLevelB
              <<  " has distance " << dist << std::endl;
    return dist;
}

std::size_t og::BoltDB::getTaskLevel(const Vertex &v) const
{
    return si_->getStateSpace()->getLevel(stateProperty_[v]);
}

std::size_t og::BoltDB::getTaskLevel(const base::State* state) const
{
    return si_->getStateSpace()->getLevel(state);
}

void og::BoltDB::initializeQueryState()
{
    if (boost::num_vertices(g_) < 1)
    {
        queryVertex_ = boost::add_vertex(g_);
        stateProperty_[queryVertex_] = NULL;
    }
}

void og::BoltDB::getPlannerData(base::PlannerData &data) const
{
    // If there are even edges here
    if (boost::num_edges(g_) > 0)
    {
        // Adding edges and all other vertices simultaneously
        BOOST_FOREACH (const Edge e, boost::edges(g_))
        {
            const Vertex v1 = boost::source(e, g_);
            const Vertex v2 = boost::target(e, g_);

            if (!data.addEdge(base::PlannerDataVertex(stateProperty_[v1], (int)typeProperty_[v1]),
                    base::PlannerDataVertex(stateProperty_[v2], (int)typeProperty_[v2]),
                    base::PlannerDataEdge(), base::Cost(edgeWeightProperty_[e])))
            {
                OMPL_ERROR("Unable to add edge");
            }

            // OMPL_INFORM("Adding edge from vertex of type %d to vertex of type %d", typeProperty_[v1],
            // typeProperty_[v2]);
        }
    }
    else
        OMPL_ERROR("There are no edges in the graph!");

    // Make sure to add edge-less nodes as well
    BOOST_FOREACH (const Vertex n, boost::vertices(g_))
        if (boost::out_degree(n, g_) == 0)
            data.addVertex(base::PlannerDataVertex(stateProperty_[n], (int)typeProperty_[n]));

    // data.properties["iterations INTEGER"] = boost::lexical_cast<std::string>(iterations_);
}

void og::BoltDB::loadFromPlannerData(const base::PlannerData &data)
{
    // Check that the query vertex is initialized (later used for internal nearest neighbor searches)
    initializeQueryState();

    // Add all vertices
    OMPL_INFORM("  Loading %u vertices into BoltDB", data.numVertices());

    std::vector<Vertex> idToVertex;

    // Temp disable verbose mode for loading database
    bool wasVerbose = verbose_;
    verbose_ = false;
    std::size_t debugFrequency = data.numVertices() / 10;

    // Add the nodes to the graph
    for (std::size_t vertexID = 0; vertexID < data.numVertices(); ++vertexID)
    {
        if (vertexID % debugFrequency == 0)
            OMPL_INFORM("    %f percent loaded", vertexID / double(data.numVertices()) * 100.0);

        // Get the state from loaded planner data
        const base::State *oldState = data.getVertex(vertexID).getState();
        base::State *state = si_->cloneState(oldState);

        // Get the tag, which in this application represents the vertex type
        GuardType type = static_cast<GuardType>(data.getVertex(vertexID).getTag());

        // ADD GUARD
        idToVertex.push_back(addVertex(state, type));
    }

    OMPL_INFORM("  Loading %u edges into BoltDB", data.numEdges());
    // Add the corresponding edges to the graph
    std::vector<unsigned int> edgeList;
    for (unsigned int fromVertex = 0; fromVertex < data.numVertices(); ++fromVertex)
    {
        if (fromVertex % debugFrequency == 0)
            OMPL_INFORM("    %f percent loaded", fromVertex / double(data.numVertices()) * 100.0);

        edgeList.clear();

        // Get the edges
        data.getEdges(fromVertex, edgeList);  // returns the id of each edge

        Vertex v1 = idToVertex[fromVertex];

        // Process edges
        for (std::size_t edgeId = 0; edgeId < edgeList.size(); ++edgeId)
        {
            unsigned int toVertex = edgeList[edgeId];
            Vertex v2 = idToVertex[toVertex];

            // Add the edge to the graph
            if (verbose_ && false)
            {
                OMPL_INFORM("    Adding edge from vertex id %d to id %d into edgeList", fromVertex, toVertex);
                OMPL_INFORM("      Vertex %d to %d", v1, v2);
            }

            base::Cost weight;
            if (!data.getEdgeWeight(fromVertex, toVertex, &weight))
            {
                OMPL_ERROR("Unable to get edge weight");
            }

            addEdge(v1, v2, weight.value());
        }
    }  // for
    OMPL_INFORM("  Finished loading %u edges", getNumEdges());

    // OMPL_WARN("temp save when load graph from file");
    // graphUnsaved_ = true;

    // Re-enable verbose mode, if necessary
    verbose_ = wasVerbose;

    // Clone the graph to have second and third layers for task planning then free space planning
    generateTaskSpace();
}

void og::BoltDB::generateGrid()
{
    OMPL_INFORM("Generating grid");

    if (!si_->isSetup())
    {
        OMPL_WARN("Space information setup was not yet called. Calling now.");
        si_->setup();
    }

    // Check that the query vertex is initialized (used for internal nearest neighbor searches)
    initializeQueryState();

    // Prepare for recursion
    // TODO(davetcoleman): currently the last joint is not being discretized, so we should set its
    // default value smartly and not just '0'
    std::vector<double> values(si_->getStateSpace()->getDimension(), 0);

    // Choose first state to discretize
    nextDiscretizedState_ =
        si_->getStateSpace()->allocState();  // Note: it is currently possible the last state is never freed

    const std::size_t starting_joint_id = 0;
    std::size_t desired_depth = si_->getStateSpace()->getDimension();

    // TODO: This is a custom dimensionality reduction hack, that maybe should not be in this location
    if (desired_depth > 5)
    {
        // OMPL_INFORM("Truncated discretization depth to 5");
        desired_depth = 6;
    }
    else if (desired_depth == 3)
    {
        // This is for the 2D case where the third dimension is task
        OMPL_INFORM("Truncated discretization depth to 2");
        desired_depth = 2;

        // Set all generated states to a level 0 task space
        values[2] = 0;
    }

    // Create vertices
    recursiveDiscretization(values, starting_joint_id, desired_depth);
    OMPL_INFORM("Generated %i vertices.", getNumVertices());

    // Remove vertices in collision using multithreading
    // TODO enable
    // std::vector<Vertex> unvalidatedVertices;
    // checkVerticesThreaded(unvalidatedVertices);

    {
        // Benchmark runtime
        time::point start_time = time::now();

        // Create edges
        generateEdges();

        // Benchmark runtime
        double duration = time::seconds(time::now() - start_time);
        OMPL_INFORM("Generate edges total time: %f seconds (%f hz)", duration, 1.0 / duration);
    }

    // Get the average vertex degree (number of connected edges)
    std::size_t average_degree = (getNumEdges() * 2) / getNumVertices();
    OMPL_INFORM("Average degree: %i", average_degree);

    // Display
    if (visualizeGridGeneration_)
        viz2TriggerCallback();

    // Mark the graph as ready to be saved
    graphUnsaved_ = true;

    // Optionally add task space
    if (si_->getStateSpace()->getDimension() == 3)
        generateTaskSpace();
}

void og::BoltDB::generateTaskSpace()
{
    OMPL_INFORM("Generating task space");
    std::vector<Vertex> vertexToNewVertex(getNumVertices());

    OMPL_INFORM("Adding task space vertices");
    BOOST_FOREACH (Vertex v, boost::vertices(g_))
    {
        if (!stateProperty_[v])
        {
            OMPL_INFORM("Skipping state copy because has value NULL at %u", v);
            continue;
        }

        // Clone the state
        base::State *newState = si_->cloneState(stateProperty_[v]);

        const std::size_t level = 2;
        si_->getStateSpace()->setLevel(newState, level);

        // Add the state back
        Vertex vNew = addVertex(newState, START);

        // Map old vertex to new vertex
        vertexToNewVertex[v] = vNew;

        // Visualize - only do this for 2/3D environments
        if (visualizeGridGeneration_)
        {
            viz2StateCallback(newState, 5, 1);  // Candidate node has already (just) been added
            viz2TriggerCallback();
            usleep(0.005 * 1000000);
        }
    }

    // Add Edges
    OMPL_INFORM("Adding task space edges");

    // Cache all current edges before adding new ones
    std::vector<Edge> edges(getNumEdges());
    std::size_t edgeID = 0;
    BOOST_FOREACH (const Edge e, boost::edges(g_))
    {
        edges[edgeID++] = e;
    }

    // Copy every edge
    for (std::size_t i = 0; i < edges.size(); ++i)
    {
        const Edge &e = edges[i];

        const Vertex v1 = boost::source(e, g_);
        const Vertex v2 = boost::target(e, g_);

        // std::cout << "v1: " << v1 << " v2: " << v2 << std::endl;
        // std::cout << "v1': " << vertexToNewVertex[v1] << " v2': " << vertexToNewVertex[v2] << std::endl;

        Edge newE = addEdge(vertexToNewVertex[v1], vertexToNewVertex[v2], edgeWeightProperty_[e]);

        // Visualize
        if (visualizeGridGeneration_)
        {
            viz2Edge(newE);
            if (i % 100 == 0)
            {
                viz2TriggerCallback();
                usleep(0.01 * 1000000);
            }
        }
    }

    // Visualize
    if (visualizeGridGeneration_)
        viz2TriggerCallback();

    OMPL_INFORM("Done generating task space graph");
}

void og::BoltDB::generateEdges()
{
    OMPL_INFORM("Generating edges");

    bool verbose = false;

    // Benchmark runtime
    time::point startTime = time::now();
    std::size_t count = 0;

    // Nearest Neighbor search
    std::vector<Vertex> graphNeighborhood;
    std::vector<Edge> unvalidatedEdges;
    std::size_t feedbackFrequency = getNumVertices() / 10;

    // Loop through each vertex
    for (std::size_t v1 = 1; v1 < getNumVertices(); ++v1)  // 1 because 0 is the search vertex?
    {
        if (v1 % feedbackFrequency == 0)
        {
            // Benchmark runtime
            double duration = time::seconds(time::now() - startTime);
            std::cout << "Edge generation progress: " << double(v1) / getNumVertices() * 100.0 << " % "
                      << "Total edges: " << count << " Total time: " << duration << std::endl;
            startTime = time::now();
        }

        // Add edges
        graphNeighborhood.clear();
        base::State *state = stateProperty_[v1];
        stateProperty_[queryVertex_] = state;

        //{
        // Benchmark runtime
        // time::point startTime = time::now();

        // in 2D this created the regular square with diagonals of 8 edges

        std::size_t findNearestKNeighbors;
        if (si_->getStateSpace()->getDimension() == 3)
        {
            findNearestKNeighbors = 8;
        }
        else  // full robot
        {
            findNearestKNeighbors = si_->getStateSpace()->getDimension() * 2;
        }
        // OMPL_INFORM("Finding %u nearest neighbors for each vertex", findNearestKNeighbors);

        const std::size_t numSameVerticiesFound = 1;  // add 1 to the end because the NN tree always returns itself
        nn_->nearestK(queryVertex_, findNearestKNeighbors + numSameVerticiesFound, graphNeighborhood);
        stateProperty_[queryVertex_] = NULL;
        if (verbose)
            OMPL_INFORM("Found %u neighbors", graphNeighborhood.size());

        // Benchmark runtime
        // double duration = time::seconds(time::now() - startTime) * 1000;
        // OMPL_INFORM("NN Total time: %f milliseconds (%f hz)", duration, 1.0 / duration);

        // Clear all visuals
        // if (visualizeGridGeneration_)
        //     viz2StateCallback(stateProperty_[v1], 0, 1);

        // For each nearby vertex, add an edge
        std::size_t errorCheckNumSameVerticies = 0;
        for (std::size_t i = 0; i < graphNeighborhood.size(); ++i)
        {
            if (verbose)
                OMPL_INFORM("Edge %u", i);

            Vertex &v2 = graphNeighborhood[i];

            // Check if these vertices are the same
            if (v1 == v2)
            {
                errorCheckNumSameVerticies++;
                continue;
            }

            // Debug: display edge
            if (visualizeGridGeneration_)
                viz2EdgeCallback(stateProperty_[v1], stateProperty_[v2], 1);

            // Check if these vertices already share an edge
            {
                // Benchmark runtime
                time::point startTime = time::now();

                if (boost::edge(v1, v2, g_).second)
                    continue;

                // Benchmark runtime
                double duration = time::seconds(time::now() - startTime) * 1000;
                if (verbose)
                    OMPL_INFORM("Check edge exist Total time: %f milliseconds (%f hz)", duration, 1.0 / duration);
            }

            // Create edge - maybe removed later
            Edge e = addEdge(v1, v2, 0);
            unvalidatedEdges.push_back(e);

            count++;
        }  // for each v2

        // Make sure one and only one vertex is returned from the NN search that is the same as parent vertex
        assert(errorCheckNumSameVerticies == 1);

        if (visualizeGridGeneration_)
        {
            viz2TriggerCallback();
            usleep(0.01 * 1000000);
        }

    }  // for each v1

    OMPL_INFORM("Generated %i edges. Finished generating grid.", getNumEdges());

    // Benchmark runtime
    time::point startTime3 = time::now();
    std::size_t numEdgesBeforeCheck = getNumEdges();

    // Collision check all edges using threading
    checkEdgesThreaded(unvalidatedEdges);

    // Calculate statistics on collision state of robot
    std::size_t numEdgesAfterCheck = getNumEdges();
    std::size_t numRemovedEdges = numEdgesBeforeCheck - numEdgesAfterCheck;
    double duration3 = time::seconds(time::now() - startTime3);

    // User feedback
    OMPL_INFORM("Collision State of Robot -----------------");
    OMPL_INFORM("   Removed Edges:        %u", numRemovedEdges);
    OMPL_INFORM("   Remaining Edges:      %u", numEdgesAfterCheck);
    OMPL_INFORM("   Percent Removed:      %f %%", numRemovedEdges / double(numEdgesBeforeCheck) * 100.0);
    OMPL_INFORM("   Total time:           %f sec (%f hz)", duration3, 1.0 / duration3);

    // Calculate cost for each edge
    std::size_t errorCheckCounter = 0;
    BOOST_FOREACH (const Edge e, boost::edges(g_))
    {
        const Vertex &v1 = boost::source(e, g_);
        const Vertex &v2 = boost::target(e, g_);

        // Determine cost for edge depending on mode
        double cost;
        if (popularityBias_)
        {
            cost = 100;
        }
        else
        {
            // cost = distanceFunction2(v1, v2);
            cost = distanceFunction(v1, v2);
        }
        edgeWeightProperty_[e] = cost;
        errorCheckCounter++;

        // Debug in Rviz
        if (visualizeGridGeneration_)
        {
            viz2EdgeCallback(stateProperty_[v1], stateProperty_[v2], edgeWeightProperty_[e]);
            if (errorCheckCounter % 100 == 0)
            {
                viz2TriggerCallback();
                usleep(0.01 * 1000000);
            }
        }
    }
    assert(errorCheckCounter == numEdgesAfterCheck);
}

void og::BoltDB::checkEdges()
{
    OMPL_WARN("Collision checking generated edges without threads");
    OMPL_ERROR("Has not been updated for remove_edge");

    BOOST_FOREACH (const Edge e, boost::edges(g_))
    {
        const Vertex &v1 = boost::source(e, g_);
        const Vertex &v2 = boost::target(e, g_);

        // Remove any edges that are in collision
        if (!si_->checkMotion(stateProperty_[v1], stateProperty_[v2]))
        {
            edgeCollisionStateProperty_[e] = BoltDB::IN_COLLISION;
        }
        else
        {
            edgeCollisionStateProperty_[e] = BoltDB::FREE;
        }
    }
}

void og::BoltDB::checkEdgesThreaded(const std::vector<Edge> &unvalidatedEdges)
{
    const bool verbose = false;

    // Error check
    assert(unvalidatedEdges.size() == getNumEdges());

    // Setup threading
    static const std::size_t numThreads = boost::thread::hardware_concurrency();
    OMPL_INFORM("Collision checking %u generated edges using %u threads", unvalidatedEdges.size(), numThreads);

    std::vector<boost::thread *> threads(numThreads);
    std::size_t numEdges = getNumEdges();  // we copy this number, because it might start shrinking when threads spin up
    std::size_t edgesPerThread = numEdges / numThreads;  // rounds down
    std::size_t startEdge = 0;
    std::size_t endEdge;
    std::size_t errorCheckTotalEdges = 0;

    // For each thread
    for (std::size_t i = 0; i < threads.size(); ++i)
    {
        endEdge = startEdge + edgesPerThread - 1;

        // Check if this is the last thread
        if (i == threads.size() - 1)
        {
            // have it do remaining edges to check
            endEdge = numEdges - 1;
        }
        errorCheckTotalEdges += (endEdge - startEdge);

        if (verbose)
            std::cout << "Thread " << i << " has edges from " << startEdge << " to " << endEdge << std::endl;

        base::SpaceInformationPtr si(new base::SpaceInformation(si_->getStateSpace()));
        si->setStateValidityChecker(si_->getStateValidityChecker());
        si->setMotionValidator(si_->getMotionValidator());

        threads[i] = new boost::thread(
            boost::bind(&og::BoltDB::checkEdgesThread, this, startEdge, endEdge, si, unvalidatedEdges));
        startEdge += edgesPerThread;
    }

    // Join threads
    for (std::size_t i = 0; i < threads.size(); ++i)
    {
        threads[i]->join();
        delete threads[i];
    }

    // Error check
    if (errorCheckTotalEdges == getNumEdges())
    {
        OMPL_ERROR("Incorrect number of edges were processed");
        exit(-1);
    }

    // Make sure all remaining edges were validated
    BOOST_FOREACH (const Edge e, boost::edges(g_))
    {
        if (edgeCollisionStateProperty_[e] != BoltDB::FREE)
        {
            OMPL_ERROR("Remaining edge %u has not been marked free", e);
        }
    }
}

void og::BoltDB::checkEdgesThread(std::size_t startEdge, std::size_t endEdge, base::SpaceInformationPtr si,
    const std::vector<Edge> &unvalidatedEdges)
{
    // Process [startEdge, endEdge] inclusive
    for (std::size_t edgeID = startEdge; edgeID <= endEdge; ++edgeID)
    {
        const Edge &e = unvalidatedEdges[edgeID];

        const Vertex &v1 = boost::source(e, g_);
        const Vertex &v2 = boost::target(e, g_);

        // Remove any edges that are in collision
        if (!si->checkMotion(stateProperty_[v1], stateProperty_[v2]))
        {
            boost::remove_edge(v1, v2, g_);
        }
        else
        {
            edgeCollisionStateProperty_[e] = BoltDB::FREE;
        }
    }
}

void og::BoltDB::recursiveDiscretization(std::vector<double> &values, std::size_t joint_id, std::size_t desired_depth)
{
    ob::RealVectorBounds bounds = si_->getStateSpace()->getBounds();

    // Error check
    assert(bounds.high.size() == bounds.low.size());
    assert(bounds.high.size() == si_->getStateSpace()->getDimension());
    assert(joint_id < values.size());

    // Loop thorugh current joint
    for (double value = bounds.low[joint_id]; value <= bounds.high[joint_id]; value += sparseDelta_)
    {
        values[joint_id] = value;

        // User feedback
        if (joint_id == 0)
        {
            const double percent =
                (value - bounds.low[joint_id]) / (bounds.high[joint_id] - bounds.low[joint_id]) * 100.0;
            std::cout << "Vertex generation progress: " << percent << " % Total vertices: " << getNumVertices()
                      << std::endl;
        }

        // Check if we are at the end of the recursion
        if (joint_id < desired_depth - 1)
        {
            // Keep recursing
            recursiveDiscretization(values, joint_id + 1, desired_depth);
        }
        else  // this is the end of recursion, create a new state
        {
            // TODO(davetcoleman): way to not allocState if in collision?
            nextDiscretizedState_ = si_->getStateSpace()->allocState();

            // Fill the state with current values
            si_->getStateSpace()->populateState(nextDiscretizedState_, values);

            // Collision check
            if (!si_->isValid(nextDiscretizedState_))
            {
                // OMPL_ERROR("Found a state that is not valid! ");
                continue;
            }

            // Add vertex to graph
            GuardType type = START;  // TODO(davetcoleman): type START is dummy
            addVertex(nextDiscretizedState_, type);

            // Visualize
            if (visualizeGridGeneration_)
            {
                viz2StateCallback(nextDiscretizedState_, 5, 1);  // Candidate node has already (just) been added
                viz2TriggerCallback();
                usleep(0.005 * 1000000);
            }

            // Prepare for next new state by allocating now
            // nextDiscretizedState_ = si_->getStateSpace()->allocState();
        }
    }
}

void og::BoltDB::checkVerticesThreaded(const std::vector<Vertex> &unvalidatedVertices)
{
    // TODO(davetcoleman): have not tested this functionality
    bool verbose = true;

    // Setup threading
    static const std::size_t numThreads = boost::thread::hardware_concurrency();
    OMPL_INFORM("Collision checking %u generated vertices using %u threads", unvalidatedVertices.size(), numThreads);

    std::vector<boost::thread *> threads(numThreads);
    std::size_t verticesPerThread = getNumVertices() / numThreads;  // rounds down
    std::size_t startVertex = 0;
    std::size_t endVertex;
    std::size_t errorCheckTotalVertices = 0;

    // For each thread
    for (std::size_t i = 0; i < threads.size(); ++i)
    {
        endVertex = startVertex + verticesPerThread - 1;

        // Check if this is the last thread
        if (i == threads.size() - 1)
        {
            // have it do remaining vertices to check
            endVertex = getNumVertices() - 1;
        }
        errorCheckTotalVertices += (endVertex - startVertex);

        if (verbose)
            std::cout << "Thread " << i << " has vertices from " << startVertex << " to " << endVertex << std::endl;

        base::SpaceInformationPtr si(new base::SpaceInformation(si_->getStateSpace()));
        si->setStateValidityChecker(si_->getStateValidityChecker());
        // si->setMotionValidator(si_->getMotionValidator());

        threads[i] = new boost::thread(
            boost::bind(&og::BoltDB::checkVerticesThread, this, startVertex, endVertex, si, unvalidatedVertices));
        startVertex += verticesPerThread;
    }

    // Join threads
    for (std::size_t i = 0; i < threads.size(); ++i)
    {
        threads[i]->join();
        delete threads[i];
    }

    // Error check
    if (errorCheckTotalVertices == getNumVertices())
    {
        OMPL_ERROR("Incorrect number of vertices were processed");
        exit(-1);
    }
}

void og::BoltDB::checkVerticesThread(std::size_t startVertex, std::size_t endVertex, base::SpaceInformationPtr si,
    const std::vector<Vertex> &unvalidatedVertices)
{
    // Process [startVertex, endVertex] inclusive
    for (std::size_t vertexID = startVertex; vertexID <= endVertex; ++vertexID)
    {
        const Vertex &v = unvalidatedVertices[vertexID];

        // Remove any vertices that are in collision
        if (!si_->isValid(stateProperty_[v]))
        {
            boost::remove_vertex(v, g_);
            std::cout << "found vertex in collision " << std::endl;
        }
    }
}

void og::BoltDB::clearEdgeCollisionStates()
{
    BOOST_FOREACH (const Edge e, boost::edges(g_))
        edgeCollisionStateProperty_[e] = NOT_CHECKED;  // each edge has an unknown state
}

void og::BoltDB::displayDatabase()
{
    OMPL_INFORM("Displaying database");

    /*
    // Loop through each vertex
    std::size_t count = 0;
    std::size_t debugFrequency = getNumVertices() / 10;
    BOOST_FOREACH (Vertex v, boost::vertices(g_))
    {
        // Check for null states
        if (stateProperty_[v])
        {
            viz2StateCallback(stateProperty_[v], 6, 1);
        }

        // Prevent cache from getting too big
        if (count % debugFrequency == 0)
        {
            OMPL_INFORM("Displaying %f %% of database", (static_cast<double>(count+1) / getNumVertices()) * 100.0);
            viz2TriggerCallback();
        }
        count++;
    }
*/
    // Loop through each edge
    std::size_t count = 0;
    std::size_t debugFrequency = getNumEdges() / 20;
    BOOST_FOREACH (Edge e, boost::edges(g_))
    {
        // Add edge
        const Vertex &v1 = boost::source(e, g_);
        const Vertex &v2 = boost::target(e, g_);

        // Visualize
        viz2EdgeCallback(stateProperty_[v1], stateProperty_[v2], 110);

        // Prevent cache from getting too big
        if (count % debugFrequency == 0)
        {
            OMPL_INFORM("Displaying %f %% of database", (static_cast<double>(count+1) / getNumEdges()) * 100.0);
            viz2TriggerCallback();
        }

        count++;
    }


    // Publish remaining edges
    viz2TriggerCallback();
}

void og::BoltDB::setVizCallbacks(ompl::base::VizStateCallback vizStateCallback,
    ompl::base::VizEdgeCallback vizEdgeCallback,
    ompl::base::VizTriggerCallback vizTriggerCallback)
{
    vizStateCallback_ = vizStateCallback;
    vizEdgeCallback_ = vizEdgeCallback;
    vizTriggerCallback_ = vizTriggerCallback;
}

void og::BoltDB::setViz2Callbacks(ompl::base::VizStateCallback vizStateCallback,
    ompl::base::VizEdgeCallback vizEdgeCallback,
    ompl::base::VizTriggerCallback vizTriggerCallback)
{
    viz2StateCallback_ = vizStateCallback;
    viz2EdgeCallback_ = vizEdgeCallback;
    viz2TriggerCallback_ = vizTriggerCallback;
}

void og::BoltDB::normalizeGraphEdgeWeights()
{
    if (!popularityBias_)
    {
        OMPL_INFORM("Skipping normalize graph edge weights because not using popularity bias currently");
        return;
    }

    // Normalize weight of graph
    double total_cost = 0;
    BOOST_FOREACH (Edge e, boost::edges(g_))
    {
        total_cost += edgeWeightProperty_[e];
    }
    double avg_cost = total_cost / getNumEdges();
    OMPL_INFORM("Average cost of the edges in graph is %f", avg_cost);
    double desired_avg_cost = 90;

    if (avg_cost < desired_avg_cost)  // need to increase cost in graph
    {
        double avg_cost_diff = desired_avg_cost - avg_cost;
        std::cout << "avg_cost_diff: " << avg_cost_diff << std::endl;
        double perEdge_reduction = avg_cost_diff;  // / getNumEdges();
        OMPL_INFORM("Increasing each edge's cost by %f", perEdge_reduction);
        BOOST_FOREACH (Edge e, boost::edges(g_))
        {
            edgeWeightProperty_[e] = std::min(edgeWeightProperty_[e] + perEdge_reduction, 100.0);
        }
    }
    else
    {
        OMPL_INFORM("Not increasing all edge's cost because average is above desired");
    }
}

og::BoltDB::Vertex og::BoltDB::addVertex(base::State *state, const GuardType &type)
{
    // Create vertex
    Vertex v1 = boost::add_vertex(g_);

    // Add properties
    typeProperty_[v1] = type;
    stateProperty_[v1] = state;

    // Add vertex to nearest neighbor structure
    nn_->add(v1);

    // Track vertex for later removal if temporary
    if (type == CARTESIAN)
    {
        tempVerticies_.push_back(v1);
    }

    return v1;
}

og::BoltDB::Edge og::BoltDB::addEdge(const Vertex &v1, const Vertex &v2, const double weight)
{
    // Error check
    assert(v2 <= getNumVertices());
    assert(v1 <= getNumVertices());

    // Create the new edge
    Edge e = (boost::add_edge(v1, v2, g_)).first;

    // std::cout << "Adding cost: " << weight << std::endl;

    // Add associated properties to the edge
    edgeWeightProperty_[e] = weight;
    // edgeWeightProperty_[e] = distanceFunction2(v1, v2);
    // edgeWeightProperty_[e] = distanceFunction(v1, v2);
    edgeCollisionStateProperty_[e] = NOT_CHECKED;

    return e;
}

void og::BoltDB::cleanupTemporaryVerticies()
{
    const bool verbose = false;

    if (tempVerticies_.empty())
    {
        OMPL_INFORM("Skipping verticies cleanup - no old middle cartesian layer verticies found");
        return;
    }

    OMPL_INFORM("Cleaning up temp verticies - vertex count: %u, edge count: %u", getNumVertices(), getNumEdges());
    BOOST_REVERSE_FOREACH(Vertex v, tempVerticies_)
    {
        if (verbose)
            std::cout << "removing vertex " << v << std::endl;
        if (verbose)
            std::cout << "remove from nearest neighbor " << std::endl;
        nn_->remove(v);
        if (verbose)
            std::cout << "free state " << std::endl;
        si_->freeState(stateProperty_[v]);
        if (verbose)
            std::cout << "setting to null " << std::endl;
        stateProperty_[v] = NULL;
        if (verbose)
            std::cout << "clear vertex" << std::endl;
        // Remove all edges to and from vertex
        boost::clear_vertex(v, g_);
        if (verbose)
            std::cout << "remove vertex " << std::endl;
        // Remove vertex
        boost::remove_vertex(v, g_);
        if (verbose)
            OMPL_DEBUG("Removed, updated - vertex count: %u, edge count: %u", getNumVertices(), getNumEdges());
    }
    tempVerticies_.clear();
    OMPL_INFORM("Finished cleaning up temp verticies");
}

bool og::BoltDB::addCartPath(std::vector<base::State *> path)
{
    // TODO: check for validity

    // Create verticies for the extremas
    Vertex startVertex = addVertex(path.front(), CARTESIAN);
    Vertex goalVertex = addVertex(path.back(), CARTESIAN);

    // Connect Start to graph --------------------------------------
    std::cout << "  start connector ---------" << std::endl;
    const std::size_t level0 = 0;
    if (!connectStateToNeighborsAtLevel(startVertex, level0, startConnectorVertex_))
    {
        OMPL_ERROR("Failed to connect start of cartesian path");
        return false;
    }

    // Record min cost for cost-to-go heurstic distance function later
    distanceAcrossCartesian_ = distanceFunction(startVertex, goalVertex);

    // Connect goal to graph --------------------------------------
    std::cout << "  goal connector ----------------" << std::endl;
    const std::size_t level2 = 2;
    if (!connectStateToNeighborsAtLevel(goalVertex, level2, endConnectorVertex_))
    {
        OMPL_ERROR("Failed to connect goal of cartesian path");
        return false;
    }

    // Add cartesian path to mid level graph --------------------
    Vertex v1 = startVertex;
    Vertex v2;
    for (std::size_t i = 1; i < path.size(); ++i)
    {
        // Check if we are on the goal vertex
        if (i == path.size() - 1)
        {
            v2 = goalVertex;  // Do not create the goal vertex twice
        }
        else
        {
            v2 = addVertex(path[i], CARTESIAN);
        }
        double cost = distanceFunction(v1, v2);
        addEdge(v1, v2, cost);
        v1 = v2;

        // Visualize
        if (visualizeCartPath_)
        {
            vizEdgeCallback(path[i - 1], path[i], 0);
            vizStateCallback(path[i], 1, 1);
            vizTriggerCallback();
            usleep(0.1 * 1000000);
        }
    }

    return true;
}

bool og::BoltDB::connectStateToNeighborsAtLevel(const Vertex &fromVertex, const std::size_t level,
    Vertex &minConnectorVertex)
{
    // Get nearby states to goal
    std::vector<Vertex> neighbors;
    const std::size_t kNeighbors = 20;
    getNeighborsAtLevel(stateProperty_[fromVertex], level, kNeighbors, neighbors);

    // Error check
    if (neighbors.empty())
    {
        OMPL_ERROR("No neighbors found when connecting cartesian path");
        return false;
    }
    else
        OMPL_INFORM("Found %u neighbors on level %u", neighbors.size(), level);

    // Find the shortest connector out of all the options
    double minConnectorCost = std::numeric_limits<double>::infinity();

    // Loop through each neighbor
    BOOST_FOREACH (Vertex v, neighbors)
    {
        // Add edge from nearby graph vertex to cart path goal
        double connectorCost = distanceFunction(fromVertex, v);
        addEdge(fromVertex, v, connectorCost);

        // Get min cost connector
        if (connectorCost < minConnectorCost)
        {
            minConnectorCost = connectorCost; // TODO(davetcoleman): should we save the cost, or just use 1.0?
            minConnectorVertex = v;
        }

        // Visualize connection to goal of cartesian path
        const double cost = (level == 0 ? 100 : 50);
        if (visualizeCartNeighbors_)
        {
            vizEdgeCallback(stateProperty_[v], stateProperty_[fromVertex], cost);
            vizStateCallback(stateProperty_[v], 1, 1);
            vizTriggerCallback();
            usleep(1.0 * 1000000);
        }
    }

    // Display ---------------------------------------
    if (visualizeCartNeighbors_)
        vizTriggerCallback();

    return true;
}

void og::BoltDB::getNeighborsAtLevel(const base::State *origState, const std::size_t level,
    const std::size_t kNeighbors, std::vector<Vertex> &neighbors)
{
    // Clone the state and change its level
    base::State *searchState = si_->cloneState(origState);
    si_->getStateSpace()->setLevel(searchState, level);

    // Get nearby state
    stateProperty_[queryVertex_] = searchState;
    nn_->nearestK(queryVertex_, kNeighbors, neighbors);

    // Cleanup
    stateProperty_[queryVertex_] = NULL;
    si_->getStateSpace()->freeState(searchState);

    // Run various checks
    for (std::size_t i = 0; i < neighbors.size(); ++i)
    {
        const Vertex &nearVertex = neighbors[i];

        // Make sure state is on correct level
        if (getTaskLevel(nearVertex) != level)
        {
            std::cout << "      Skipping neighbor " << nearVertex << ", i=" << i
                      << ", because wrong level: " << getTaskLevel(nearVertex) << ", desired level: " << level
                      << std::endl;
            neighbors.erase(neighbors.begin() + i);
            i--;
            continue;
        }

        // Collision check
        if (!si_->checkMotion(origState, stateProperty_[nearVertex]))  // is valid motion
        {
            std::cout << "      Skipping neighbor " << nearVertex << ", i=" << i
                      << ", at level=" << getTaskLevel(nearVertex) << " because invalid motion" << std::endl;
            neighbors.erase(neighbors.begin() + i);
            i--;
            continue;
        }

        std::cout << "      Neighbor " << nearVertex << " is a keeper! " << std::endl;
    }
}

void og::BoltDB::viz2Edge(Edge &e)
{
    const Vertex &v1 = boost::source(e, g_);
    const Vertex &v2 = boost::target(e, g_);

    // Visualize
    viz2EdgeCallback(stateProperty_[v1], stateProperty_[v2], edgeWeightProperty_[e]);
}
