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
#include <queue>

// Allow hooks for visualizing planner
#define OMPL_BOLT_DEBUG

namespace og = ompl::geometric;
namespace ob = ompl::base;

// edgeWeightMap methods ////////////////////////////////////////////////////////////////////////////

BOOST_CONCEPT_ASSERT(
    (boost::ReadablePropertyMapConcept<ompl::geometric::BoltDB::edgeWeightMap, ompl::geometric::BoltDB::Edge>));

og::BoltDB::edgeWeightMap::edgeWeightMap(const Graph &graph, const EdgeCollisionStateMap &collisionStates,
    const double &popularityBias, const bool popularityBiasEnabled)
    : g_(graph), collisionStates_(collisionStates), popularityBias_(popularityBias), popularityBiasEnabled_(popularityBiasEnabled)
{
}

double og::BoltDB::edgeWeightMap::get(Edge e) const
{
    // Maximum cost an edge can have based on popularity
    const double MAX_POPULARITY_WEIGHT = 100.0;

    // Get the status of collision checking for this edge
    if (collisionStates_[e] == IN_COLLISION)
        return std::numeric_limits<double>::infinity();

    double weight;
    if (popularityBiasEnabled_)
    {
        // static const double popularityBias = 10;
        weight = boost::get(boost::edge_weight, g_, e) / MAX_POPULARITY_WEIGHT * popularityBias_;
        //std::cout << "getting popularity weight of edge " << e << " with value " << weight << std::endl;
    }
    else
    {
        weight = boost::get(boost::edge_weight, g_, e);
    }

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
    , popularityBiasEnabled_(false)
    , verbose_(true)
    , distanceAcrossCartesian_(0.0)
    , visualizeAstar_(false)
    , visualizeGridGeneration_(false)
    , visualizeCartNeighbors_(false)
    , visualizeCartPath_(false)
    , sparseDelta_(2.0)
    , discretization_(2.0)
    , visualizeAstarSpeed_(0.1)
{
    nn_.reset(new NearestNeighborsGNATNoThreadSafety<Vertex>());
    nn_->setDistanceFunction(boost::bind(&og::BoltDB::distanceFunction, this, _1, _2));
    sparse_nn_.reset(new NearestNeighborsGNATNoThreadSafety<Vertex>());
    sparse_nn_->setDistanceFunction(boost::bind(&og::BoltDB::distanceFunction, this, _1, _2));
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
    if (sparse_nn_)
        sparse_nn_->clear();

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

    OMPL_INFORM("BoltDB: Loaded planner data with \n  %d vertices\n  %d edges", plannerData->numVertices(),
        plannerData->numEdges());

    if (!plannerData->numVertices() || !plannerData->numEdges())
    {
        OMPL_ERROR("Corrupted planner data loaded, skipping building graph");
        return false;
    }

    // Add to db
    OMPL_INFORM("Adding plannerData to database.");
    loadFromPlannerData(*plannerData);

    // Close file
    iStream.close();

    double loadTime = time::seconds(time::now() - start);
    OMPL_INFORM("Loading from file took %f sec", loadTime);
    return true;
}

bool og::BoltDB::postProcessPath(og::PathGeometric &solutionPath, double &insertionTime)
{
    bool verbose = false;

    // Prevent inserting into database
    if (!savingEnabled_)
    {
        OMPL_WARN("BoltDB: Saving is disabled so not adding path");
        return false;
    }

    // Clear all visuals
    //viz2StateCallback(currentPathState, 0, 1);

    // Get starting state
    base::State *currentPathState = solutionPath.getStates()[0];
    std::size_t currVertexIndex = 1;

    // Find starting state's vertex
    // TODO(davetcoleman): loop through all possible start vertices
    stateProperty_[queryVertex_] = currentPathState;
    Vertex prevGraphVertex = nn_->nearest(queryVertex_);
    stateProperty_[queryVertex_] = NULL; // Set search vertex to NULL to prevent segfault on class unload of memory

    // Visualize
    vizStateCallback(stateProperty_[prevGraphVertex], 1, 1);

    // Create new path that is 'snapped' onto the roadmap
    std::vector<Vertex> roadmapPath;
    roadmapPath.push_back(prevGraphVertex);

    // Remember if any connections failed
    bool allValid = true;

    // Start recursive function
    if (!recurseSnapWaypoints(solutionPath, roadmapPath, currVertexIndex, prevGraphVertex, allValid))
    {
        // TODO
        OMPL_ERROR("Could not connect to second point in trajectory - TODO loop through first point neighbors");
        return false;
    }

    // Visualize
    vizTriggerCallback();
    usleep(0.1 * 1000000);

    // Error check
    if (!allValid)
    {
        OMPL_ERROR("Found case where unable to connect");
        exit(-1); // TODO(davetcoleman): gracefully fail - just ignore that path
    }

    // Error check
    if (roadmapPath.size() < 2)
    {
        OMPL_WARN("Snapped path waypoint count is too short, only contains %u waypoints", roadmapPath.size());
        if (roadmapPath.empty())
        {
            OMPL_ERROR("Trajectory completely empty!");
            exit(-1);
        }
        // It is possible to have a path of [actualStart, middlePoint, actualGoal], in which case we can't save any experience from it
    }

    // Update edge weights based on this newly created path
    for (std::size_t vertexID = 1; vertexID < roadmapPath.size(); ++vertexID)
    {
        std::pair<BoltDB::Edge, bool> edgeResult = boost::edge(roadmapPath[vertexID - 1], roadmapPath[vertexID], g_);
        BoltDB::Edge& edge = edgeResult.first;

        // Error check
        if (!edgeResult.second)
        {
            OMPL_ERROR("No edge found on snapped path at index %u", vertexID);
            viz3EdgeCallback(stateProperty_[roadmapPath[vertexID - 1]], stateProperty_[roadmapPath[vertexID]], 0);
            viz3TriggerCallback();
            usleep(4*1000000);
        }
        else
        {
            // reduce cost of this edge because it was just used (increase popularity)
            // Note: 100 is an *unpopular* edge, and 0 is a super highway
            static const double REDUCTION_AMOUNT = 5;
            if (verbose)
            {
                std::cout << "Edge weight for vertex " << vertexID << " of edge " << edge << std::endl;
                std::cout << "    old: " << edgeWeightProperty_[edge];
            }
            edgeWeightProperty_[edge] = std::max(edgeWeightProperty_[edge] - REDUCTION_AMOUNT, 0.0);
            if (verbose)
                std::cout << " new: " << edgeWeightProperty_[edge] << std::endl;

            // Visualize
            viz3EdgeCallback(stateProperty_[roadmapPath[vertexID - 1]], stateProperty_[roadmapPath[vertexID]], 100);
        }
    }
    viz3TriggerCallback();

    // Record this new addition
    graphUnsaved_ = true;

    return true;
}

bool og::BoltDB::recurseSnapWaypoints(og::PathGeometric& inputPath, std::vector<Vertex>& roadmapPath,
    std::size_t currVertexIndex, const Vertex& prevGraphVertex, bool& allValid)

{
    bool verbose = false;

    if (verbose)
    {
        std::cout << std::endl;
        std::cout << std::string(currVertexIndex, ' ') + "recurseSnapWaypoints() -------" << std::endl;
    }

    // Find multiple nearby nodes on the graph
    std::vector<Vertex> graphNeighborhood;

    // Get the next state
    base::State *currentPathState = inputPath.getState(currVertexIndex);

    // Find multiple nearby nodes on the graph
    stateProperty_[queryVertex_] = currentPathState;
    std::size_t findNearestKNeighbors = 10;
    const std::size_t numSameVerticiesFound = 1;  // add 1 to the end because the NN tree always returns itself
    nn_->nearestK(queryVertex_, findNearestKNeighbors + numSameVerticiesFound, graphNeighborhood);

    // Loop through each neighbor until one is found that connects to the previous vertex
    bool foundValidConnToPrevious = false;
    Vertex currGraphVertex;
    for (std::size_t neighborID = 0; neighborID < graphNeighborhood.size(); ++neighborID)
    {
        bool isValid = false;
        bool isRepeatOfPrevWaypoint = false; // don't add current waypoint if same as last one

        // Developer feedback - help tune variable findNearestKNeighbors
        if (neighborID > 4)
        {
            OMPL_WARN("Using a neighbor ID as high as %u", neighborID);
        }

        // Find next state's vertex
        currGraphVertex = graphNeighborhood[neighborID];

        // Check if next vertex is same as previous
        if (prevGraphVertex == currGraphVertex)
        {
            // Do not do anything, we are done here
            foundValidConnToPrevious = true;
            if (verbose)
                std::cout << std::string(currVertexIndex, ' ') <<
                    "Previous vertex is same as current vertex, skipping current vertex" << std::endl;
            isValid = true;
            isRepeatOfPrevWaypoint = true;
        }
        else
        {
            // Visualize nearby state
            vizStateCallback(stateProperty_[currGraphVertex], 1, 1);
            vizEdgeCallback(currentPathState, stateProperty_[currGraphVertex], 30);

            // Check for collision
            isValid = si_->checkMotion(stateProperty_[prevGraphVertex], stateProperty_[currGraphVertex]);
            double color = isValid ? 0 : 60;

            // Visualize
            vizEdgeCallback(stateProperty_[prevGraphVertex], stateProperty_[currGraphVertex], color);
        }

        // Remember if any connections failed
        if (isValid)
        {
            if (verbose)
                std::cout << std::string(currVertexIndex, ' ') + "Loop " << neighborID << " valid" << std::endl;
            if (neighborID > 0)
            {
                OMPL_WARN("Found case where double loop fixed the problem - loop %u", neighborID);
                //vizTriggerCallback();
                //usleep(6*1000000);
            }
            foundValidConnToPrevious = true;

            // Add this waypoint solution
            if (!isRepeatOfPrevWaypoint)
                roadmapPath.push_back(currGraphVertex);

            // Check if there are more points to process
            if (currVertexIndex + 1 >= inputPath.getStateCount())
            {
                if (verbose)
                    std::cout << std::string(currVertexIndex, ' ') + "End reached" << std::endl;
                return true;
            }
            else
            {
                // Recurisvely call next level
                if (recurseSnapWaypoints(inputPath, roadmapPath, currVertexIndex + 1, currGraphVertex, allValid))
                {
                    return true;
                }
                else
                {
                    // Keep trying to find a working neighbor
                    // remove last roadmapPath node
                    roadmapPath.pop_back();
                }
            }
        }
        else
        {
            if (verbose)
                std::cout << std::string(currVertexIndex, ' ') + "Loop " << neighborID << " not valid" << std::endl;
        }
    }

    if (!foundValidConnToPrevious)
    {
        OMPL_ERROR("Unable to find valid connection to previous");
        allValid = false;

        // Visualize
        vizTriggerCallback();
        usleep(1 * 1000000);

        return false;
    }

    // This should not happen?
    OMPL_WARN("This should not happen");
    exit(-1);
    return false;
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
    if (useTaskPlanning_)
    {
        if (getTaskLevel(start) != 0)
        {
            OMPL_ERROR("astarSearch: start level is %u", getTaskLevel(start));
            exit(-1);
        }
        if (getTaskLevel(goal) != 2)
        {
            OMPL_ERROR("astarSearch: goal level is %u", getTaskLevel(goal));
            exit(-1);
        }
    }

    OMPL_INFORM("Beginning AStar Search");
    try
    {
        // Note: could not get astar_search to compile within BoltRetrieveRepair.cpp class because of namespacing issues
        boost::astar_search(g_,     // graph
            start,  // start state
            // boost::bind(&og::BoltDB::distanceFunction2, this, _1, goal),  // the heuristic
            // boost::bind(&og::BoltDB::distanceFunction, this, _1, goal),  // the heuristic
            boost::bind(&og::BoltDB::distanceFunctionTasks, this, _1, goal),  // the heuristic
            // ability to disable edges (set cost to inifinity):
            boost::weight_map(edgeWeightMap(g_, edgeCollisionStateProperty_, popularityBias_, popularityBiasEnabled_))
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
            if (v1 != v2)
            {
                //std::cout << "Edge " << v1 << " to " << v2 << std::endl;
                vizEdgeCallback(stateProperty_[v1], stateProperty_[v2], 10);
            }
        }
        viz2TriggerCallback();
    }

    // Unload
    delete[] vertexPredecessors;
    delete[] vertexDistances;

    // No solution found from start to goal
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
    // Do not use task distance if that mode is not enabled
    if (!useTaskPlanning_)
        return distanceFunction(a, b);

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

    double dist;                                // the result
    static const double TASK_LEVEL_COST = 1.0;  // cost to change levels/tasks

    // Error check
    assert(stateProperty_[a]);
    assert(stateProperty_[b]);
    assert(stateProperty_[startConnectorVertex_]);
    assert(stateProperty_[endConnectorVertex_]);

    if (taskLevelA == 0)
    {
        if (taskLevelB == 0)  // regular distance for bottom level
        {
            if (verbose)
                std::cout << "a ";
            dist = si_->distance(stateProperty_[a], stateProperty_[b]);
        }
        else if (taskLevelB == 1)
        {
            if (verbose)
                std::cout << "b ";
            dist = si_->distance(stateProperty_[a], stateProperty_[startConnectorVertex_]) + TASK_LEVEL_COST +
                si_->distance(stateProperty_[startConnectorVertex_], stateProperty_[b]);
        }
        else if (taskLevelB == 2)
        {
            if (verbose)
                std::cout << "c ";
            dist = si_->distance(stateProperty_[a], stateProperty_[startConnectorVertex_]) + TASK_LEVEL_COST +
                distanceAcrossCartesian_ + TASK_LEVEL_COST +
                si_->distance(stateProperty_[endConnectorVertex_], stateProperty_[b]);
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
            dist = si_->distance(stateProperty_[a], stateProperty_[endConnectorVertex_]) + TASK_LEVEL_COST +
                si_->distance(stateProperty_[endConnectorVertex_], stateProperty_[b]);
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
                  << " has distance " << dist << std::endl;
    return dist;
}

std::size_t og::BoltDB::getTaskLevel(const Vertex &v) const
{
    return si_->getStateSpace()->getLevel(stateProperty_[v]);
}

std::size_t og::BoltDB::getTaskLevel(const base::State *state) const
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
    std::cout << "Vertices loaded: ";
    for (std::size_t vertexID = 0; vertexID < data.numVertices(); ++vertexID)
    {
        if ((vertexID + 1) % debugFrequency == 0)
            std::cout << std::fixed << std::setprecision(0) << (vertexID / double(data.numVertices())) * 100.0 << "% ";

        // Get the state from loaded planner data
        const base::State *oldState = data.getVertex(vertexID).getState();
        base::State *state = si_->cloneState(oldState);

        // Get the tag, which in this application represents the vertex type
        GuardType type = static_cast<GuardType>(data.getVertex(vertexID).getTag());

        // ADD GUARD
        idToVertex.push_back(addVertex(state, type));
    }
    std::cout << std::endl;

    OMPL_INFORM("  Loading %u edges into BoltDB", data.numEdges());
    // Add the corresponding edges to the graph
    std::vector<unsigned int> edgeList;
    std::cout << "Edges loaded: ";
    for (unsigned int fromVertex = 0; fromVertex < data.numVertices(); ++fromVertex)
    {
        if ((fromVertex + 1) % debugFrequency == 0)
            std::cout << std::fixed << std::setprecision(0) << fromVertex / double(data.numVertices()) * 100.0 << "% ";

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
    std::cout << std::endl;
    OMPL_INFORM("  Finished loading %u edges", getNumEdges());

    // OMPL_WARN("temp save when load graph from file");
    // graphUnsaved_ = true;

    // Re-enable verbose mode, if necessary
    verbose_ = wasVerbose;
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
}

void og::BoltDB::generateTaskSpace()
{
    OMPL_INFORM("Generating task space");
    std::vector<Vertex> vertexToNewVertex(getNumVertices());

    OMPL_INFORM("Adding task space vertices");
    BOOST_FOREACH (Vertex v, boost::vertices(g_))
    {
        // The first vertex (id=0) should have a NULL state because it is used for searching
        if (!stateProperty_[v])
        {
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
        stateProperty_[queryVertex_] = NULL; // Set search vertex to NULL to prevent segfault on class unload of memory

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
        if (popularityBiasEnabled_)
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
    for (double value = bounds.low[joint_id]; value <= bounds.high[joint_id]; value += discretization_)
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

void og::BoltDB::displayDatabase(bool showVertices)
{
    OMPL_INFORM("Displaying database");

    if (showVertices)
    {
        // Loop through each vertex
        std::size_t count = 0;
        std::size_t debugFrequency = getNumVertices() / 10;
        std::cout << "Displaying database: ";
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
                std::cout << std::fixed << std::setprecision(0) << (static_cast<double>(count+1) / getNumVertices()) * 100.0 << "% ";
                viz2TriggerCallback();
            }
            count++;
        }
        std::cout << std::endl;
    }
    else
    {
        // Loop through each edge
        std::size_t count = 0;
        std::size_t debugFrequency = getNumEdges() / 10;
        std::cout << "Displaying database: ";
        BOOST_FOREACH (Edge e, boost::edges(g_))
        {
            // Add edge
            const Vertex &v1 = boost::source(e, g_);
            const Vertex &v2 = boost::target(e, g_);

            // Visualize
            viz2EdgeCallback(stateProperty_[v1], stateProperty_[v2], edgeWeightProperty_[e]);

            // Prevent cache from getting too big
            if (count % debugFrequency == 0)
            {
                std::cout << std::fixed << std::setprecision(0) << (static_cast<double>(count+1) / getNumEdges()) * 100.0 << "% ";
                viz2TriggerCallback();
            }

            count++;
        }
        std::cout << std::endl;
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

void og::BoltDB::setViz3Callbacks(ompl::base::VizStateCallback vizStateCallback,
    ompl::base::VizEdgeCallback vizEdgeCallback,
    ompl::base::VizTriggerCallback vizTriggerCallback)
{
    viz3StateCallback_ = vizStateCallback;
    viz3EdgeCallback_ = vizEdgeCallback;
    viz3TriggerCallback_ = vizTriggerCallback;
}

void og::BoltDB::normalizeGraphEdgeWeights()
{
    if (!popularityBiasEnabled_)
    {
        OMPL_INFORM("Skipping normalize graph edge weights because not using popularity bias currently");
        return;
    }

    // Maximum cost an edge can have based on popularity
    const double MAX_POPULARITY_WEIGHT = 100.0;

    // Normalize weight of graph
    double total_cost = 0;
    BOOST_FOREACH (Edge e, boost::edges(g_))
    {
        total_cost += edgeWeightProperty_[e];
    }
    double avg_cost = total_cost / getNumEdges();
    OMPL_INFORM("Average cost of the edges in graph is %f", avg_cost);
    double desired_avg_cost = 90;

    if (avg_cost < desired_avg_cost)  // need to decrease cost in graph
    {
        double avg_cost_diff = desired_avg_cost - avg_cost;
        std::cout << "avg_cost_diff: " << avg_cost_diff << std::endl;
        double perEdge_reduction = avg_cost_diff;  // / getNumEdges();
        OMPL_INFORM("Decreasing each edge's cost by %f", perEdge_reduction);
        BOOST_FOREACH (Edge e, boost::edges(g_))
        {
            edgeWeightProperty_[e] = std::min(edgeWeightProperty_[e] + perEdge_reduction, MAX_POPULARITY_WEIGHT);
        }
    }
    else
    {
        OMPL_INFORM("Not decreasing all edge's cost because average is above desired");
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
        OMPL_INFORM("Skipping verticies cleanup - no middle cartesian layer verticies found");
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
    // Error check
    if (path.size() < 2)
    {
        OMPL_ERROR("Invalid cartesian path - too few states");
        exit(-1);
    }
    // TODO: check for validity

    // Create verticies for the extremas
    Vertex startVertex = addVertex(path.front(), CARTESIAN);
    Vertex goalVertex = addVertex(path.back(), CARTESIAN);

    // Record min cost for cost-to-go heurstic distance function later
    distanceAcrossCartesian_ = distanceFunction(startVertex, goalVertex);

    // Connect Start to graph --------------------------------------
    std::cout << "  start connector ---------" << std::endl;
    const std::size_t level0 = 0;
    if (!connectStateToNeighborsAtLevel(startVertex, level0, startConnectorVertex_))
    {
        OMPL_ERROR("Failed to connect start of cartesian path");
        return false;
    }

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
            usleep(0.001 * 1000000);
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
    else if (neighbors.size() < 3)
    {
        OMPL_WARN("Only %u neighbors found on level %u", neighbors.size(), level);
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
            minConnectorCost = connectorCost;  // TODO(davetcoleman): should we save the cost, or just use 1.0?
            minConnectorVertex = v;
        }

        // Visualize connection to goal of cartesian path
        const double cost = (level == 0 ? 100 : 50);
        if (visualizeCartNeighbors_)
        {
            vizEdgeCallback(stateProperty_[v], stateProperty_[fromVertex], cost);
            vizStateCallback(stateProperty_[v], 1, 1);
            vizTriggerCallback();
            usleep(0.001 * 1000000);
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
    stateProperty_[queryVertex_] = NULL; // Set search vertex to NULL to prevent segfault on class unload of memory
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

        std::cout << "      Keeping neighbor " << nearVertex << std::endl;
    }
}

void og::BoltDB::viz2Edge(Edge &e)
{
    const Vertex &v1 = boost::source(e, g_);
    const Vertex &v2 = boost::target(e, g_);

    // Visualize
    viz2EdgeCallback(stateProperty_[v1], stateProperty_[v2], edgeWeightProperty_[e]);
}

bool og::BoltDB::checkTaskPathSolution(og::PathGeometric &path, ob::State *start, ob::State *goal)
{
    bool error = false;
    int current_level = 0;

    for (std::size_t i = 0; i < path.getStateCount(); ++i)
    {
        int level = si_->getStateSpace()->getLevel(path.getState(i));

        // Check if start state is correct
        if (i == 0)
        {
            if (!si_->getStateSpace()->equalStates(path.getState(i), start))
            {
                OMPL_ERROR("Start state of path is not same as original problem");
                error = true;
            }

            if (level != 0)
            {
                OMPL_ERROR("Start state is not at level 0, instead %i", level);
                error = true;
            }
        }

        // Check if goal state is correct
        if (i == path.getStateCount() - 1)
        {
            if (!si_->getStateSpace()->equalStates(path.getState(i), goal))
            {
                OMPL_ERROR("Goal state of path is not same as original problem");
                error = true;
            }

            if (level != 2)
            {
                OMPL_ERROR("Goal state is not at level 2, instead %i", level);
                error = true;
            }
        }

        // Ensure that level is always increasing
        if (level < current_level)
        {
            OMPL_ERROR("State decreased in level (%i) from previous level of ", current_level);
            error = true;
        }
        current_level = level;

    }  // for loop

    // Show more data if error
    if (error)
    {
        OMPL_ERROR("Showing data on path:");
        for (std::size_t i = 0; i < path.getStateCount(); ++i)
        {
            int level = si_->getStateSpace()->getLevel(path.getState(i));
            OMPL_INFORM(" - Path state %i has level %i", i, level);
        }
    }

    return error;
}

void og::BoltDB::checkStateType()
{
    std::size_t count = 0;
    BOOST_FOREACH (const Vertex v, boost::vertices(g_))
    {
        // The first vertex (id=0) should have a NULL state because it is used for searching
        if (!stateProperty_[v])
        {
            if (count != 0)
            {
                OMPL_ERROR("Null state found for vertex that is not zero");
            }
            continue;
        }

        std::size_t level = getTaskLevel(stateProperty_[v]);
        if (level > 2)
        {
            OMPL_ERROR("State is on wrong level: %u", level);
            exit(-1);
        }
    }
    OMPL_INFORM("All states checked for task level");
}

// Create SPARs graph using popularity
void og::BoltDB::createSPARS()
{
    bool verbose = false;

    // Sort the verticies by popularity in a queue
    std::priority_queue<WeightedVertex, std::vector<WeightedVertex>, CompareWeightedVertex> pqueue;

    // Loop through each popular edge in the dense graph
    BOOST_FOREACH (Vertex v, boost::vertices(g_))
    {
        // Do not process the search vertex, it is null
        if (v == 0)
            continue;

        if (verbose)
            std::cout << "Vertex: " << v << std::endl;
        double popularity = 0;
        // std::pair<out_edge_iterator, out_edge_iterator> edge
        BOOST_FOREACH (Edge edge, boost::out_edges(v, g_))
        {
            if (verbose)
                std::cout << "  Edge: " << edge << std::endl;
            popularity += (100 - edgeWeightProperty_[edge]);
        }
        if (verbose)
            std::cout << "  Total popularity: " << popularity << std::endl;
        pqueue.push(WeightedVertex(v, popularity));
    }

    double largestWeight = pqueue.top().weight_;

    // Output the vertices in order
    while (!pqueue.empty())
    {
        Vertex v = pqueue.top().v_;
        std::cout << "  Visualizing vertex " << v << " with popularity " << pqueue.top().weight_
                  << " queue remaining size " << pqueue.size() << std::endl;

        // Visualize
        const double visualWeight = pqueue.top().weight_ / largestWeight;
        vizStateCallback(stateProperty_[v], 7, visualWeight);
        vizTriggerCallback();
        usleep(0.01*1000000);

        // Attempt to insert into SPARS graph
        double seconds = 1000;
        ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(seconds, 0.1);
        if (!addStateToRoadmap(ptc, stateProperty_[v]))
            OMPL_INFORM("Failed to add state to roadmap");

        // Remove from priority queue
        pqueue.pop();
    } // end while

}

bool og::BoltDB::addStateToRoadmap(const base::PlannerTerminationCondition &ptc, base::State *newState)
{
    bool stateAdded = false;

    // Deep copy
    base::State *qNew = si_->cloneState(newState); // TODO(davetcoleman): do i need to clone it?
    base::State *workState = si_->allocState(); // TODO(davetcoleman): do i need this state?

    /* Nodes near our newState */
    std::vector<Vertex> graphNeighborhood;
    /* Visible nodes near our newState */
    std::vector<Vertex> visibleNeighborhood;

    // Find nearby nodes
    findGraphNeighbors(qNew, graphNeighborhood, visibleNeighborhood);

    // Always add a node if no other nodes around it are visible (GUARD)
    if (checkAddCoverage(qNew, visibleNeighborhood))
    {
        stateAdded = true;
    }
    else if (checkAddConnectivity(qNew, visibleNeighborhood)) // Connectivity criterion
    {
        stateAdded = true;
    }

    /*
    else if (checkAddInterface(qNew, graphNeighborhood, visibleNeighborhood))
    {
        stateAdded = true;
    }
    else
    {
        if (verbose_)
            OMPL_INFORM(" ---- Ensure SPARS asymptotic optimality");
        if (visibleNeighborhood.size() > 0)
        {
            std::map<Vertex, base::State*> closeRepresentatives;
            if (verbose_)
                OMPL_INFORM(" ----- findCloseRepresentatives()");

            findCloseRepresentatives(workState, qNew, visibleNeighborhood[0], closeRepresentatives, ptc);
            if (verbose_)
                OMPL_INFORM("------ Found %d close representatives", closeRepresentatives.size());

            for (std::map<Vertex, base::State*>::iterator it = closeRepresentatives.begin(); it != closeRepresentatives.end(); ++it)
            {
                if (verbose_)
                    OMPL_INFORM(" ------ Looping through close representatives");
                updatePairPoints(visibleNeighborhood[0], qNew, it->first, it->second);
                updatePairPoints(it->first, it->second, visibleNeighborhood[0], qNew);
            }
            if (verbose_)
                OMPL_INFORM(" ------ checkAddPath()");
            if (checkAddPath(visibleNeighborhood[0]))
            {
                if (verbose_)
                {
                    OMPL_INFORM("nearest visible neighbor added ");
                }
            }

            for (std::map<Vertex, base::State*>::iterator it = closeRepresentatives.begin(); it != closeRepresentatives.end(); ++it)
            {
                if (verbose_)
                    OMPL_INFORM(" ------- Looping through close representatives to add path");
                checkAddPath(it->first);
                si_->freeState(it->second);
            }
            if (verbose_)
                OMPL_INFORM("------ Done with inner most loop ");
        }
    }
    */

    si_->freeState(workState);
    si_->freeState(qNew);

    return stateAdded;
}

og::BoltDB::Vertex og::BoltDB::addSparseVertex(base::State *state, const GuardType &type)
{
    // Create vertex
    //Vertex v1 = boost::add_vertex(g_);

    // Add properties
    //typeProperty_[v1] = type;
    //stateProperty_[v1] = state;

    // Temp hack
    stateProperty_[queryVertex_] = state;
    Vertex v1 = nn_->nearest( queryVertex_ );
    stateProperty_[queryVertex_] = NULL;

    // Add vertex to nearest neighbor structure
    sparse_nn_->add(v1);

    return v1;
}

bool og::BoltDB::checkAddCoverage(const base::State *qNew, std::vector<Vertex> &visibleNeighborhood)
{
    if (verbose_)
        OMPL_INFORM(" - checkAddCoverage() Are other nodes around it visible?");

    if (visibleNeighborhood.size() > 0)
        return false; // has visible neighbors

    // No free paths means we add for coverage
    if (verbose_)
        OMPL_INFORM(" --- Adding node for COVERAGE ");

    Vertex v = addSparseVertex(si_->cloneState(qNew), COVERAGE);
    // Note: we do not connect this node with any edges because we have already determined
    // it is too far away from any nearby nodes

    // Visualize
    viz3StateCallback(stateProperty_[v], 4, sparseDelta_);
    viz3TriggerCallback();
    usleep(0.01*1000000);

    return true;
}

bool og::BoltDB::checkAddConnectivity(const base::State *qNew, std::vector<Vertex> &visibleNeighborhood)
{
    if (verbose_)
        OMPL_INFORM(" -- checkAddConnectivity() Does this node connect neighboring nodes that are not connected? ");

    // Error check
    if (visibleNeighborhood.size() < 2)
    {
        // if less than 2 there is no way to find a pair of nodes in different connected components
        return false;
    }

    // Identify visibile nodes around our new state that are unconnected (in different connected components)
    // and connect them
    std::vector<Vertex> statesInDiffConnectedComponents;

    //For each neighbor
    for (std::size_t i = 0; i < visibleNeighborhood.size(); ++i)
    {
        //For each other neighbor
        for (std::size_t j = i + 1; j < visibleNeighborhood.size(); ++j)
        {
            //If they are in different components
            if (!sameComponent(visibleNeighborhood[i], visibleNeighborhood[j]))
            {
                statesInDiffConnectedComponents.push_back(visibleNeighborhood[i]);
                statesInDiffConnectedComponents.push_back(visibleNeighborhood[j]);
            }
        }
    }

    // Were any diconnected states found?
    if (statesInDiffConnectedComponents.size() > 0)
    {
        if (verbose_)
            OMPL_INFORM(" --- Adding node for CONNECTIVITY ");
        //Add the node
        Vertex newVertex = addSparseVertex(si_->cloneState(qNew), CONNECTIVITY);

        for (std::size_t i = 0; i < statesInDiffConnectedComponents.size() ; ++i)
        {
            //If there's no edge between the two new states
            // DTC: this should actually never happen - we just created the new vertex so
            // why would it be connected to anything?
            if (!boost::edge(newVertex, statesInDiffConnectedComponents[i], g_).second)
            {
                //The components haven't been united by previous links
                if (!sameComponent(statesInDiffConnectedComponents[i], newVertex))
                    connectGuards(newVertex, statesInDiffConnectedComponents[i]);
            }
        }

        return true;
    }

    return false;
}
/*
bool og::BoltDB::checkAddInterface(const base::State *qNew, std::vector<Vertex> &graphNeighborhood, std::vector<Vertex> &visibleNeighborhood)
{
        if (verbose_)
            OMPL_INFORM(" --- checkAddInterface() Does this node's neighbor's need it to better connect them? ");

    //If we have at least 2 neighbors
    if (visibleNeighborhood.size() > 1)
    {
        // If the two closest nodes are also visible
        if (graphNeighborhood[0] == visibleNeighborhood[0] && graphNeighborhood[1] == visibleNeighborhood[1])
        {
            // If our two closest neighbors don't share an edge
            if (!boost::edge(visibleNeighborhood[0], visibleNeighborhood[1], g_).second)
            {
                //If they can be directly connected
                if (si_->checkMotion(stateProperty_[visibleNeighborhood[0]], stateProperty_[visibleNeighborhood[1]]))
                {
                    //Connect them
                    if (verbose_)
                        OMPL_INFORM(" ---   INTERFACE: directly connected nodes ");
                    connectGuards(visibleNeighborhood[0], visibleNeighborhood[1]);
                    //And report that we added to the roadmap
                    resetFailures();
                    //Report success
                    return true;
                }
                else
                {
                    //Add the new node to the graph, to bridge the interface
                    if (verbose_)
                        OMPL_INFORM(" --- Adding node for INTERFACE  ");
                    Vertex v = addSparseVertex(si_->cloneState(qNew), INTERFACE);
                    connectGuards(v, visibleNeighborhood[0]);
                    connectGuards(v, visibleNeighborhood[1]);
                    if (verbose_)
                        OMPL_INFORM(" ---   INTERFACE: connected two neighbors through new interface node ");
                    //Report success
                    return true;
                }
            }
        }
    }
    return false;
}

bool og::BoltDB::checkAddPath( Vertex v )
{
    bool spannerPropertyWasViolated = false;

    std::vector< Vertex > rs;
    foreach( Vertex r, boost::adjacent_vertices( v, g_ ) )
        rs.push_back(r);

    // Candidate x vertices as described in the method, filled by function computeX().
    std::vector<Vertex> Xs;

    // Candidate v" vertices as described in the method, filled by function computeVPP().
    std::vector<Vertex> VPPs;

    for (std::size_t i = 0; i < rs.size() && !spannerPropertyWasViolated; ++i)
    {
        Vertex r = rs[i];
        computeVPP(v, r, VPPs);
        foreach (Vertex rp, VPPs)
        {
            //First, compute the longest path through the graph
            computeX(v, r, rp, Xs);
            double rm_dist = 0.0;
            foreach( Vertex rpp, Xs)
            {
                double tmp_dist = (si_->distance( stateProperty_[r], stateProperty_[v] )
                                   + si_->distance( stateProperty_[v], stateProperty_[rpp] ) )/2.0;
                if( tmp_dist > rm_dist )
                    rm_dist = tmp_dist;
            }

            InterfaceData& d = getData( v, r, rp );

            //Then, if the spanner property is violated
            if (rm_dist > stretchFactor_ * d.last_distance_)
            {
                spannerPropertyWasViolated = true; //Report that we added for the path
                if (si_->checkMotion(stateProperty_[r], stateProperty_[rp]))
                    connectGuards(r, rp);
                else
                {
                    PathGeometric *p = new PathGeometric( si_ );
                    if (r < rp)
                    {
                        p->append(d.sigmaA_);
                        p->append(d.pointA_);
                        p->append(stateProperty_[v]);
                        p->append(d.pointB_);
                        p->append(d.sigmaB_);
                    }
                    else
                    {
                        p->append(d.sigmaB_);
                        p->append(d.pointB_);
                        p->append(stateProperty_[v]);
                        p->append(d.pointA_);
                        p->append(d.sigmaA_);
                    }

                    psimp_->reduceVertices(*p, 10);
                    psimp_->shortcutPath(*p, 50);

                    if (p->checkAndRepair(100).second)
                    {
                        Vertex prior = r;
                        Vertex vnew;
                        std::vector<base::State*>& states = p->getStates();

                        foreach (base::State *st, states)
                        {
                            // no need to clone st, since we will destroy p; we just copy the pointer
                            if (verbose_)
                                OMPL_INFORM(" --- Adding node for QUALITY");
                            vnew = addSparseVertex(st , QUALITY);

                            connectGuards(prior, vnew);
                            prior = vnew;
                        }
                        // clear the states, so memory is not freed twice
                        states.clear();
                        connectGuards(prior, rp);
                    }

                    delete p;
                }
            }
        }
    }

    if (!spannerPropertyWasViolated)
    {
        if (verbose_)
        {
            OMPL_INFORM(" ------- Spanner property was NOT violated, SKIPPING");
        }
    }

    return spannerPropertyWasViolated;
}
*/
void og::BoltDB::findGraphNeighbors(base::State *state, std::vector<Vertex> &graphNeighborhood,
    std::vector<Vertex> &visibleNeighborhood)
{
    visibleNeighborhood.clear();

    // Search
    stateProperty_[queryVertex_] = state;
    sparse_nn_->nearestR( queryVertex_, sparseDelta_, graphNeighborhood);
    stateProperty_[queryVertex_] = NULL;

    // Now that we got the neighbors from the NN, we must remove any we can't see
    for (std::size_t i = 0; i < graphNeighborhood.size() ; ++i )
    {
        if (si_->checkMotion(state, stateProperty_[graphNeighborhood[i]]))
        {
            visibleNeighborhood.push_back(graphNeighborhood[i]);
        }
    }

    if (verbose_)
    {
        OMPL_INFORM(" Graph neighborhood: %u | visible neighborhood: %u", graphNeighborhood.size(),
            visibleNeighborhood.size());
    }

}
/*
og::BoltDB::Vertex og::BoltDB::findGraphRepresentative(base::State *st)
{
    std::vector<Vertex> nbh;
    stateProperty_[ queryVertex_ ] = st;
    sparse_nn_->nearestR( queryVertex_, sparseDelta_, nbh);
    stateProperty_[queryVertex_] = NULL;

    if (verbose_)
        OMPL_INFORM(" ------- findGraphRepresentative found %d nearest neighbors of distance %f",
                    nbh.size(), sparseDelta_);

    Vertex result = boost::graph_traits<Graph>::null_vertex();

    for (std::size_t i = 0 ; i< nbh.size() ; ++i)
    {
        if (verbose_)
            OMPL_INFORM(" -------- Checking motion of graph rep candidate %d", i);
        if (si_->checkMotion(st, stateProperty_[nbh[i]]))
        {
            if (verbose_)
                OMPL_INFORM(" --------- VALID ");
            result = nbh[i];
            break;
        }
    }
    return result;
}

void og::BoltDB::findCloseRepresentatives(base::State *workState, const base::State *qNew, const Vertex qRep,
                                                        std::map<Vertex, base::State*> &closeRepresentatives,
                                                        const base::PlannerTerminationCondition &ptc)
{
    // Properly clear the vector by also deleting previously sampled unused states
    for (std::map<Vertex, base::State*>::iterator it = closeRepresentatives.begin(); it != closeRepresentatives.end(); ++it)
        si_->freeState(it->second);
    closeRepresentatives.clear();

    //denseDelta_ = 0.25 * sparseDelta_;
    nearSamplePoints_ /= 10; // HACK - this makes it look for the same number of samples as dimensions

    if (verbose_)
        OMPL_INFORM(" ----- nearSamplePoints: %f, denseDelta: %f", nearSamplePoints_, denseDelta_);

    // Then, begin searching the space around new potential state qNew
    for (unsigned int i = 0 ; i < nearSamplePoints_ && ptc == false ; ++i)
    {
        do
        {
            sampler_->sampleNear(workState, qNew, denseDelta_);

#ifdef OMPL_THUNDER_DEBUG
                vizStateCallback(workState, 3, sparseDelta_);
                sleep(0.1);
#endif

            if (verbose_)
            {
                OMPL_INFORM(" ------ findCloseRepresentatives sampled state ");

                if (!si_->isValid(workState))
                {
                    OMPL_INFORM(" ------ isValid ");
                }
                if (si_->distance(qNew, workState) > denseDelta_)
                {
                    OMPL_INFORM(" ------ Distance too far ");
                }
                if (!si_->checkMotion(qNew, workState))
                {
                    OMPL_INFORM(" ------ Motion invalid ");
                }
            }

        } while ((!si_->isValid(workState) || si_->distance(qNew, workState) > denseDelta_ || !si_->checkMotion(qNew, workState)) && ptc == false);

        // if we were not successful at sampling a desirable state, we are out of time
        if (ptc == true)
        {
            if (verbose_)
                OMPL_INFORM(" ------ We are out of time ");
            break;
        }

        if (verbose_)
            OMPL_INFORM(" ------ Find graph representative ");

        // Compute who his graph neighbors are
        Vertex representative = findGraphRepresentative(workState);

        // Assuming this sample is actually seen by somebody (which he should be in all likelihood)
        if (representative != boost::graph_traits<Graph>::null_vertex())
        {

            if (verbose_)
                OMPL_INFORM(" ------ Representative is not null ");

            //If his representative is different than qNew
            if (qRep != representative)
            {
                if (verbose_)
                    OMPL_INFORM(" ------ qRep != representative ");

                //And we haven't already tracked this representative
                if (closeRepresentatives.find(representative) == closeRepresentatives.end())
                {
                    if (verbose_)
                        OMPL_INFORM(" ------ Track the representative");
                    //Track the representativen
                    closeRepresentatives[representative] = si_->cloneState(workState);
                }
            }
            else
            {
                if (verbose_)
                    OMPL_INFORM(" ------ qRep == representative, no good ");
            }
        }
        else
        {
            if (verbose_)
                OMPL_INFORM(" ------ Rep is null ");

            //This guy can't be seen by anybody, so we should take this opportunity to add him
            if (verbose_)
                OMPL_INFORM(" --- Adding node for COVERAGE");
            addSparseVertex(si_->cloneState(workState), COVERAGE);

            if (verbose_)
            {
                OMPL_INFORM(" ------ STOP EFFORS TO ADD A DENSE PATH");
            }

            //We should also stop our efforts to add a dense path
            for (std::map<Vertex, base::State*>::iterator it = closeRepresentatives.begin(); it != closeRepresentatives.end(); ++it)
                si_->freeState(it->second);
            closeRepresentatives.clear();
            break;
        }
    } // for loop
}
*/
