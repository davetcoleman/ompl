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

    double weight = boost::get(boost::edge_weight, g_, e);
    // if (popularityBias_)
    // {
    //     static const double popularity_bias = 10;
    //     weight = boost::get(boost::edge_weight, g_, e) / 100.0 * popularity_bias;
    // }
    // else

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
        // usleep(50000);
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
  , visualizeAstar_(false)
  , visualizeGridGeneration_(false)
  , sparseDelta_(2.0)
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
    setPlannerData(*plannerData);

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
    Vertex *vertexPredecessors = new Vertex[getNumVertices()];
    // boost::vector_property_map<Vertex> vertexPredecessors(getNumVertices());

    bool foundGoal = false;

    double *vertexDistances = new double[getNumVertices()];

    try
    {
        // Note: could not get astar_search to compile within BoltRetrieveRepair class because of namespacing issues
        boost::astar_search(g_,                                                          // graph
                            start,                                                       // start state
                            boost::bind(&og::BoltDB::distanceFunction, this, _1, goal),  // the heuristic
                            // ability to disable edges (set cost to inifinity):
                            boost::weight_map(edgeWeightMap(g_, edgeCollisionStateProperty_))
                                .predecessor_map(vertexPredecessors)
                                .distance_map(&vertexDistances[0])
                                .visitor(CustomAstarVisitor(goal, this)));
    }
    catch (foundGoalException &)
    {
        // the custom exception from CustomAstarVisitor
        if (verbose_ && false)
        {
            OMPL_INFORM("astarSearch: Astar found goal vertex ------------------------");
            OMPL_INFORM("distance to goal: %f", vertexDistances[goal]);
        }

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

    // delete[] vertexPredecessors;
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
    // std::cout << "getting distance from " << a << " to " << b << " of value "
    //<< si_->distance(stateProperty_[a], stateProperty_[b]) << std::endl;
    return si_->distance(stateProperty_[a], stateProperty_[b]);
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

void og::BoltDB::setPlannerData(const base::PlannerData &data)
{
    // Check that the query vertex is initialized (used for internal nearest neighbor searches)
    initializeQueryState();

    // Add all vertices
    OMPL_INFORM("  Loading %u vertices into BoltDB", data.numVertices());

    std::vector<Vertex> idToVertex;

    // Temp disable verbose mode for loading database
    bool wasVerbose = verbose_;
    verbose_ = false;

    // Add the nodes to the graph
    for (std::size_t vertexID = 0; vertexID < data.numVertices(); ++vertexID)
    {
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

            base::Cost *weight = new base::Cost();
            if (!data.getEdgeWeight(fromVertex, toVertex, weight))
            {
                OMPL_ERROR("Unable to get edge weight");
            }

            addEdge(v1, v2, weight->value());
        }
    }  // for
    OMPL_INFORM("  Finished loading %u edges", getNumEdges());

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
    // TODO(davetcoleman): currently the last joint is not being discretized, so we should set its default value smartly
    // and not just '0'
    std::vector<double> values(si_->getStateSpace()->getDimension(), 0);

    // Choose first state to discretize
    next_disc_state_ =
        si_->getStateSpace()->allocState();  // Note: it is currently possible the last state is never freed

    const std::size_t starting_joint_id = 0;
    std::size_t desired_depth = si_->getStateSpace()->getDimension();
    if (desired_depth > 5)
    {
        OMPL_INFORM("Truncated discretization depth to 5");
        desired_depth = 5;
    }

    // Create vertices
    recursiveDiscretization(values, starting_joint_id, desired_depth);
    OMPL_INFORM("Generated %i vertices.", getNumVertices());


    // Remove vertices in collision using multithreading
    // TODO enable
    //std::vector<Vertex> unvalidatedVertices;
    //checkVerticesThreaded(unvalidatedVertices);

    {
        // Benchmark runtime
        time::point start_time = time::now();

        // Create edges
        generateEdges();

        // Benchmark runtime
        double duration = time::seconds(time::now() - start_time);
        OMPL_WARN("Generate edges total time: %f seconds (%f hz)", duration, 1.0 / duration);
    }

    // Get the average vertex degree (number of connected edges)
    std::size_t average_degree = (getNumEdges() * 2) / getNumVertices();
    OMPL_INFORM("Average degree: %i", average_degree);

    // Display
    viz2TriggerCallback();

    // Mark the graph as ready to be saved
    graphUnsaved_ = true;
}

void og::BoltDB::generateEdges()
{
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

        {
            // Benchmark runtime
            //time::point startTime = time::now();

            // in 2D this created the regular square with diagonals of 8 edges
            static const double FIND_NEAREST_K_NEIGHBORS = si_->getStateSpace()->getDimension();  // * 4;

            nn_->nearestK(queryVertex_, FIND_NEAREST_K_NEIGHBORS, graphNeighborhood);
            stateProperty_[queryVertex_] = NULL;

            // Benchmark runtime
            //double duration = time::seconds(time::now() - startTime) * 1000;
            //OMPL_INFORM("NN Total time: %f milliseconds (%f hz)", duration, 1.0 / duration);
        }

        bool verbose = false;

        // For each nearby vertex, add an edge
        for (std::size_t i = 0; i < graphNeighborhood.size(); ++i)
        {
            if (verbose)
                OMPL_INFORM("Edge %u", i);
            Vertex &v2 = graphNeighborhood[i];

            // Check if these vertices are the same
            if (v1 == v2)
                continue;

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
    }      // for each v1

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
        double cost = 100;
        if (!popularityBias_)
        {
            cost = distanceFunction(v1, v2);
        }
        edgeWeightProperty_[e] = cost;
        errorCheckCounter++;

        // Debug in Rviz
        if (visualizeGridGeneration_)
             viz2EdgeCallback(stateProperty_[v1], stateProperty_[v2], edgeWeightProperty_[e]);
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
    bool verbose = false;

    // Error check
    assert(unvalidatedEdges.size() == getNumEdges());

    // Setup threading
    static const std::size_t numThreads = boost::thread::hardware_concurrency();
    OMPL_INFORM("Collision checking %u generated edges using %u threads", unvalidatedEdges.size(), numThreads);

    std::vector<boost::thread *> threads(numThreads);
    std::size_t edgesPerThread = getNumEdges() / numThreads;  // rounds down
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
            endEdge = getNumEdges() - 1;
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
            //edgeCollisionStateProperty_[e] = BoltDB::IN_COLLISION;
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

        // Use feedback
        if (joint_id == 0)
        {
            const double percent =
                (value - bounds.low[joint_id]) / (bounds.high[joint_id] - bounds.low[joint_id]) * 100.0;
            std::cout << "Vertex generation progress: " << percent << " % Total vertices: " << getNumVertices() << std::endl;
        }

        // std::copy(values.begin(), values.end(), std::ostream_iterator<double>(std::cout, ", "));
        // std::cout << std::endl;

        // Check if we are at the end of the recursion
        if (joint_id < desired_depth - 1)
        {
            // Keep recursing
            recursiveDiscretization(values, joint_id + 1, desired_depth);
        }
        else  // this is the end of recursion, create a new state
        {
            // TODO(davetcoleman): way to not allocState if in collision?
            next_disc_state_ = si_->getStateSpace()->allocState();

            // Fill the state with current values
            si_->getStateSpace()->populateState(next_disc_state_, values);

            // Collision check
            if (!si_->isValid(next_disc_state_))
            {
                // OMPL_ERROR("Found a state that is not valid! ");
                continue;
            }

            // Add vertex to graph
            GuardType type = START;  // TODO(davetcoleman): type START is dummy
            addVertex(next_disc_state_, type);

            // Visualize
            if (visualizeGridGeneration_)
            {
                viz2StateCallback(next_disc_state_, 5, 1);  // Candidate node has already (just) been added
                // viz2TriggerCallback();
                // usleep(0.01 * 1000000);
            }

            // Prepare for next new state by allocating now
            // next_disc_state_ = si_->getStateSpace()->allocState();
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
        //si->setMotionValidator(si_->getMotionValidator());

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

    // Loop through each edge
    std::size_t count = 0;
    BOOST_FOREACH (Edge e, boost::edges(g_))
    {
        const Vertex &v1 = boost::source(e, g_);
        const Vertex &v2 = boost::target(e, g_);

        // Choose color
        const int intensity = edgeWeightProperty_[e];

        // Add edge
        viz2EdgeCallback(stateProperty_[v1], stateProperty_[v2], intensity);
        if (count % 100 == 0)
            viz2TriggerCallback();

        count++;
    }
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
    edgeWeightProperty_[e] = weight;  // distanceFunction(v1, v2);
    edgeCollisionStateProperty_[e] = NOT_CHECKED;

    return e;
}
