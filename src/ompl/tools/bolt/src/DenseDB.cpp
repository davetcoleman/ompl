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

// OMPL
#include <ompl/tools/bolt/DenseDB.h>
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

#define foreach BOOST_FOREACH

namespace og = ompl::geometric;
namespace ot = ompl::tools;
namespace ob = ompl::base;
namespace otb = ompl::tools::bolt;

// edgeWeightMap methods ////////////////////////////////////////////////////////////////////////////

BOOST_CONCEPT_ASSERT((boost::ReadablePropertyMapConcept<otb::DenseDB::edgeWeightMap, otb::DenseEdge>));

otb::DenseDB::edgeWeightMap::edgeWeightMap(const DenseGraph &graph, const DenseEdgeCollisionStateMap &collisionStates,
                                           const double &popularityBias, const bool popularityBiasEnabled)
  : g_(graph)
  , collisionStates_(collisionStates)
  , popularityBias_(popularityBias)
  , popularityBiasEnabled_(popularityBiasEnabled)
{
}

double otb::DenseDB::edgeWeightMap::get(DenseEdge e) const
{
    // Get the status of collision checking for this edge
    if (collisionStates_[e] == IN_COLLISION)
        return std::numeric_limits<double>::infinity();

    double weight;
    if (popularityBiasEnabled_)
    {
        // static const double popularityBias = 10;
        weight = boost::get(boost::edge_weight, g_, e) / MAX_POPULARITY_WEIGHT * popularityBias_;
        // std::cout << "getting popularity weight of edge " << e << " with value " << weight << std::endl;
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
double get(const otb::DenseDB::edgeWeightMap &m, const otb::DenseEdge &e)
{
    return m.get(e);
}
}

// CustomAstarVisitor methods ////////////////////////////////////////////////////////////////////////////

BOOST_CONCEPT_ASSERT((boost::AStarVisitorConcept<otb::DenseDB::CustomAstarVisitor, otb::DenseGraph>));

otb::DenseDB::CustomAstarVisitor::CustomAstarVisitor(DenseVertex goal, DenseDB *parent) : goal_(goal), parent_(parent)
{
}

void otb::DenseDB::CustomAstarVisitor::discover_vertex(DenseVertex v, const DenseGraph &) const
{
    if (parent_->visualizeAstar_)
        parent_->getVisual()->viz4State(parent_->stateProperty_[v], /*mode=*/1, 1);
}

void otb::DenseDB::CustomAstarVisitor::examine_vertex(DenseVertex v, const DenseGraph &) const
{
    if (parent_->visualizeAstar_)
    {
        parent_->getVisual()->viz4State(parent_->stateProperty_[v], /*mode=*/5, 1);
        parent_->getVisual()->viz4Trigger();
        usleep(parent_->visualizeAstarSpeed_ * 1000000);
    }

    if (v == goal_)
        throw foundGoalException();
}

// Actual class ////////////////////////////////////////////////////////////////////////////

otb::DenseDB::DenseDB(base::SpaceInformationPtr si, base::VisualizerPtr visual)
  : si_(si)
  , visual_(visual)
  , graphUnsaved_(false)
  , savingEnabled_(true)
  // Property accessors of edges
  , edgeWeightProperty_(boost::get(boost::edge_weight, g_))
  , edgeCollisionStateProperty_(boost::get(edge_collision_state_t(), g_))
  // Property accessors of vertices
  , stateProperty_(boost::get(vertex_state_t(), g_))
  //, state3Property_(boost::get(vertex_state3_t(), g_))
  , typeProperty_(boost::get(vertex_type_t(), g_))
  , representativesProperty_(boost::get(vertex_sparse_rep_t(), g_))
  , popularityBiasEnabled_(false)
  , verbose_(true)
  , distanceAcrossCartesian_(0.0)
  , useTaskPlanning_(false)
  , snapPathVerbose_(false)
  , visualizeAstar_(false)
  , visualizeGridGeneration_(false)
  , visualizeCartNeighbors_(false)
  , visualizeCartPath_(false)
  , visualizeSnapPath_(false)
  , visualizeSnapPathSpeed_(0.001)
  , visualizeAddSample_(false)
  , visualizeAstarSpeed_(0.1)
  , discretization_(2.0)
  , desiredAverageCost_(90)
{
    // Add search state
    initializeQueryState();

    // Initialize sparse database
    sparseDB_.reset(new SparseDB(si_, this, visual_));

    // Initialize nearest neighbor datastructure
    nn_.reset(new NearestNeighborsGNATNoThreadSafety<DenseVertex>());
    nn_->setDistanceFunction(boost::bind(&otb::DenseDB::distanceFunction, this, _1, _2));
}

otb::DenseDB::~DenseDB(void)
{
    if (graphUnsaved_)
        OMPL_WARN("The database is being unloaded with unsaved experiences");
    freeMemory();
}

void otb::DenseDB::freeMemory()
{
    foreach (DenseVertex v, boost::vertices(g_))
    {
        // foreach (InterfaceData &d, interfaceDataProperty_[v].interfaceHash | boost::adaptors::map_values)
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

bool otb::DenseDB::setup()
{
    if (!sampler_)
        sampler_ = si_->allocValidStateSampler();

    sparseDB_->setup();

    return true;
}

bool otb::DenseDB::load(const std::string &fileName)
{
    OMPL_INFORM("DenseDB: load()");

    // Error checking
    if (getNumEdges() > 1 || getNumVertices() > 1)  // the search verticie may already be there
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

    OMPL_INFORM("DenseDB: Loaded planner data with %d vertices, %d edges", plannerData->numVertices(),
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

bool otb::DenseDB::postProcessPath(og::PathGeometric &solutionPath)
{
    // Prevent inserting into database
    if (!savingEnabled_)
    {
        OMPL_WARN("DenseDB: Saving is disabled so not adding path");
        return false;
    }

    if (visualizeSnapPath_)  // Clear old path
    {
        visual_->viz5State(NULL, /*deleteAllMarkers*/ 0, 0);
        visual_->viz4State(NULL, /*deleteAllMarkers*/ 0, 0);
    }

    // Get starting state
    base::State *currentPathState = solutionPath.getStates()[0];

    // Get neighbors
    std::vector<DenseVertex> graphNeighborhood;
    std::vector<DenseVertex> visibleNeighborhood;
    std::size_t coutIndent = 0;
    findGraphNeighbors(currentPathState, graphNeighborhood, visibleNeighborhood, sparseDB_->sparseDelta_, coutIndent);

    std::vector<DenseVertex> roadmapPath;

    // Run in non-debug mode
    bool recurseVerbose = snapPathVerbose_;
    if (!postProcessPathWithNeighbors(solutionPath, visibleNeighborhood, recurseVerbose, roadmapPath))
    {
        OMPL_ERROR("Could not find snap waypoint path. Running again in debug");
        std::cout << "-------------------------------------------------------" << std::endl;

        // Run in debug mode
        recurseVerbose = true;
        visualizeSnapPath_ = true;
        roadmapPath.clear();
        postProcessPathWithNeighbors(solutionPath, visibleNeighborhood, recurseVerbose, roadmapPath);

        // temp
        std::cout << "exiting for debug " << std::endl;
        exit(-1);
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
        // It is possible to have a path of [actualStart, middlePoint, actualGoal], in which case we can't save any
        // experience from it
    }

    if (roadmapPath.size() > 100)
        OMPL_WARN("Roadmap size is %u", roadmapPath.size());

    if (snapPathVerbose_)
        std::cout << "Finished recurseSnapWaypoints(), now updating edge weights in Dense graph " << std::endl;

    // Update edge weights based on this newly created path
    updateEdgeWeights(roadmapPath);

    // Record this new addition
    graphUnsaved_ = true;

    return true;
}

bool otb::DenseDB::postProcessPathWithNeighbors(og::PathGeometric &solutionPath,
    const std::vector<DenseVertex> &visibleNeighborhood, bool recurseVerbose, std::vector<DenseVertex> &roadmapPath)
{
    std::size_t currVertexIndex = 1;

    // Remember if any connections failed
    bool allValid = true;

    for (std::size_t i = 0; i < visibleNeighborhood.size(); ++i)
    {
        std::cout << "Attempting to start with neighbor " << i << std::endl;
        DenseVertex prevGraphVertex = visibleNeighborhood[i];

        if (visualizeSnapPath_)  // Add first state
        {
            visual_->viz5State(stateProperty_[prevGraphVertex], /*mode=*/1, 1);
        }

        // Add this start state
        roadmapPath.push_back(prevGraphVertex);

        // Start recursive function
        allValid = true;
        if (!recurseSnapWaypoints(solutionPath, roadmapPath, currVertexIndex, prevGraphVertex, allValid, recurseVerbose))
        {
            std::cout << "Failed to find path with starting state neighbor " << i << std::endl;
        }
        else
        {
            break; // sucess
        }

        if (visualizeSnapPath_)  // Visualize
        {
            visual_->viz5Trigger();
            usleep(visualizeSnapPathSpeed_ * 1000000);
        }
    }

    return allValid;
}

bool otb::DenseDB::updateEdgeWeights(const std::vector<DenseVertex> &roadmapPath)
{
    for (std::size_t vertexID = 1; vertexID < roadmapPath.size(); ++vertexID)
    {
        std::pair<DenseEdge, bool> edgeResult = boost::edge(roadmapPath[vertexID - 1], roadmapPath[vertexID], g_);
        DenseEdge &edge = edgeResult.first;

        // Error check
        if (!edgeResult.second)
        {
            std::cout << std::string(2, ' ') << "WARNING: No edge found on snapped path at index " << vertexID
                      << ", unable to save popularity of this edge. perhaps path needs interpolation first"
                      << std::endl;

            if (visualizeSnapPath_)  // Visualize
            {
                const double cost = 100;  // red
                visual_->viz6Edge(stateProperty_[roadmapPath[vertexID - 1]], stateProperty_[roadmapPath[vertexID]],
                                  cost);
                visual_->viz6Trigger();
                usleep(visualizeSnapPathSpeed_ * 1000000);
            }
            std::cout << "shutting down out of curiosity " << std::endl;
            exit(-1);
        }
        else
        {
            // reduce cost of this edge because it was just used (increase popularity)
            // Note: 100 is an *unpopular* edge, and 0 is a super highway
            if (snapPathVerbose_)
            {
                std::cout << "Edge weight for vertex " << vertexID << " of edge " << edge << std::endl;
                std::cout << "    old: " << edgeWeightProperty_[edge];
            }
            edgeWeightProperty_[edge] = std::max(edgeWeightProperty_[edge] - POPULARITY_WEIGHT_REDUCTION, 0.0);
            if (snapPathVerbose_)
                std::cout << " new: " << edgeWeightProperty_[edge] << std::endl;

            if (visualizeSnapPath_)  // Visualize
            {
                visual_->viz5Edge(stateProperty_[roadmapPath[vertexID - 1]], stateProperty_[roadmapPath[vertexID]],
                                  100);
            }
        }
    }

    if (visualizeSnapPath_)  // Visualize
    {
        visual_->viz5Trigger();
        usleep(visualizeSnapPathSpeed_ * 1000000);
    }

    return true;
}

bool otb::DenseDB::recurseSnapWaypoints(og::PathGeometric &inputPath, std::vector<DenseVertex> &roadmapPath,
                                        std::size_t currVertexIndex, const DenseVertex &prevGraphVertex, bool &allValid,
                                        bool verbose)
{
    if (verbose)
        std::cout << std::string(currVertexIndex, ' ') << "recurseSnapWaypoints() -------" << std::endl;

    // Find multiple nearby nodes on the graph
    std::vector<DenseVertex> graphNeighborhood;

    // Get the next state
    base::State *currentPathState = inputPath.getState(currVertexIndex);

    // Find multiple nearby nodes on the graph
    stateProperty_[queryVertex_] = currentPathState;
    std::size_t findNearestKNeighbors = 10;
    const std::size_t numSameVerticiesFound = 1;  // add 1 to the end because the NN tree always returns itself
    nn_->nearestK(queryVertex_, findNearestKNeighbors + numSameVerticiesFound, graphNeighborhood);
    stateProperty_[queryVertex_] = NULL;

    // Loop through each neighbor until one is found that connects to the previous vertex
    bool foundValidConnToPrevious = false;
    bool addedToRoadmapPath =
        false;  // track if we added a vertex to the roadmapPath, so that we can remove it later if needed
    DenseVertex candidateVertex;
    for (std::size_t neighborID = 0; neighborID < graphNeighborhood.size(); ++neighborID)
    {
        bool isValid = false;
        bool isRepeatOfPrevWaypoint = false;  // don't add current waypoint if same as last one

        // Find next state's vertex
        candidateVertex = graphNeighborhood[neighborID];

        // Check if next vertex is same as previous
        if (prevGraphVertex == candidateVertex)
        {
            // Do not do anything, we are done here
            foundValidConnToPrevious = true;
            if (verbose)
                std::cout << std::string(currVertexIndex, ' ') << "Previous vertex is same as current vertex, skipping "
                                                                  "current vertex" << std::endl;

            isValid = true;
            isRepeatOfPrevWaypoint = true;
        }
        else
        {
            // Check for collision
            isValid = si_->checkMotion(stateProperty_[prevGraphVertex], stateProperty_[candidateVertex]);

            if (visualizeSnapPath_)  // Visualize
            {
                // Show the node we're currently considering going through
                visual_->viz5State(stateProperty_[candidateVertex], /*regular, purple*/ 4, 1);
                // edge between the state on the original inputPath and its neighbor we are currently considering
                double color = 25;  // light green
                visual_->viz5Edge(currentPathState, stateProperty_[candidateVertex], color);

                color = isValid ? 75 : 100;  // orange, red
                // edge between the previous connection point we chose for the roadmapPath, and the currently considered
                // next state
                visual_->viz5Edge(stateProperty_[prevGraphVertex], stateProperty_[candidateVertex], color);

                visual_->viz5Trigger();
                usleep(visualizeSnapPathSpeed_ * 1000000);
            }

            if (isValid && verbose)  // Debug
                std::cout << std::string(currVertexIndex, ' ') << "Found valid nearby edge on loop " << neighborID
                          << std::endl;
        }

        // Remember if any connections failed
        if (isValid)
        {
            if (neighborID > 0)
            {
                if (verbose)
                    std::cout << std::string(currVertexIndex + 2, ' ') << "Found case where double loop fixed the "
                                                                         "problem - loop " << neighborID << std::endl;
                // visual_->viz5Trigger();
                // usleep(6*1000000);
            }
            foundValidConnToPrevious = true;

            // Add this waypoint solution
            if (!isRepeatOfPrevWaypoint)
            {
                // std::cout << std::string(currVertexIndex+2, ' ') << "roadmapPath.size=" << std::fixed <<
                // roadmapPath.size() << std::flush;
                // std::cout << " Vertex: " << candidateVertex;
                // std::cout << " State: " << stateProperty_[candidateVertex];
                // std::cout << std::endl;
                roadmapPath.push_back(candidateVertex);
                addedToRoadmapPath = true;  // remember it was added

                if (visualizeSnapPath_)  // Visualize
                {
                    double color = 25;  // light green
                    visual_->viz4Edge(stateProperty_[prevGraphVertex], stateProperty_[candidateVertex], color);
                    visual_->viz4Trigger();
                    usleep(visualizeSnapPathSpeed_ * 1000000);
                }
            }

            // Check if there are more points to process
            if (currVertexIndex + 1 >= inputPath.getStateCount())
            {
                if (verbose)
                    std::cout << std::string(currVertexIndex, ' ') << "END OF PATH, great job :)" << std::endl;
                allValid = true;
                return true;
            }
            else
            {
                // Recurisvely call next level
                if (recurseSnapWaypoints(inputPath, roadmapPath, currVertexIndex + 1, candidateVertex, allValid,
                                         verbose))
                {
                    return true;
                }
                else
                {
                    // Keep trying to find a working neighbor, remove last roadmapPath node if we added one
                    if (addedToRoadmapPath)
                    {
                        assert(roadmapPath.size() > 0);
                        roadmapPath.pop_back();
                        addedToRoadmapPath = false;
                    }
                }
            }
        }
        else
        {
            if (verbose)
                std::cout << std::string(currVertexIndex, ' ') << "Loop " << neighborID << " not valid" << std::endl;
        }
    } // for every neighbor

    if (!foundValidConnToPrevious)
    {
        std::cout << std::string(currVertexIndex, ' ') << "Unable to find valid connection to previous, backing up a "
                                                         "level and trying again" << std::endl;
        allValid = false;

        if (visualizeSnapPath_)  // Visualize
        {
            visual_->viz5Trigger();
            usleep(visualizeSnapPathSpeed_ * 1000000);
        }

        // TODO(davetcoleman): remove hack
        std::cout << std::string(currVertexIndex, ' ') << "TODO remove this viz" << std::endl;

        // Show the node we're currently considering going through
        visual_->viz5State(stateProperty_[prevGraphVertex], /*Medium purple, translucent outline*/ 4, 3);
        visual_->viz5Trigger();
        usleep(0.001 * 1000000);

        return false;
    }
    std::cout << std::string(currVertexIndex, ' ') << "This loop found a valid connection, but higher recursive loop "
                                                     "(one that has already returned) did not" << std::endl;

    return false;  // this loop found a valid connection, but lower recursive loop did not
}

bool otb::DenseDB::saveIfChanged(const std::string &fileName)
{
    if (graphUnsaved_)
    {
        return save(fileName);
    }
    else
        OMPL_INFORM("Not saving because database has not changed");
    return true;
}

bool otb::DenseDB::save(const std::string &fileName)
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
    OMPL_INFORM("Saving PlannerData with %d vertices %d edges", data->numVertices(), data->numEdges());

    // Write the number of paths we will be saving
    double numPaths = 1;
    outStream << numPaths;

    // Start saving each planner data object
    ompl::base::PlannerData &pd = *data.get();
    OMPL_INFORM("Saving graph with %d vertices and %d edges", pd.numVertices(), pd.numEdges());

    if (false)  // Debug
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

bool otb::DenseDB::astarSearch(const DenseVertex start, const DenseVertex goal, std::vector<DenseVertex> &vertexPath)
{
    // Hold a list of the shortest path parent to each vertex
    DenseVertex *vertexPredecessors = new DenseVertex[getNumVertices()];
    // boost::vector_property_map<DenseVertex> vertexPredecessors(getNumVertices());

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
        boost::astar_search(
            g_,     // graph
            start,  // start state
            // boost::bind(&otb::DenseDB::distanceFunction2, this, _1, goal),  // the heuristic
            // boost::bind(&otb::DenseDB::distanceFunction, this, _1, goal),  // the heuristic
            boost::bind(&otb::DenseDB::distanceFunctionTasks, this, _1, goal),  // the heuristic
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
            DenseVertex v;
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
            const DenseVertex v1 = i;
            const DenseVertex v2 = vertexPredecessors[v1];
            if (v1 != v2)
            {
                // std::cout << "Edge " << v1 << " to " << v2 << std::endl;
                visual_->viz4Edge(stateProperty_[v1], stateProperty_[v2], 10);
            }
        }
        visual_->viz4Trigger();
    }

    // Unload
    delete[] vertexPredecessors;
    delete[] vertexDistances;

    // No solution found from start to goal
    return foundGoal;
}

void otb::DenseDB::computeDensePath(const DenseVertex start, const DenseVertex goal, DensePath &path)
{
    path.clear();

    boost::vector_property_map<DenseVertex> prev(boost::num_vertices(g_));

    try
    {
        boost::astar_search(g_,                                                                 // graph
                            start,                                                              // start state
                            boost::bind(&otb::DenseDB::distanceFunctionTasks, this, _1, goal),  // the heuristic
                            boost::predecessor_map(prev).visitor(CustomAstarVisitor(goal, this)));
    }
    catch (foundGoalException &)
    {
    }

    if (prev[goal] == goal)
        OMPL_WARN("No dense path was found?");
    else
    {
        for (DenseVertex pos = goal; prev[pos] != pos; pos = prev[pos])
            path.push_front(stateProperty_[pos]);
        path.push_front(stateProperty_[start]);
    }
}

void otb::DenseDB::getAllPlannerDatas(std::vector<ompl::base::PlannerDataPtr> &plannerDatas) const
{
    base::PlannerDataPtr data(new base::PlannerData(si_));
    getPlannerData(*data);
    plannerDatas.push_back(data);  // TODO(davetcoleman): don't make second copy of this?
}

void otb::DenseDB::debugVertex(const ompl::base::PlannerDataVertex &vertex)
{
    debugState(vertex.getState());
}

void otb::DenseDB::debugState(const ompl::base::State *state)
{
    si_->printState(state, std::cout);
}

double otb::DenseDB::distanceFunction(const DenseVertex a, const DenseVertex b) const
{
    // const double dist = si_->distance(stateProperty_[a], stateProperty_[b]);
    // std::cout << "getting distance from " << a << " to " << b << " of value " << dist << std::endl;
    // return dist;
    return si_->distance(stateProperty_[a], stateProperty_[b]);
}

double otb::DenseDB::distanceFunction2(const DenseVertex a, const DenseVertex b) const
{
    // const double dist = si_->getStateSpace()->distance2(stateProperty_[a], stateProperty_[b]);
    // std::cout << "getting distance from " << a << " to " << b << " of value " << dist << std::endl;
    // return dist;
    return si_->getStateSpace()->distance2(stateProperty_[a], stateProperty_[b]);
}

double otb::DenseDB::distanceFunctionTasks(const DenseVertex a, const DenseVertex b) const
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

std::size_t otb::DenseDB::getTaskLevel(const DenseVertex &v) const
{
    return si_->getStateSpace()->getLevel(stateProperty_[v]);
}

std::size_t otb::DenseDB::getTaskLevel(const base::State *state) const
{
    return si_->getStateSpace()->getLevel(state);
}

void otb::DenseDB::initializeQueryState()
{
    if (boost::num_vertices(g_) < 1)
    {
        queryVertex_ = boost::add_vertex(g_);
        stateProperty_[queryVertex_] = NULL;
    }
}

void otb::DenseDB::getPlannerData(base::PlannerData &data) const
{
    // If there are even edges here
    if (boost::num_edges(g_) > 0)
    {
        // Adding edges and all other vertices simultaneously
        foreach (const DenseEdge e, boost::edges(g_))
        {
            const DenseVertex v1 = boost::source(e, g_);
            const DenseVertex v2 = boost::target(e, g_);

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
    foreach (const DenseVertex n, boost::vertices(g_))
        if (boost::out_degree(n, g_) == 0)
            data.addVertex(base::PlannerDataVertex(stateProperty_[n], (int)typeProperty_[n]));

    // data.properties["iterations INTEGER"] = boost::lexical_cast<std::string>(iterations_);
}

void otb::DenseDB::loadFromPlannerData(const base::PlannerData &data)
{
    // Add all vertices
    OMPL_INFORM("  Loading %u vertices into DenseDB", data.numVertices());

    std::vector<DenseVertex> idToVertex;

    // Temp disable verbose mode for loading database
    bool wasVerbose = verbose_;
    verbose_ = false;
    std::size_t debugFrequency = data.numVertices() / 10;

    // Add the nodes to the graph
    std::cout << "Vertices loaded: " << std::flush;
    for (std::size_t vertexID = 0; vertexID < data.numVertices(); ++vertexID)
    {
        if ((vertexID + 1) % debugFrequency == 0)
            std::cout << std::fixed << std::setprecision(0) << (vertexID / double(data.numVertices())) * 100.0 << "% "
                      << std::flush;

        // Get the state from loaded planner data
        const base::State *oldState = data.getVertex(vertexID).getState();
        base::State *state = si_->cloneState(oldState);

        // Get the tag, which in this application represents the vertex type
        GuardType type = static_cast<GuardType>(data.getVertex(vertexID).getTag());

        // ADD GUARD
        idToVertex.push_back(addVertex(state, type));
    }
    std::cout << std::endl;

    OMPL_INFORM("  Loading %u edges into DenseDB", data.numEdges());
    // Add the corresponding edges to the graph
    std::vector<unsigned int> edgeList;
    std::cout << "Edges loaded: " << std::flush;
    for (unsigned int fromVertex = 0; fromVertex < data.numVertices(); ++fromVertex)
    {
        if ((fromVertex + 1) % debugFrequency == 0)
            std::cout << std::fixed << std::setprecision(0) << fromVertex / double(data.numVertices()) * 100.0 << "% "
                      << std::flush;

        edgeList.clear();

        // Get the edges
        data.getEdges(fromVertex, edgeList);  // returns the id of each edge

        DenseVertex v1 = idToVertex[fromVertex];

        // Process edges
        for (std::size_t edgeId = 0; edgeId < edgeList.size(); ++edgeId)
        {
            unsigned int toVertex = edgeList[edgeId];
            DenseVertex v2 = idToVertex[toVertex];

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

void otb::DenseDB::generateGrid()
{
    OMPL_INFORM("Generating grid");

    if (!si_->isSetup())
    {
        OMPL_WARN("Space information setup was not yet called. Calling now.");
        si_->setup();
    }

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

    // Error check
    if (getNumVertices() < 2)
    {
        OMPL_ERROR("No vertices generated, failing");
        exit(-1);
    }

    // Remove vertices in collision using multithreading
    // TODO enable
    // std::vector<DenseVertex> unvalidatedVertices;
    // checkVerticesThreaded(unvalidatedVertices);

    {
        // Benchmark runtime
        time::point start_time = time::now();

        // Create edges ----------------------------------------
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
        visual_->viz1Trigger();

    // Mark the graph as ready to be saved
    graphUnsaved_ = true;
}

void otb::DenseDB::generateTaskSpace()
{
    OMPL_INFORM("Generating task space");
    std::vector<DenseVertex> vertexToNewVertex(getNumVertices());

    OMPL_INFORM("Adding task space vertices");
    foreach (DenseVertex v, boost::vertices(g_))
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
        DenseVertex vNew = addVertex(newState, START);

        // Map old vertex to new vertex
        vertexToNewVertex[v] = vNew;

        if (visualizeGridGeneration_)  // Visualize - only do this for 2/3D environments
        {
            visual_->viz1State(newState, /*mode=*/5, 1);  // Candidate node has already (just) been added
            visual_->viz1Trigger();
            usleep(0.001 * 1000000);
        }
    }

    // Add Edges
    OMPL_INFORM("Adding task space edges");

    // Cache all current edges before adding new ones
    std::vector<DenseEdge> edges(getNumEdges());
    std::size_t edgeID = 0;
    foreach (const DenseEdge e, boost::edges(g_))
    {
        edges[edgeID++] = e;
    }

    // Copy every edge
    for (std::size_t i = 0; i < edges.size(); ++i)
    {
        const DenseEdge &e = edges[i];

        const DenseVertex v1 = boost::source(e, g_);
        const DenseVertex v2 = boost::target(e, g_);

        // std::cout << "v1: " << v1 << " v2: " << v2 << std::endl;
        // std::cout << "v1': " << vertexToNewVertex[v1] << " v2': " << vertexToNewVertex[v2] << std::endl;

        DenseEdge newE = addEdge(vertexToNewVertex[v1], vertexToNewVertex[v2], edgeWeightProperty_[e]);

        if (visualizeGridGeneration_)  // Visualize
        {
            viz1Edge(newE);
            if (i % 100 == 0)
            {
                visual_->viz1Trigger();
                usleep(0.001 * 1000000);
            }
        }
    }

    if (visualizeGridGeneration_)  // Visualize
        visual_->viz1Trigger();

    OMPL_INFORM("Done generating task space graph");
}

std::size_t otb::DenseDB::getEdgesPerVertex()
{
    // in 2D this creates the regular square with diagonals of 8 edges
    if (si_->getStateSpace()->getDimension() == 3)
    {
        return 8;
    }

    // full robot
    return si_->getStateSpace()->getDimension() * 2;
}

void otb::DenseDB::generateEdges()
{
    OMPL_INFORM("Generating edges");
    bool verbose = false;

    // Benchmark runtime
    time::point startTime = time::now();
    std::size_t count = 0;

    // Nearest Neighbor search
    std::vector<DenseVertex> graphNeighborhood;
    std::vector<DenseEdge> unvalidatedEdges;
    std::size_t feedbackFrequency = std::max(static_cast<int>(getNumVertices() / 10), 10);

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

        // How many edges should each vertex connect with?
        std::size_t findNearestKNeighbors = getEdgesPerVertex();
        const std::size_t numSameVerticiesFound = 1;  // add 1 to the end because the NN tree always returns itself

        // Search
        stateProperty_[queryVertex_] = stateProperty_[v1];
        nn_->nearestK(queryVertex_, findNearestKNeighbors + numSameVerticiesFound, graphNeighborhood);
        stateProperty_[queryVertex_] = NULL;  // Set search vertex to NULL to prevent segfault on class unload of memory

        if (verbose)
            OMPL_INFORM("Found %u neighbors", graphNeighborhood.size());

        // For each nearby vertex, add an edge
        std::size_t errorCheckNumSameVerticies = 0;  // sanity check
        for (std::size_t i = 0; i < graphNeighborhood.size(); ++i)
        {
            if (verbose)
                OMPL_INFORM("Edge %u", i);

            DenseVertex &v2 = graphNeighborhood[i];

            // Check if these vertices are the same
            if (v1 == v2)
            {
                errorCheckNumSameVerticies++;  // sanity check
                continue;
            }

            // Check if these vertices already share an edge
            if (boost::edge(v1, v2, g_).second)
                continue;

            if (visualizeGridGeneration_)  // Debug: display edge
                visual_->viz1Edge(stateProperty_[v1], stateProperty_[v2], 1);

            // Create edge - maybe removed later
            DenseEdge e = addEdge(v1, v2, desiredAverageCost_);
            unvalidatedEdges.push_back(e);

            count++;
        }  // for each v2

        // Make sure one and only one vertex is returned from the NN search that is the same as parent vertex
        assert(errorCheckNumSameVerticies == 1);

        // Visualize
        if (visualizeGridGeneration_)
        {
            visual_->viz1Trigger();
            usleep(0.001 * 1000000);
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
    foreach (const DenseEdge e, boost::edges(g_))
    {
        const DenseVertex &v1 = boost::source(e, g_);
        const DenseVertex &v2 = boost::target(e, g_);

        // Determine cost for edge depending on mode
        double cost;
        if (popularityBiasEnabled_)
        {
            cost = MAX_POPULARITY_WEIGHT;
        }
        else
        {
            // cost = distanceFunction2(v1, v2);
            cost = distanceFunction(v1, v2);
        }
        edgeWeightProperty_[e] = cost;
        errorCheckCounter++;

        if (visualizeGridGeneration_)  // Debug in Rviz
        {
            visual_->viz1Edge(stateProperty_[v1], stateProperty_[v2], edgeWeightProperty_[e]);
            if (errorCheckCounter % 100 == 0)
            {
                visual_->viz1Trigger();
                usleep(0.001 * 1000000);
            }
        }
    }
    assert(errorCheckCounter == numEdgesAfterCheck);
}

void otb::DenseDB::checkEdges()  // TODO: deprecated, remove this func
{
    OMPL_WARN("Collision checking generated edges without threads");
    OMPL_ERROR("Has not been updated for remove_edge");

    foreach (const DenseEdge e, boost::edges(g_))
    {
        const DenseVertex &v1 = boost::source(e, g_);
        const DenseVertex &v2 = boost::target(e, g_);

        // Remove any edges that are in collision
        if (!si_->checkMotion(stateProperty_[v1], stateProperty_[v2]))
        {
            edgeCollisionStateProperty_[e] = IN_COLLISION;
        }
        else
        {
            edgeCollisionStateProperty_[e] = FREE;
        }
    }
}

void otb::DenseDB::checkEdgesThreaded(const std::vector<DenseEdge> &unvalidatedEdges)
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
            boost::bind(&otb::DenseDB::checkEdgesThread, this, startEdge, endEdge, si, unvalidatedEdges));
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

    // Sanity check: make sure all remaining edges were validated
    foreach (const DenseEdge e, boost::edges(g_))
    {
        if (edgeCollisionStateProperty_[e] != FREE)
        {
            OMPL_ERROR("Remaining edge %u has not been marked free", e);
        }
    }
}

void otb::DenseDB::checkEdgesThread(std::size_t startEdge, std::size_t endEdge, base::SpaceInformationPtr si,
                                    const std::vector<DenseEdge> &unvalidatedEdges)
{
    // Process [startEdge, endEdge] inclusive
    for (std::size_t edgeID = startEdge; edgeID <= endEdge; ++edgeID)
    {
        const DenseEdge &e = unvalidatedEdges[edgeID];

        const DenseVertex &v1 = boost::source(e, g_);
        const DenseVertex &v2 = boost::target(e, g_);

        // Remove any edges that are in collision
        if (!si->checkMotion(stateProperty_[v1], stateProperty_[v2]))
        {
            boost::remove_edge(v1, v2, g_);
        }
        else
        {
            edgeCollisionStateProperty_[e] = FREE;
        }
    }
}

void otb::DenseDB::recursiveDiscretization(std::vector<double> &values, std::size_t joint_id, std::size_t desiredDepth)
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
        if (joint_id < desiredDepth - 1)
        {
            // Keep recursing
            recursiveDiscretization(values, joint_id + 1, desiredDepth);
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
                // Candidate node has already (just) been added
                visual_->viz1State(nextDiscretizedState_, /*mode=*/5, 1);
                visual_->viz1Trigger();
                usleep(0.001 * 1000000);
            }

            // Prepare for next new state by allocating now
            // nextDiscretizedState_ = si_->getStateSpace()->allocState();
        }
    }
}

void otb::DenseDB::checkVerticesThreaded(const std::vector<DenseVertex> &unvalidatedVertices)
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
            boost::bind(&otb::DenseDB::checkVerticesThread, this, startVertex, endVertex, si, unvalidatedVertices));
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

void otb::DenseDB::checkVerticesThread(std::size_t startVertex, std::size_t endVertex, base::SpaceInformationPtr si,
                                       const std::vector<DenseVertex> &unvalidatedVertices)
{
    // Process [startVertex, endVertex] inclusive
    for (std::size_t vertexID = startVertex; vertexID <= endVertex; ++vertexID)
    {
        const DenseVertex &v = unvalidatedVertices[vertexID];

        // Remove any vertices that are in collision
        if (!si_->isValid(stateProperty_[v]))
        {
            boost::remove_vertex(v, g_);
            std::cout << "found vertex in collision " << std::endl;
        }
    }
}

void otb::DenseDB::clearEdgeCollisionStates()
{
    foreach (const DenseEdge e, boost::edges(g_))
        edgeCollisionStateProperty_[e] = NOT_CHECKED;  // each edge has an unknown state
}

void otb::DenseDB::displayDatabase(bool showVertices)
{
    OMPL_INFORM("Displaying database");

    // Error check
    if (getNumVertices() == 0 || getNumEdges() == 0)
    {
        OMPL_ERROR("Unable to show database because no vertices/edges available");
        exit(-1);
    }

    // Clear old database
    visual_->viz1State(NULL, /*deleteAllMarkers*/ 0, 0);

    // if (showVertices)
    {
        // Loop through each vertex
        std::size_t count = 0;
        std::size_t debugFrequency = std::min(10000, static_cast<int>(getNumEdges() / 10));
        std::cout << "Displaying vertices: " << std::flush;
        foreach (DenseVertex v, boost::vertices(g_))
        {
            // Check for null states
            if (stateProperty_[v])
            {
                visual_->viz1State(stateProperty_[v], /*mode=*/6, 1);
            }

            // Prevent cache from getting too big
            if (count % debugFrequency == 0)
            {
                std::cout << std::fixed << std::setprecision(0)
                          << (static_cast<double>(count + 1) / getNumVertices()) * 100.0 << "% " << std::flush;
                visual_->viz1Trigger();
            }
            count++;
        }
        std::cout << std::endl;
    }
    // else
    {
        // Loop through each edge
        std::size_t count = 0;
        std::size_t debugFrequency = std::min(10000, static_cast<int>(getNumEdges() / 10));
        std::cout << "Displaying edges: " << std::flush;
        foreach (DenseEdge e, boost::edges(g_))
        {
            // Add edge
            const DenseVertex &v1 = boost::source(e, g_);
            const DenseVertex &v2 = boost::target(e, g_);

            // Visualize
            assert(edgeWeightProperty_[e] <= MAX_POPULARITY_WEIGHT);
            visual_->viz1Edge(stateProperty_[v1], stateProperty_[v2], edgeWeightProperty_[e]);

            // Prevent cache from getting too big
            if (count % debugFrequency == 0)
            {
                std::cout << std::fixed << std::setprecision(0)
                          << (static_cast<double>(count + 1) / getNumEdges()) * 100.0 << "% " << std::flush;
                visual_->viz1Trigger();
            }

            count++;
        }
        std::cout << std::endl;
    }

    // Publish remaining edges
    visual_->viz1Trigger();
}

void otb::DenseDB::normalizeGraphEdgeWeights()
{
    bool verbose = false;

    if (!popularityBiasEnabled_)
    {
        OMPL_INFORM("Skipping normalize graph edge weights because not using popularity bias currently");
        return;
    }

    // Normalize weight of graph
    double total_cost = 0;
    foreach (DenseEdge e, boost::edges(g_))
    {
        total_cost += edgeWeightProperty_[e];
    }
    double avg_cost = total_cost / getNumEdges();

    if (verbose)
        OMPL_INFORM("Average cost of the edges in graph is %f", avg_cost);

    if (avg_cost < desiredAverageCost_)  // need to decrease cost in graph
    {
        double avgCostDiff = desiredAverageCost_ - avg_cost;

        if (verbose)
            std::cout << "avgCostDiff: " << avgCostDiff << std::endl;
        double perEdgeReduction = avgCostDiff;  // / getNumEdges();

        if (verbose)
            OMPL_INFORM("Decreasing each edge's cost by %f", perEdgeReduction);
        foreach (DenseEdge e, boost::edges(g_))
        {
            assert(edgeWeightProperty_[e] <= MAX_POPULARITY_WEIGHT);
            edgeWeightProperty_[e] = std::min(edgeWeightProperty_[e] + perEdgeReduction, MAX_POPULARITY_WEIGHT);
            if (verbose)
                std::cout << "Edge " << e << " now has weight " << edgeWeightProperty_[e] << " via reduction "
                          << perEdgeReduction << std::endl;
        }
    }
    else if (verbose)
    {
        OMPL_INFORM("Not decreasing all edge's cost because average is above desired");
    }
}

otb::DenseVertex otb::DenseDB::addVertex(base::State *state, const GuardType &type)
{
    // Create vertex
    DenseVertex v1 = boost::add_vertex(g_);

    // Add properties
    typeProperty_[v1] = type;
    stateProperty_[v1] = state;
    // state3Property_[v1] = state;
    representativesProperty_[v1] = 0;

    // Add vertex to nearest neighbor structure
    nn_->add(v1);

    // Track vertex for later removal if temporary
    if (type == CARTESIAN)
    {
        tempVerticies_.push_back(v1);
    }

    return v1;
}

otb::DenseEdge otb::DenseDB::addEdge(const DenseVertex &v1, const DenseVertex &v2, const double weight)
{
    // Error check
    assert(v2 <= getNumVertices());
    assert(v1 <= getNumVertices());

    // Create the new edge
    DenseEdge e = (boost::add_edge(v1, v2, g_)).first;

    // std::cout << "Adding cost: " << weight << std::endl;

    // Add associated properties to the edge
    edgeWeightProperty_[e] = weight;
    // edgeWeightProperty_[e] = distanceFunction2(v1, v2);
    // edgeWeightProperty_[e] = distanceFunction(v1, v2);
    edgeCollisionStateProperty_[e] = NOT_CHECKED;

    return e;
}

void otb::DenseDB::cleanupTemporaryVerticies()
{
    const bool verbose = false;

    if (tempVerticies_.empty())
    {
        OMPL_INFORM("Skipping verticies cleanup - no middle cartesian layer verticies found");
        return;
    }

    OMPL_INFORM("Cleaning up temp verticies - vertex count: %u, edge count: %u", getNumVertices(), getNumEdges());
    BOOST_REVERSE_FOREACH(DenseVertex v, tempVerticies_)
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

bool otb::DenseDB::addCartPath(std::vector<base::State *> path)
{
    // Error check
    if (path.size() < 2)
    {
        OMPL_ERROR("Invalid cartesian path - too few states");
        exit(-1);
    }
    // TODO: check for validity

    // Create verticies for the extremas
    DenseVertex startVertex = addVertex(path.front(), CARTESIAN);
    DenseVertex goalVertex = addVertex(path.back(), CARTESIAN);

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
    DenseVertex v1 = startVertex;
    DenseVertex v2;
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
            visual_->viz1Edge(path[i - 1], path[i], 0);
            visual_->viz1State(path[i], /*mode=*/1, 1);
            visual_->viz1Trigger();
            usleep(0.001 * 1000000);
        }
    }

    return true;
}

bool otb::DenseDB::connectStateToNeighborsAtLevel(const DenseVertex &fromVertex, const std::size_t level,
                                                  DenseVertex &minConnectorVertex)
{
    // Get nearby states to goal
    std::vector<DenseVertex> neighbors;
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
    foreach (DenseVertex v, neighbors)
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
            visual_->viz1Edge(stateProperty_[v], stateProperty_[fromVertex], cost);
            visual_->viz1State(stateProperty_[v], /*mode=*/1, 1);
            visual_->viz1Trigger();
            usleep(0.001 * 1000000);
        }
    }

    // Display ---------------------------------------
    if (visualizeCartNeighbors_)
        visual_->viz1Trigger();

    return true;
}

void otb::DenseDB::findGraphNeighbors(const DenseVertex &denseV, std::vector<DenseVertex> &graphNeighborhood,
                                      std::vector<DenseVertex> &visibleNeighborhood, double searchRadius,
                                      std::size_t coutIndent)
{
    findGraphNeighbors(stateProperty_[denseV], graphNeighborhood, visibleNeighborhood, searchRadius, coutIndent);
}

void otb::DenseDB::findGraphNeighbors(base::State* state, std::vector<DenseVertex> &graphNeighborhood,
                                      std::vector<DenseVertex> &visibleNeighborhood, double searchRadius,
                                      std::size_t coutIndent)
{
    bool verbose = false;
    if (verbose)
        std::cout << std::string(coutIndent, ' ') << "findGraphNeighbors()" << std::endl;

    // Search
    stateProperty_[queryVertex_] = state;
    nn_->nearestR(queryVertex_, searchRadius, graphNeighborhood);
    stateProperty_[queryVertex_] = NULL;

    // Now that we got the neighbors from the NN, we must remove any we can't see
    for (std::size_t i = 0; i < graphNeighborhood.size(); ++i)
    {
        if (si_->checkMotion(state, stateProperty_[graphNeighborhood[i]]))
        {
            visibleNeighborhood.push_back(graphNeighborhood[i]);
        }
    }

    if (verbose)
        std::cout << std::string(coutIndent + 2, ' ') << "Graph neighborhood: " << graphNeighborhood.size()
                  << " | Visible neighborhood: " << visibleNeighborhood.size() << std::endl;
}

void otb::DenseDB::getNeighborsAtLevel(const base::State *origState, const std::size_t level,
                                       const std::size_t kNeighbors, std::vector<DenseVertex> &neighbors)
{
    // Clone the state and change its level
    base::State *searchState = si_->cloneState(origState);
    si_->getStateSpace()->setLevel(searchState, level);

    // Get nearby state
    stateProperty_[queryVertex_] = searchState;
    nn_->nearestK(queryVertex_, kNeighbors, neighbors);
    stateProperty_[queryVertex_] = NULL;  // Set search vertex to NULL to prevent segfault on class unload of memory
    si_->getStateSpace()->freeState(searchState);

    // Run various checks
    for (std::size_t i = 0; i < neighbors.size(); ++i)
    {
        const DenseVertex &nearVertex = neighbors[i];

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

void otb::DenseDB::viz1Edge(DenseEdge &e)
{
    const DenseVertex &v1 = boost::source(e, g_);
    const DenseVertex &v2 = boost::target(e, g_);

    // Visualize
    visual_->viz1Edge(stateProperty_[v1], stateProperty_[v2], edgeWeightProperty_[e]);
}

bool otb::DenseDB::checkTaskPathSolution(og::PathGeometric &path, ob::State *start, ob::State *goal)
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

void otb::DenseDB::checkStateType()
{
    std::size_t count = 0;
    foreach (const DenseVertex v, boost::vertices(g_))
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

void otb::DenseDB::connectNewVertex(DenseVertex v1)
{
    bool verbose = false;

    // Visualize new vertex
    if (visualizeAddSample_)
    {
        visual_->viz1State(stateProperty_[v1], /*mode=*/1, 1);
    }

    // Now connect to nearby vertices
    std::vector<DenseVertex> graphNeighborhood;
    std::size_t findNearestKNeighbors = getEdgesPerVertex();
    OMPL_INFORM("connectNewVertex(): Finding %u nearest neighbors for new vertex", findNearestKNeighbors);
    const std::size_t numSameVerticiesFound = 1;  // add 1 to the end because the NN tree always returns itself

    // Search
    stateProperty_[queryVertex_] = stateProperty_[v1];
    nn_->nearestK(queryVertex_, findNearestKNeighbors + numSameVerticiesFound, graphNeighborhood);
    stateProperty_[queryVertex_] = NULL;  // Set search vertex to NULL to prevent segfault on class unload of memory

    // For each nearby vertex, add an edge
    std::size_t errorCheckNumSameVerticies = 0;  // sanity check
    for (std::size_t i = 0; i < graphNeighborhood.size(); ++i)
    {
        DenseVertex &v2 = graphNeighborhood[i];

        // Check if these vertices are the same
        if (v1 == v2)
        {
            if (verbose)
                std::cout << "connectNewVertex attempted to connect edge to itself: " << v1 << ", " << v2 << std::endl;
            errorCheckNumSameVerticies++;  // sanity check
            continue;
        }

        // Check if these vertices are the same STATE
        if (si_->getStateSpace()->equalStates(stateProperty_[v1], stateProperty_[v2]))
        {
            OMPL_ERROR("This state has already been added, should not happen");
            exit(-1);
        }

        // Check if these vertices already share an edge
        if (boost::edge(v1, v2, g_).second)
        {
            std::cout << "skipped bc already share an edge " << std::endl;
            continue;
        }

        // Create edge if not in collision
        if (si_->checkMotion(stateProperty_[v1], stateProperty_[v2]))
        {
            DenseEdge e = addEdge(v1, v2, desiredAverageCost_);
            std::cout << "added valid edge " << e << std::endl;

            if (visualizeAddSample_)  // Debug: display edge
            {
                double popularity = 100;  // TODO: maybe make edge really popular so we can be sure its added to the
                                          // spars graph since we need it
                visual_->viz1Edge(stateProperty_[v1], stateProperty_[v2], popularity);
            }
        }

    }  // for each neighbor

    // Make sure one and only one vertex is returned from the NN search that is the same as parent vertex
    BOOST_ASSERT_MSG(errorCheckNumSameVerticies <= numSameVerticiesFound, "Too many same verticies found ");

    // Visualize
    if (visualizeAddSample_)
    {
        visual_->viz1Trigger();
        usleep(0.001 * 1000000);
    }

    // Record this new addition
    graphUnsaved_ = true;
}
