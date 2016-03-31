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
#include <ompl/tools/bolt/SparseDB.h>
#include <ompl/base/ScopedState.h>
#include <ompl/util/Time.h>
#include <ompl/util/Console.h>
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>  // TODO: remove, this is not space agnostic

// Boost
#include <boost/graph/incremental_components.hpp>
#include <boost/foreach.hpp>
#include <boost/unordered_set.hpp>

// C++
#include <limits>
#include <queue>

// Allow hooks for visualizing planner
#define OMPL_BOLT_DEBUG

namespace og = ompl::geometric;
namespace ot = ompl::tools;
namespace otb = ompl::tools::bolt;
namespace ob = ompl::base;

// edgeWeightMap methods ////////////////////////////////////////////////////////////////////////////

BOOST_CONCEPT_ASSERT((boost::ReadablePropertyMapConcept<otb::SparseDB::edgeWeightMap, otb::SparseEdge>));

otb::SparseDB::edgeWeightMap::edgeWeightMap(const SparseGraph &graph, const SparseEdgeCollisionStateMap &collisionStates,
                                            const double &popularityBias, const bool popularityBiasEnabled)
  : g_(graph)
  , collisionStates_(collisionStates)
  , popularityBias_(popularityBias)
  , popularityBiasEnabled_(popularityBiasEnabled)
{
}

double otb::SparseDB::edgeWeightMap::get(SparseEdge e) const
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
double get(const otb::SparseDB::edgeWeightMap &m, const otb::SparseEdge &e)
{
    return m.get(e);
}
}

// CustomAstarVisitor methods ////////////////////////////////////////////////////////////////////////////

BOOST_CONCEPT_ASSERT((boost::AStarVisitorConcept<otb::SparseDB::CustomAstarVisitor, otb::SparseGraph>));

otb::SparseDB::CustomAstarVisitor::CustomAstarVisitor(SparseVertex goal, SparseDB *parent)
  : goal_(goal), parent_(parent)
{
}

void otb::SparseDB::CustomAstarVisitor::discover_vertex(SparseVertex v, const SparseGraph &) const
{
    if (parent_->visualizeAstar_)
        parent_->visual_->viz1StateCallback(parent_->statePropertySparse_[v], /*mode=*/1, 1);
}

void otb::SparseDB::CustomAstarVisitor::examine_vertex(SparseVertex v, const SparseGraph &) const
{
    if (parent_->visualizeAstar_)
    {
        parent_->visual_->viz1StateCallback(parent_->statePropertySparse_[v], /*mode=*/5, 1);
        parent_->visual_->viz1TriggerCallback();
        usleep(parent_->visualizeAstarSpeed_ * 1000000);
    }

    if (v == goal_)
        throw foundGoalException();
}

// Actual class ////////////////////////////////////////////////////////////////////////////

otb::SparseDB::SparseDB(base::SpaceInformationPtr si, BoltDB *boltDB, VisualizerPtr visual)
  : si_(si)
  , boltDB_(boltDB)
  , visual_(visual)
  // Property accessors of edges
  , edgeWeightPropertySparse_(boost::get(boost::edge_weight, g_))
  , edgeCollisionStatePropertySparse_(boost::get(edge_collision_state_t(), g_))
  // Property accessors of vertices
  , statePropertySparse_(boost::get(vertex_state_t(), g_))
  , typePropertySparse_(boost::get(vertex_type_t(), g_))
  // interfaceDataProperty_(boost::get(vertex_interface_data_t(), g_)),
  // Disjoint set accessors
  , disjointSets_(boost::get(boost::vertex_rank, g_), boost::get(boost::vertex_predecessor, g_))
  , verbose_(true)
  , visualizeAstar_(false)
  , visualizeSparsCreation_(false)
  , sparseDelta_(2.0)
  , visualizeAstarSpeed_(0.1)
{
    // Initialize nearest neighbor datastructure
    nn_.reset(new NearestNeighborsGNATNoThreadSafety<SparseVertex>());
    nn_->setDistanceFunction(boost::bind(&otb::SparseDB::distanceFunction, this, _1, _2));

    // Add search state
    initializeQueryState();
}

otb::SparseDB::~SparseDB(void)
{
    freeMemory();
}

void otb::SparseDB::freeMemory()
{
    BOOST_FOREACH (SparseVertex v, boost::vertices(g_))
    {
        // BOOST_FOREACH (InterfaceData &d, interfaceDataProperty_[v].interfaceHash | boost::adaptors::map_values)
        // d.clear(si_);
        if (statePropertySparse_[v] != NULL)
            si_->freeState(statePropertySparse_[v]);
        statePropertySparse_[v] = NULL;  // TODO(davetcoleman): is this needed??
    }
    g_.clear();

    if (nn_)
        nn_->clear();
}

bool otb::SparseDB::setup()
{
    return true;
}

bool otb::SparseDB::astarSearch(const SparseVertex start, const SparseVertex goal,
                                std::vector<SparseVertex> &vertexPath)
{
    // Hold a list of the shortest path parent to each vertex
    SparseVertex *vertexPredecessors = new SparseVertex[getNumVertices()];
    // boost::vector_property_map<SparseVertex> vertexPredecessors(getNumVertices());

    bool foundGoal = false;
    double *vertexDistances = new double[getNumVertices()];

    OMPL_INFORM("Beginning AStar Search");
    try
    {
        double popularityBias = 0;
        bool popularityBiasEnabled = false;
        // Note: could not get astar_search to compile within BoltRetrieveRepair.cpp class because of namespacing issues
        boost::astar_search(g_,     // graph
                            start,  // start state
                            // boost::bind(&otb::SparseDB::distanceFunction2, this, _1, goal),  // the heuristic
                            boost::bind(&otb::SparseDB::distanceFunction, this, _1, goal),  // the heuristic
                            // ability to disable edges (set cost to inifinity):
                            boost::weight_map(edgeWeightMap(g_, edgeCollisionStatePropertySparse_, popularityBias,
                                                            popularityBiasEnabled))
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
            SparseVertex v;
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
            const SparseVertex v1 = i;
            const SparseVertex v2 = vertexPredecessors[v1];
            if (v1 != v2)
            {
                // std::cout << "Edge " << v1 << " to " << v2 << std::endl;
                visual_->viz1EdgeCallback(statePropertySparse_[v1], statePropertySparse_[v2], 10);
            }
        }
        visual_->viz1TriggerCallback();
    }

    // Unload
    delete[] vertexPredecessors;
    delete[] vertexDistances;

    // No solution found from start to goal
    return foundGoal;
}

void otb::SparseDB::debugVertex(const ompl::base::PlannerDataVertex &vertex)
{
    debugState(vertex.getState());
}

void otb::SparseDB::debugState(const ompl::base::State *state)
{
    si_->printState(state, std::cout);
}

double otb::SparseDB::distanceFunction(const SparseVertex a, const SparseVertex b) const
{
    // const double dist = si_->distance(statePropertySparse_[a], statePropertySparse_[b]);
    // std::cout << "getting distance from " << a << " to " << b << " of value " << dist << std::endl;
    // return dist;
    return si_->distance(statePropertySparse_[a], statePropertySparse_[b]);
}

void otb::SparseDB::initializeQueryState()
{
    if (boost::num_vertices(g_) < 1)
    {
        queryVertex_ = boost::add_vertex(g_);
        statePropertySparse_[queryVertex_] = NULL;
    }
}

void otb::SparseDB::clearEdgeCollisionStates()
{
    BOOST_FOREACH (const SparseEdge e, boost::edges(g_))
        edgeCollisionStatePropertySparse_[e] = NOT_CHECKED;  // each edge has an unknown state
}

// Create SPARs graph using popularity
void otb::SparseDB::createSPARS()
{
    bool verbose = false;

    // Sort the verticies by popularity in a queue
    std::priority_queue<BoltDB::WeightedVertex, std::vector<BoltDB::WeightedVertex>, BoltDB::CompareWeightedVertex>
        pqueue;

    // Loop through each popular edge in the dense graph
    BOOST_FOREACH (DenseVertex v, boost::vertices(boltDB_->g_))
    {
        // Do not process the search vertex, it is null
        if (v == 0)
            continue;

        if (verbose)
            std::cout << "Vertex: " << v << std::endl;
        double popularity = 0;
        // std::pair<out_edge_iterator, out_edge_iterator> edge
        BOOST_FOREACH (DenseEdge edge, boost::out_edges(v, boltDB_->g_))
        {
            if (verbose)
                std::cout << "  Edge: " << edge << std::endl;
            popularity += (100 - boltDB_->edgeWeightProperty_[edge]);
        }
        if (verbose)
            std::cout << "  Total popularity: " << popularity << std::endl;
        pqueue.push(BoltDB::WeightedVertex(v, popularity));
    }

    double largestWeight = pqueue.top().weight_;
    std::vector<DenseVertex> failedInsertVertices;

    // Output the vertices in order
    while (!pqueue.empty())
    {
        DenseVertex v = pqueue.top().v_;

        // Don't use unpopular nodes
        // if (pqueue.size() < boltDB_->getNumVertices() / 2.0)
        // break;

        // Visualize
        if (visualizeSparsCreation_)
        {
            const double weightPercent = pqueue.top().weight_ / largestWeight * 100.0;
            // std::cout << "  Visualizing vertex " << v << " with popularity " << weightPercent
            //<< " queue remaining size " << pqueue.size() << std::endl;
            visual_->viz1StateCallback(boltDB_->stateProperty_[v], /*mode=*/7, weightPercent);
            visual_->viz1TriggerCallback();
            usleep(0.01 * 1000000);
        }

        // Attempt to insert into SPARS graph
        double seconds = 1000;
        ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(seconds, 0.1);

        // Run SPARS checks
        if (!addStateToRoadmap(ptc))  //, v))
        {
            std::cout << "Failed to add state to roadmap------" << std::endl;
            failedInsertVertices.push_back(v);
        }

        // Remove from priority queue
        pqueue.pop();
    }  // end while

    // Attempt to re-insert the failed vertices
    bool succeededInInserting = true;
    std::size_t loopAttempt = 2;
    while (succeededInInserting)
    {
        std::cout << "-------------------------------------------------------" << std::endl;
        std::cout << "Attempting to re-insert " << failedInsertVertices.size() << " vertices for the " << loopAttempt
                  << " loop" << std::endl;
        usleep(1 * 1000000);

        // TODO: is the wrapped while loop ever necessary? or do we just need to attempt twice?
        assert(loopAttempt < 5);  // just curious if this ever trips

        succeededInInserting = false;
        std::size_t sucessfulReinsertions = 0;
        for (std::size_t i = 0; i < failedInsertVertices.size(); ++i)
        {
            // Attempt to insert into SPARS graph
            double seconds = 1000;
            ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(seconds, 0.1);

            // Run SPARS checks
            if (!addStateToRoadmap(ptc))  //, failedInsertVertices[i]))
            {
                std::cout << "Failed AGAIN to add state to roadmap------" << std::endl;
            }
            else
            {
                failedInsertVertices.erase(failedInsertVertices.begin() + i);
                i--;
                sucessfulReinsertions++;
                succeededInInserting = true;
            }
        }
        std::cout << "Succeeded in inserting " << sucessfulReinsertions << " vertices on the " << loopAttempt << " loop"
                  << std::endl;
        loopAttempt++;
    }
}

bool otb::SparseDB::addStateToRoadmap(const base::PlannerTerminationCondition &ptc)  //, DenseVertex denseVertex)
{
    OMPL_ERROR("need to add second arg");
    DenseVertex denseVertex;  // TODO
    std::size_t coutIndent = 0;
    if (verbose_)
        std::cout << "addStateToRoadmap()" << std::endl;

    bool stateAdded = false;

    // Deep copy
    // base::State *qNew = si_->cloneState(newState);  // TODO(davetcoleman): do i need to clone it?
    // base::State *workState = si_->allocState();     // TODO(davetcoleman): do i need this state?

    base::State *qNew = boltDB_->stateProperty_[denseVertex];

    /* Nodes near our input state */
    std::vector<SparseVertex> graphNeighborhood;
    /* Visible nodes near our input state */
    std::vector<SparseVertex> visibleNeighborhood;

    // Find nearby nodes
    findGraphNeighbors(qNew, graphNeighborhood, visibleNeighborhood, coutIndent);

    // Always add a node if no other nodes around it are visible (GUARD)
    if (checkAddCoverage(qNew, visibleNeighborhood, coutIndent + 4))
    {
        stateAdded = true;
    }
    else if (checkAddConnectivity(qNew, visibleNeighborhood, coutIndent + 8))  // Connectivity criterion
    {
        stateAdded = true;
    }
    else if (checkAddInterface(qNew, graphNeighborhood, visibleNeighborhood, coutIndent + 12))
    {
        stateAdded = true;
    }
    // else if (checkAsymptoticOptimal(denseVertex, coutIndent+16))
    // {
    //     stateAdded = true;
    // }
    /*
    else
    {
        if (verbose_)
            OMPL_INFORM(" ---- Ensure SPARS asymptotic optimality");
        if (visibleNeighborhood.size() > 0)
        {
            std::map<SparseVertex, base::State*> closeRepresentatives;
            if (verbose_)
                OMPL_INFORM(" ----- findCloseRepresentatives()");

            findCloseRepresentatives(workState, qNew, visibleNeighborhood[0], closeRepresentatives, ptc);
            if (verbose_)
                OMPL_INFORM("------ Found %d close representatives", closeRepresentatives.size());

            for (std::map<SparseVertex, base::State*>::iterator it = closeRepresentatives.begin(); it !=
    closeRepresentatives.end(); ++it)
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

            for (std::map<SparseVertex, base::State*>::iterator it = closeRepresentatives.begin(); it !=
    closeRepresentatives.end(); ++it)
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

    // si_->freeState(workState);
    // si_->freeState(qNew);

    return stateAdded;
}

otb::SparseVertex otb::SparseDB::addVertex(base::State *state, const GuardType &type)
{
    // Create vertex
    SparseVertex v = boost::add_vertex(g_);

    // Add properties
    typePropertySparse_[v] = type;
    statePropertySparse_[v] = state;

    // Connected component tracking
    disjointSets_.make_set(v);

    // Add vertex to nearest neighbor structure
    nn_->add(v);

    // Visualize
    if (visualizeSparsCreation_)
    {
        // visual_->viz2StateCallback(statePropertySparse_[v], /*mode=*/ getVizVertexType(type), sparseDelta_);
        visual_->viz2StateCallback(statePropertySparse_[v], /*mode=*/4, sparseDelta_);
        visual_->viz2TriggerCallback();
        usleep(0.01 * 1000000);
    }

    return v;
}

std::size_t otb::SparseDB::getVizVertexType(const GuardType &type)
{
    switch (type)
    {
        case COVERAGE:
            return 1;
        case CONNECTIVITY:
            return 2;
        case INTERFACE:
            return 3;
        case QUALITY:
            return 4;
        case START:
        case GOAL:
        case CARTESIAN:
            OMPL_ERROR("Type: %u not handled yet in getVizVertexType()", type);
            return 5;
    }
    OMPL_ERROR("Unknown vertex type: %u", type);
    return 5;
}

void otb::SparseDB::addEdge(SparseVertex v1, SparseVertex v2, std::size_t visualColor, std::size_t coutIndent)
{
    assert(v1 <= getNumVertices());
    assert(v2 <= getNumVertices());
    assert(v1 != v2);

    if (verbose_)
        std::cout << std::string(coutIndent, ' ') + "addEdge: Connecting vertex " << v1 << " to vertex " << v2
                  << std::endl;

    // Create the new edge
    SparseEdge e = (boost::add_edge(v1, v2, g_)).first;

    // Add associated properties to the edge
    edgeWeightPropertySparse_[e] = distanceFunction(v1, v2);  // TODO: use this value with astar
    edgeCollisionStatePropertySparse_[e] = NOT_CHECKED;

    // Add the edge to the incrementeal connected components datastructure
    disjointSets_.union_set(v1, v2);

    // Visualize
    if (visualizeSparsCreation_)
    {
        /* Color Key:
           0 - connectivity
           50 - interface default
           100 - interface dave modification
        */
        visual_->viz2EdgeCallback(statePropertySparse_[v1], statePropertySparse_[v2], visualColor);
        visual_->viz2TriggerCallback();
        usleep(0.01 * 1000000);
    }
}

bool otb::SparseDB::checkAddCoverage(const base::State *qNew, std::vector<SparseVertex> &visibleNeighborhood,
                                     std::size_t coutIndent)
{
    if (verbose_)
        std::cout << std::string(coutIndent, ' ') + "checkAddCoverage() Are other nodes around it visible?"
                  << std::endl;

    // Only add a node for coverage if it has no neighbors
    if (visibleNeighborhood.size() > 0)
    {
        if (verbose_)
            std::cout << std::string(coutIndent + 2, ' ') + "NOT adding node for coverage " << std::endl;
        return false;  // has visible neighbors
    }

    // No free paths means we add for coverage
    if (verbose_)
        std::cout << std::string(coutIndent + 2, ' ') + "Adding node for COVERAGE " << std::endl;

    // SparseVertex v =
    addVertex(si_->cloneState(qNew), COVERAGE);
    // Note: we do not connect this node with any edges because we have already determined
    // it is too far away from any nearby nodes

    return true;
}

bool otb::SparseDB::checkAddConnectivity(const base::State *qNew, std::vector<SparseVertex> &visibleNeighborhood,
                                         std::size_t coutIndent)
{
    if (verbose_)
        std::cout << std::string(coutIndent, ' ') + "checkAddConnectivity() Does this node connect neighboring nodes "
                                                    "that are not connected? " << std::endl;

    // If less than 2 neighbors there is no way to find a pair of nodes in different connected components
    if (visibleNeighborhood.size() < 2)
    {
        if (verbose_)
            std::cout << std::string(coutIndent + 2, ' ') + "NOT adding node for connectivity" << std::endl;
        return false;
    }

    // Identify visibile nodes around our new state that are unconnected (in different connected components)
    // and connect them
    boost::unordered_set<SparseVertex> statesInDiffConnectedComponents;  // TODO(davetcoleman): in C++11 change to
                                                                         // std::unordered_set

    // For each neighbor
    for (std::size_t i = 0; i < visibleNeighborhood.size(); ++i)
    {
        // For each other neighbor
        for (std::size_t j = i + 1; j < visibleNeighborhood.size(); ++j)
        {
            // If they are in different components
            if (!sameComponent(visibleNeighborhood[i], visibleNeighborhood[j]))
            {
                statesInDiffConnectedComponents.insert(visibleNeighborhood[i]);
                statesInDiffConnectedComponents.insert(visibleNeighborhood[j]);
            }
        }
    }

    // Were any disconnected states found?
    if (statesInDiffConnectedComponents.empty())
    {
        if (verbose_)
            std::cout << std::string(coutIndent + 2, ' ') + "No states in diff connected components found "
                      << std::endl;
        return false;
    }

    if (verbose_)
        std::cout << std::string(coutIndent + 2, ' ') + "Adding node for CONNECTIVITY " << std::endl;

    // Add the node
    SparseVertex newVertex = addVertex(si_->cloneState(qNew), CONNECTIVITY);

    for (boost::unordered_set<SparseVertex>::const_iterator vertex_it = statesInDiffConnectedComponents.begin();
         vertex_it != statesInDiffConnectedComponents.end(); ++vertex_it)
    {
        if (verbose_)
            std::cout << std::string(coutIndent + 3, ' ') + "Loop: Adding vertex " << *vertex_it << std::endl;

        // Make sure vertices are not the same
        if (newVertex == *vertex_it)
        {
            OMPL_ERROR("Somehow the new vertex %u is same as the old vertex %u", newVertex, *vertex_it);

            exit(-1);
        }

        // New vertex should not be connected to anything - there's no edge between the two states
        if (boost::edge(newVertex, *vertex_it, g_).second == true)
        {
            OMPL_ERROR("Somehow the new vertex %u is already connected to old vertex %u", newVertex, *vertex_it);
            exit(-1);
        }

        // The components haven't been united by previous edges created in this for loop
        if (!sameComponent(*vertex_it, newVertex))
        {
            addEdge(newVertex, *vertex_it, 0, coutIndent + 4);
        }
        else
        {
            // This is not a big deal
            OMPL_WARN("Two states that where not prev in the same component were joined during the same for loop");
        }
    }

    return true;
}

bool otb::SparseDB::checkAddInterface(const base::State *qNew, std::vector<SparseVertex> &graphNeighborhood,
                                      std::vector<SparseVertex> &visibleNeighborhood, std::size_t coutIndent)
{
    if (verbose_)
        std::cout << std::string(coutIndent, ' ') + "checkAddInterface() Does this node's neighbor's need it to better "
                                                    "connect them?" << std::endl;

    // If we have at least 2 neighbors
    if (visibleNeighborhood.size() < 2)
    {
        return false;
    }
    // TODO(davetcoleman): why only the first two nodes??

    std::size_t visualColor = 50;

    // If the two closest nodes are also visible
    if (graphNeighborhood[0] == visibleNeighborhood[0] && graphNeighborhood[1] == visibleNeighborhood[1])
    {
        // If our two closest neighbors don't share an edge
        if (!boost::edge(visibleNeighborhood[0], visibleNeighborhood[1], g_).second)
        {
            // If they can be directly connected
            if (si_->checkMotion(statePropertySparse_[visibleNeighborhood[0]],
                                 statePropertySparse_[visibleNeighborhood[1]]))
            {
                if (verbose_)
                    std::cout << std::string(coutIndent + 2, ' ') + "INTERFACE: directly connected nodes" << std::endl;

                // Connect them
                addEdge(visibleNeighborhood[0], visibleNeighborhood[1], visualColor, coutIndent + 4);
            }
            else
            {
                // Add the new node to the graph, to bridge the interface
                if (verbose_)
                    std::cout << std::string(coutIndent + 2, ' ') + "Adding node for INTERFACE" << std::endl;

                SparseVertex v = addVertex(si_->cloneState(qNew), INTERFACE);
                addEdge(v, visibleNeighborhood[0], visualColor, coutIndent + 4);
                addEdge(v, visibleNeighborhood[1], visualColor, coutIndent + 4);
                if (verbose_)
                    std::cout << std::string(coutIndent + 2, ' ') + "INTERFACE: connected two neighbors through new "
                                                                    "interface node" << std::endl;
            }
            // Report success
            return true;
        }
        else
        {
            if (verbose_)
                std::cout << std::string(coutIndent + 2, ' ') + "Two neighbors already share an edge, not connecting"
                          << std::endl;
        }
    }
    return false;
}

/*bool otb::SparseDB::checkAsymptoticOptimal(DenseVertex denseVertex, std::size_t coutIndent)
{
    if (verbose_)
        std::cout << std::string(coutIndent, ' ') + "checkAsymptoticOptimal()" << std::endl;

    // Storage for the interface neighborhood, populated by getInterfaceNeighborhood()
    std::vector<DenseVertex> interfaceNeighborhood;

    // Check to see if Vertex is on an interface
    getInterfaceNeighborhood(qNew, interfaceNeighborhood);
    if (interfaceNeighborhood.size() > 0)
    {
        if (verbose_)
            std::cout << std::string(coutIndent+2, ' ') + "Interface neighborhood is not empty" << std::endl;

        //Check for addition for spanner prop
        //if (checkAddPath(q, interfaceNeighborhood))
        //return true;
    }
    //All of the tests have failed.  Report failure for the sample
    return false;
}

void otb::SparseDB::getInterfaceNeighborhood(DenseVertex q, std::vector<DenseVertex> &interfaceNeighborhood)
{
    // Get our representative
    BoltDB::SparseVertex rep = boltDB_->representativesProperty_[q];

    // // For each neighbor we are connected to
    // BOOST_FOREACH( DenseVertex n, boost::adjacent_vertices( q, boltDB_->g_ ) )
    // {
    //     // If neighbor representative is not our own
    //     if (representativesProperty_[n] != rep )
    //     {
    //         // If he is within denseDelta_
    //         if (distanceFunction( q, n ) < denseDelta_ )
    //         {
    //             // Append him to the list
    //             interfaceNeighborhood.push_back( n );
    //         }
    //     }
    // }

}
*/
/*
bool otb::SparseDB::checkAddPath( SparseVertex v )
{
    bool spannerPropertyWasViolated = false;

    std::vector< SparseVertex > rs;
    foreach( SparseVertex r, boost::adjacent_vertices( v, g_ ) )
        rs.push_back(r);

    // Candidate x vertices as described in the method, filled by function computeX().
    std::vector<SparseVertex> Xs;

    // Candidate v" vertices as described in the method, filled by function computeVPP().
    std::vector<SparseVertex> VPPs;

    for (std::size_t i = 0; i < rs.size() && !spannerPropertyWasViolated; ++i)
    {
        SparseVertex r = rs[i];
        computeVPP(v, r, VPPs);
        foreach (SparseVertex rp, VPPs)
        {
            //First, compute the longest path through the graph
            computeX(v, r, rp, Xs);
            double rm_dist = 0.0;
            foreach( SparseVertex rpp, Xs)
            {
                double tmp_dist = (si_->distance( statePropertySparse_[r], statePropertySparse_[v] )
                                   + si_->distance( statePropertySparse_[v], statePropertySparse_[rpp] ) )/2.0;
                if( tmp_dist > rm_dist )
                    rm_dist = tmp_dist;
            }

            InterfaceData& d = getData( v, r, rp );

            //Then, if the spanner property is violated
            if (rm_dist > stretchFactor_ * d.last_distance_)
            {
                spannerPropertyWasViolated = true; //Report that we added for the path
                if (si_->checkMotion(statePropertySparse_[r], statePropertySparse_[rp]))
                    addEdge(r, rp, coutIndent+4);
                else
                {
                    PathGeometric *p = new PathGeometric( si_ );
                    if (r < rp)
                    {
                        p->append(d.sigmaA_);
                        p->append(d.pointA_);
                        p->append(statePropertySparse_[v]);
                        p->append(d.pointB_);
                        p->append(d.sigmaB_);
                    }
                    else
                    {
                        p->append(d.sigmaB_);
                        p->append(d.pointB_);
                        p->append(statePropertySparse_[v]);
                        p->append(d.pointA_);
                        p->append(d.sigmaA_);
                    }

                    psimp_->reduceVertices(*p, 10);
                    psimp_->shortcutPath(*p, 50);

                    if (p->checkAndRepair(100).second)
                    {
                        SparseVertex prior = r;
                        SparseVertex vnew;
                        std::vector<base::State*>& states = p->getStates();

                        foreach (base::State *st, states)
                        {
                            // no need to clone st, since we will destroy p; we just copy the pointer
                            if (verbose_)
                                OMPL_INFORM(" --- Adding node for QUALITY");
                            vnew = addVertex(st , QUALITY);

                            addEdge(prior, vnew, coutIndent+4);
                            prior = vnew;
                        }
                        // clear the states, so memory is not freed twice
                        states.clear();
                        addEdge(prior, rp, coutIndent+4);
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
void otb::SparseDB::findGraphNeighbors(base::State *state, std::vector<SparseVertex> &graphNeighborhood,
                                       std::vector<SparseVertex> &visibleNeighborhood, std::size_t coutIndent)
{
    visibleNeighborhood.clear();

    // Search
    statePropertySparse_[queryVertex_] = state;
    nn_->nearestR(queryVertex_, sparseDelta_, graphNeighborhood);
    statePropertySparse_[queryVertex_] = NULL;

    // Now that we got the neighbors from the NN, we must remove any we can't see
    for (std::size_t i = 0; i < graphNeighborhood.size(); ++i)
    {
        if (si_->checkMotion(state, statePropertySparse_[graphNeighborhood[i]]))
        {
            visibleNeighborhood.push_back(graphNeighborhood[i]);
        }
    }

    if (verbose_)
        std::cout << std::string(coutIndent + 2, ' ') + "Graph neighborhood: " << graphNeighborhood.size()
                  << " | Visible neighborhood: " << visibleNeighborhood.size() << std::endl;
}
/*
otb::SparseVertex otb::SparseDB::findGraphRepresentative(base::State *st)
{
    std::vector<SparseVertex> nbh;
    statePropertySparse_[ queryVertex_ ] = st;
    nn_->nearestR( queryVertex_, sparseDelta_, nbh);
    statePropertySparse_[queryVertex_] = NULL;

    if (verbose_)
        OMPL_INFORM(" ------- findGraphRepresentative found %d nearest neighbors of distance %f",
                    nbh.size(), sparseDelta_);

    SparseVertex result = boost::graph_traits<Graph>::null_vertex();

    for (std::size_t i = 0 ; i< nbh.size() ; ++i)
    {
        if (verbose_)
            OMPL_INFORM(" -------- Checking motion of graph rep candidate %d", i);
        if (si_->checkMotion(st, statePropertySparse_[nbh[i]]))
        {
            if (verbose_)
                OMPL_INFORM(" --------- VALID ");
            result = nbh[i];
            break;
        }
    }
    return result;
}

void otb::SparseDB::findCloseRepresentatives(base::State *workState, const base::State *qNew, const SparseVertex qRep,
                                                        std::map<SparseVertex, base::State*> &closeRepresentatives,
                                                        const base::PlannerTerminationCondition &ptc)
{
    // Properly clear the vector by also deleting previously sampled unused states
    for (std::map<SparseVertex, base::State*>::iterator it = closeRepresentatives.begin(); it !=
closeRepresentatives.end();
++it)
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
                visual_->viz1StateCallback(workState, 3, sparseDelta_);
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

        } while ((!si_->isValid(workState) || si_->distance(qNew, workState) > denseDelta_ || !si_->checkMotion(qNew,
workState)) && ptc == false);

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
        SparseVertex representative = findGraphRepresentative(workState);

        // Assuming this sample is actually seen by somebody (which he should be in all likelihood)
        if (representative != boost::graph_traits<SparseGraph>::null_vertex())
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
            addVertex(si_->cloneState(workState), COVERAGE);

            if (verbose_)
            {
                OMPL_INFORM(" ------ STOP EFFORS TO ADD A DENSE PATH");
            }

            //We should also stop our efforts to add a dense path
            for (std::map<Vertex, base::State*>::iterator it = closeRepresentatives.begin(); it !=
closeRepresentatives.end(); ++it)
                si_->freeState(it->second);
            closeRepresentatives.clear();
            break;
        }
    } // for loop
}

*/

bool otb::SparseDB::sameComponent(SparseVertex m1, SparseVertex m2)
{
    return boost::same_component(m1, m2, disjointSets_);
}
