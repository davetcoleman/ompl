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
//#include <ompl/base/PlannerDataStorage.h>
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>  // TODO: remove, this is not space agnostic

// Boost
// #include <boost/filesystem.hpp>
// #include <boost/lambda/bind.hpp>
#include <boost/graph/incremental_components.hpp>
// #include <boost/property_map/vector_property_map.hpp>
#include <boost/foreach.hpp>
// #include <boost/thread.hpp>

// C++
#include <limits>
#include <queue>

// Allow hooks for visualizing planner
#define OMPL_BOLT_DEBUG

namespace og = ompl::geometric;
namespace ob = ompl::base;

// edgeWeightMap methods ////////////////////////////////////////////////////////////////////////////

BOOST_CONCEPT_ASSERT(
    (boost::ReadablePropertyMapConcept<ompl::geometric::SparseDB::edgeWeightMap, ompl::geometric::SparseDB::Edge>));

og::SparseDB::edgeWeightMap::edgeWeightMap(const Graph &graph, const EdgeCollisionStateMap &collisionStates,
                                         const double &popularityBias, const bool popularityBiasEnabled)
  : g_(graph)
  , collisionStates_(collisionStates)
  , popularityBias_(popularityBias)
  , popularityBiasEnabled_(popularityBiasEnabled)
{
}

double og::SparseDB::edgeWeightMap::get(Edge e) const
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
double get(const og::SparseDB::edgeWeightMap &m, const og::SparseDB::Edge &e)
{
    return m.get(e);
}
}

// CustomAstarVisitor methods ////////////////////////////////////////////////////////////////////////////

BOOST_CONCEPT_ASSERT((boost::AStarVisitorConcept<og::SparseDB::CustomAstarVisitor, og::SparseDB::Graph>));

og::SparseDB::CustomAstarVisitor::CustomAstarVisitor(Vertex goal, SparseDB *parent) : goal_(goal), parent_(parent)
{
}

void og::SparseDB::CustomAstarVisitor::discover_vertex(Vertex v, const Graph &) const
{
    if (parent_->visualizeAstar_)
        parent_->visual_->viz1StateCallback(parent_->statePropertySparse_[v], /*mode=*/ 1, 1);
}

void og::SparseDB::CustomAstarVisitor::examine_vertex(Vertex v, const Graph &) const
{
    if (parent_->visualizeAstar_)
    {
        parent_->visual_->viz1StateCallback(parent_->statePropertySparse_[v], /*mode=*/ 5, 1);
        parent_->visual_->viz1TriggerCallback();
        usleep(parent_->visualizeAstarSpeed_ * 1000000);
    }

    if (v == goal_)
        throw foundGoalException();
}

// Actual class ////////////////////////////////////////////////////////////////////////////

og::SparseDB::SparseDB(base::SpaceInformationPtr si, BoltDB* boltDB, tools::VisualizerPtr visual)
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
  , sparseDelta_(2.0)
  , visualizeAstarSpeed_(0.1)
{
    // Initialize nearest neighbor datastructure
    nn_.reset(new NearestNeighborsGNATNoThreadSafety<Vertex>());
    nn_->setDistanceFunction(boost::bind(&og::SparseDB::distanceFunction, this, _1, _2));

    // Add search state
    initializeQueryState();
}

og::SparseDB::~SparseDB(void)
{
    freeMemory();
}

void og::SparseDB::freeMemory()
{
    BOOST_FOREACH (Vertex v, boost::vertices(g_))
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

bool og::SparseDB::setup()
{
    return true;
}

bool og::SparseDB::astarSearch(const Vertex start, const Vertex goal, std::vector<Vertex> &vertexPath)
{
    // Hold a list of the shortest path parent to each vertex
    Vertex *vertexPredecessors = new Vertex[getNumVertices()];
    // boost::vector_property_map<Vertex> vertexPredecessors(getNumVertices());

    bool foundGoal = false;
    double *vertexDistances = new double[getNumVertices()];

    OMPL_INFORM("Beginning AStar Search");
    try
    {
        double popularityBias = 0;
        bool popularityBiasEnabled = false;
        // Note: could not get astar_search to compile within BoltRetrieveRepair.cpp class because of namespacing issues
        boost::astar_search(
            g_,     // graph
            start,  // start state
            // boost::bind(&og::SparseDB::distanceFunction2, this, _1, goal),  // the heuristic
            boost::bind(&og::SparseDB::distanceFunction, this, _1, goal),  // the heuristic
            // ability to disable edges (set cost to inifinity):
            boost::weight_map(edgeWeightMap(g_, edgeCollisionStatePropertySparse_, popularityBias, popularityBiasEnabled))
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

void og::SparseDB::debugVertex(const ompl::base::PlannerDataVertex &vertex)
{
    debugState(vertex.getState());
}

void og::SparseDB::debugState(const ompl::base::State *state)
{
    si_->printState(state, std::cout);
}

double og::SparseDB::distanceFunction(const Vertex a, const Vertex b) const
{
    // const double dist = si_->distance(statePropertySparse_[a], statePropertySparse_[b]);
    // std::cout << "getting distance from " << a << " to " << b << " of value " << dist << std::endl;
    // return dist;
    return si_->distance(statePropertySparse_[a], statePropertySparse_[b]);
}

void og::SparseDB::initializeQueryState()
{
    if (boost::num_vertices(g_) < 1)
    {
        queryVertex_ = boost::add_vertex(g_);
        statePropertySparse_[queryVertex_] = NULL;
    }
}

void og::SparseDB::clearEdgeCollisionStates()
{
    BOOST_FOREACH (const Edge e, boost::edges(g_))
        edgeCollisionStatePropertySparse_[e] = NOT_CHECKED;  // each edge has an unknown state
}

// Create SPARs graph using popularity
void og::SparseDB::createSPARS()
{
    bool verbose = true;

    // Sort the verticies by popularity in a queue
    std::priority_queue<BoltDB::WeightedVertex, std::vector<BoltDB::WeightedVertex>,
                        BoltDB::CompareWeightedVertex> pqueue;

    // Loop through each popular edge in the dense graph
    BOOST_FOREACH (BoltDB::Vertex v, boost::vertices(boltDB_->g_))
    {
        // Do not process the search vertex, it is null
        if (v == 0)
            continue;

        if (verbose)
            std::cout << "Vertex: " << v << std::endl;
        double popularity = 0;
        // std::pair<out_edge_iterator, out_edge_iterator> edge
        BOOST_FOREACH (BoltDB::Edge edge, boost::out_edges(v, boltDB_->g_))
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

    // Output the vertices in order
    while (!pqueue.empty())
    {
        BoltDB::Vertex v = pqueue.top().v_;
        //std::cout << "  Visualizing vertex " << v << " with popularity " << pqueue.top().weight_
                  //<< " queue remaining size " << pqueue.size() << std::endl;

        // Visualize
        const double visualWeight = pqueue.top().weight_ / largestWeight;
        visual_->viz1StateCallback(boltDB_->stateProperty_[v], /*mode=*/ 7, visualWeight);
        visual_->viz1TriggerCallback();
        usleep(0.01 * 1000000);

        // Attempt to insert into SPARS graph
        double seconds = 1000;
        ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(seconds, 0.1);
        if (!addStateToRoadmap(ptc, boltDB_->stateProperty_[v]))
            std::cout << "Failed to add state to roadmap" << std::endl;

        // Remove from priority queue
        pqueue.pop();
    }  // end while
}

bool og::SparseDB::addStateToRoadmap(const base::PlannerTerminationCondition &ptc, base::State *newState)
{
    if (verbose_)
        std::cout << "addStateToRoadmap()" << std::endl;

    bool stateAdded = false;
    std::size_t coutIndent = 0;

    // Deep copy
    base::State *qNew = si_->cloneState(newState);  // TODO(davetcoleman): do i need to clone it?
    base::State *workState = si_->allocState();     // TODO(davetcoleman): do i need this state?

    /* Nodes near our newState */
    std::vector<Vertex> graphNeighborhood;
    /* Visible nodes near our newState */
    std::vector<Vertex> visibleNeighborhood;

    // Find nearby nodes
    findGraphNeighbors(qNew, graphNeighborhood, visibleNeighborhood, coutIndent);

    // Always add a node if no other nodes around it are visible (GUARD)
    if (checkAddCoverage(qNew, visibleNeighborhood))
    {
        stateAdded = true;
    }
    else if (checkAddConnectivity(qNew, visibleNeighborhood))  // Connectivity criterion
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

            for (std::map<Vertex, base::State*>::iterator it = closeRepresentatives.begin(); it !=
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

            for (std::map<Vertex, base::State*>::iterator it = closeRepresentatives.begin(); it !=
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

    si_->freeState(workState);
    si_->freeState(qNew);

    return stateAdded;
}

og::SparseDB::Vertex og::SparseDB::addVertex(base::State *state, const GuardType &type)
{
    // Create vertex
    Vertex v = boost::add_vertex(g_);

    // Add properties
    typePropertySparse_[v] = type;
    statePropertySparse_[v] = state;

    // Connected component tracking
    disjointSets_.make_set(v);

    // Add vertex to nearest neighbor structure
    nn_->add(v);

    // Visualize
    visual_->viz2StateCallback(statePropertySparse_[v], /*mode=*/ 4, sparseDelta_);
    visual_->viz2TriggerCallback();
    usleep(0.01 * 1000000);

    return v;
}

void og::SparseDB::addEdge(Vertex v1, Vertex v2, std::size_t coutIndent)
{
    assert(v1 <= getNumVertices());
    assert(v2 <= getNumVertices());
    assert(v1 != v2);

    if (verbose_)
        std::cout << std::string(coutIndent+4, ' ') + "addEdge: Connecting vertex " << v1 << " to vertex " << v2 << std::endl;

    // Create the new edge
    Edge e = (boost::add_edge(v1, v2, g_)).first;

    // Add associated properties to the edge
    edgeWeightPropertySparse_[e] = distanceFunction(v1, v2); // TODO: use this value with astar
    edgeCollisionStatePropertySparse_[e] = NOT_CHECKED;

    // Add the edge to the incrementeal connected components datastructure
    disjointSets_.union_set(v1, v2);

    // Visualize
    visual_->viz2EdgeCallback(statePropertySparse_[v1], statePropertySparse_[v2], 0);
    visual_->viz2TriggerCallback();
    usleep(0.01 * 1000000);
}

bool og::SparseDB::checkAddCoverage(const base::State *qNew, std::vector<Vertex> &visibleNeighborhood)
{
    std::size_t coutIndent = 4;
    if (verbose_)
        std::cout << std::string(coutIndent, ' ') + "checkAddCoverage() Are other nodes around it visible?" << std::endl;

    // Only add a node for coverage if it has no neighbors
    if (visibleNeighborhood.size() > 0)
    {
        if (verbose_)
            std::cout << std::string(coutIndent+2, ' ') + "NOT adding node for coverage " << std::endl;
        return false;  // has visible neighbors
    }

    // No free paths means we add for coverage
    if (verbose_)
        std::cout << std::string(coutIndent+2, ' ') + "Adding node for COVERAGE " << std::endl;

    //Vertex v =
    addVertex(si_->cloneState(qNew), COVERAGE);
    // Note: we do not connect this node with any edges because we have already determined
    // it is too far away from any nearby nodes

    return true;
}

bool og::SparseDB::checkAddConnectivity(const base::State *qNew, std::vector<Vertex> &visibleNeighborhood)
{
    std::size_t coutIndent = 8;
    if (verbose_)
        std::cout << std::string(coutIndent, ' ') + "checkAddConnectivity() Does this node connect neighboring nodes "
                                                      "that are not connected? " << std::endl;

    // If less than 2 neighbors there is no way to find a pair of nodes in different connected components
    if (visibleNeighborhood.size() < 2)
    {
        if (verbose_)
            std::cout << std::string(coutIndent+2, ' ') + "NOT adding node for connectivity" << std::endl;
        return false;
    }

    // Identify visibile nodes around our new state that are unconnected (in different connected components)
    // and connect them
    std::set<Vertex> statesInDiffConnectedComponents; // TODO(davetcoleman): in C++11 change to unordered_set for faster performance

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
            std::cout << std::string(coutIndent+2, ' ') + "No states in diff connected components found " << std::endl;
        return false;
    }

    if (verbose_)
        std::cout << std::string(coutIndent+2, ' ') + "Adding node for CONNECTIVITY " << std::endl;

    // Add the node
    Vertex newVertex = addVertex(si_->cloneState(qNew), CONNECTIVITY);

    for (std::set<Vertex>::const_iterator vertex_it = statesInDiffConnectedComponents.begin();
         vertex_it != statesInDiffConnectedComponents.end(); ++vertex_it)
    {
        if (verbose_)
            std::cout << std::string(coutIndent+3, ' ') + "Loop: Adding vertex " << *vertex_it << std::endl;

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
            addEdge(newVertex, *vertex_it, coutIndent);
        }
        else
        {
            // This is not a big deal
            OMPL_WARN("Two states that where not prev in the same component were joined during the same for loop");
        }
    }

    return true;
}
/*
bool og::SparseDB::checkAddInterface(const base::State *qNew, std::vector<Vertex> &graphNeighborhood, std::vector<Vertex>
&visibleNeighborhood)
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
                if (si_->checkMotion(statePropertySparse_[visibleNeighborhood[0]], statePropertySparse_[visibleNeighborhood[1]]))
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
                    Vertex v = addVertex(si_->cloneState(qNew), INTERFACE);
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

bool og::SparseDB::checkAddPath( Vertex v )
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
                    connectGuards(r, rp);
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
                        Vertex prior = r;
                        Vertex vnew;
                        std::vector<base::State*>& states = p->getStates();

                        foreach (base::State *st, states)
                        {
                            // no need to clone st, since we will destroy p; we just copy the pointer
                            if (verbose_)
                                OMPL_INFORM(" --- Adding node for QUALITY");
                            vnew = addVertex(st , QUALITY);

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
void og::SparseDB::findGraphNeighbors(base::State *state, std::vector<Vertex> &graphNeighborhood,
    std::vector<Vertex> &visibleNeighborhood, std::size_t coutIndent)
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
        std::cout << std::string(coutIndent+2, ' ') + "Graph neighborhood: " << graphNeighborhood.size()
                  << " | Visible neighborhood: " << visibleNeighborhood.size() << std::endl;
}
/*
og::SparseDB::Vertex og::SparseDB::findGraphRepresentative(base::State *st)
{
    std::vector<Vertex> nbh;
    statePropertySparse_[ queryVertex_ ] = st;
    nn_->nearestR( queryVertex_, sparseDelta_, nbh);
    statePropertySparse_[queryVertex_] = NULL;

    if (verbose_)
        OMPL_INFORM(" ------- findGraphRepresentative found %d nearest neighbors of distance %f",
                    nbh.size(), sparseDelta_);

    Vertex result = boost::graph_traits<Graph>::null_vertex();

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

void og::SparseDB::findCloseRepresentatives(base::State *workState, const base::State *qNew, const Vertex qRep,
                                                        std::map<Vertex, base::State*> &closeRepresentatives,
                                                        const base::PlannerTerminationCondition &ptc)
{
    // Properly clear the vector by also deleting previously sampled unused states
    for (std::map<Vertex, base::State*>::iterator it = closeRepresentatives.begin(); it != closeRepresentatives.end();
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

bool og::SparseDB::sameComponent(Vertex m1, Vertex m2)
{
    return boost::same_component(m1, m2, disjointSets_);
}
