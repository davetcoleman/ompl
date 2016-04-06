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

#define foreach BOOST_FOREACH

namespace og = ompl::geometric;
namespace ot = ompl::tools;
namespace otb = ompl::tools::bolt;
namespace ob = ompl::base;

// edgeWeightMap methods ////////////////////////////////////////////////////////////////////////////

BOOST_CONCEPT_ASSERT((boost::ReadablePropertyMapConcept<otb::SparseDB::edgeWeightMap, otb::SparseEdge>));

otb::SparseDB::edgeWeightMap::edgeWeightMap(const SparseGraph &graph,
                                            const SparseEdgeCollisionStateMap &collisionStates,
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
        parent_->visual_->viz4StateCallback(parent_->getSparseState(v), /*mode=*/1, 1);
}

void otb::SparseDB::CustomAstarVisitor::examine_vertex(SparseVertex v, const SparseGraph &) const
{
    if (parent_->visualizeAstar_)
    {
        parent_->visual_->viz4StateCallback(parent_->getSparseState(v), /*mode=*/5, 1);
        parent_->visual_->viz4TriggerCallback();
        usleep(parent_->visualizeAstarSpeed_ * 1000000);
    }

    if (v == goal_)
        throw foundGoalException();
}

// Actual class ////////////////////////////////////////////////////////////////////////////

namespace ompl
{
namespace tools
{
namespace bolt
{
SparseDB::SparseDB(base::SpaceInformationPtr si, BoltDB *boltDB, VisualizerPtr visual)
  : si_(si)
  , boltDB_(boltDB)
  , visual_(visual)
  , smoothingGeomPath_(si)
  // Property accessors of edges
  , edgeWeightPropertySparse_(boost::get(boost::edge_weight, g_))
  , edgeCollisionStatePropertySparse_(boost::get(edge_collision_state_t(), g_))
  // Property accessors of vertices
  , denseVertexProperty_(boost::get(vertex_state2_t(), g_))
  , typePropertySparse_(boost::get(vertex_type_t(), g_))
  , nonInterfaceListsProperty_(boost::get(vertex_list_t(), g_))
  , interfaceListsProperty_(boost::get(vertex_interface_list_t(), g_))
  // Disjoint set accessors
  , disjointSets_(boost::get(boost::vertex_rank, g_), boost::get(boost::vertex_predecessor, g_))
  // Remember what round we're on
  , secondSparseInsertionAttempt_(false)
  // Derived properties
  , sparseDelta_(2.0)
  // Sparse properites
  , denseDeltaFraction_(.05)
  , sparseDeltaFraction_(.25)
  //, denseDeltaFraction_(.001)
  , stretchFactor_(3.)
  // Visualization settings
    , checksVerbose_(false)
  , fourthCheckVerbose_(true)
  , visualizeAstar_(false)
  , visualizeSparsCreation_(false)
  , visualizeDenseRepresentatives_(false)
  , visualizeAstarSpeed_(0.1)
  , sparseCreationInsertionOrder_(0)
{
    // Add search state
    initializeQueryState();

    // Initialize nearest neighbor datastructure
    nn_.reset(new NearestNeighborsGNATNoThreadSafety<SparseVertex>());
    nn_->setDistanceFunction(boost::bind(&otb::SparseDB::distanceFunction, this, _1, _2));

    // Initialize path simplifier
    psimp_.reset(new geometric::PathSimplifier(si_));
    psimp_->freeStates(false);
}

SparseDB::~SparseDB(void)
{
    freeMemory();
}

void SparseDB::freeMemory()
{
    foreach (SparseVertex v, boost::vertices(g_))
    {
        // foreach (InterfaceData &d, interfaceDataProperty_[v].interfaceHash |
        // boost::adaptors::map_values)
        // d.clear(si_);
        if (getSparseState(v) != NULL)
            si_->freeState(getSparseState(v));
        // getSparseState(v) = NULL;  // TODO(davetcoleman): is this needed??
    }
    g_.clear();

    if (nn_)
        nn_->clear();
}

bool SparseDB::setup()
{
    // Calculate variables for the graph
    const double maxExt = si_->getMaximumExtent();
    sparseDelta_ = sparseDeltaFraction_ * maxExt;
    denseDelta_ = denseDeltaFraction_ * maxExt;
    OMPL_INFORM("sparseDelta_ = %f", sparseDelta_);
    OMPL_INFORM("denseDelta_ = %f", denseDelta_);

    assert(maxExt > 0);
    assert(denseDelta_ > 0);

    return true;
}

bool SparseDB::astarSearch(const SparseVertex start, const SparseVertex goal, std::vector<SparseVertex> &vertexPath)
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
        // Note: could not get astar_search to compile within BoltRetrieveRepair.cpp class because of
        // namespacing issues
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
        // if ((diff < std::numeric_limits<double>::epsilon()) && (-diff <
        // std::numeric_limits<double>::epsilon()))
        // check if the distance to goal is inifinity. if so, it is unreachable
        // if (d[goal] >= std::numeric_limits<double>::infinity())
        {
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
                visual_->viz4EdgeCallback(getSparseStateConst(v1), getSparseStateConst(v2), 10);
            }
        }
        visual_->viz4TriggerCallback();
    }

    // Unload
    delete[] vertexPredecessors;
    delete[] vertexDistances;

    // No solution found from start to goal
    return foundGoal;
}

void SparseDB::debugVertex(const ompl::base::PlannerDataVertex &vertex)
{
    debugState(vertex.getState());
}

void SparseDB::debugState(const ompl::base::State *state)
{
    si_->printState(state, std::cout);
}

double SparseDB::distanceFunction(const SparseVertex a, const SparseVertex b) const
{
    // const double dist = si_->distance(getSparseState(a), getSparseState(b));
    // std::cout << "getting distance from " << a << " to " << b << " of value " << dist << std::endl;
    // return dist;
    return si_->distance(getSparseStateConst(a), getSparseStateConst(b));
}

void SparseDB::initializeQueryState()
{
    if (boost::num_vertices(g_) < 1)
    {
        queryVertex_ = boost::add_vertex(g_);
        denseVertexProperty_[queryVertex_] = boltDB_->queryVertex_;
        getSparseState(queryVertex_) = NULL;
    }
}

void SparseDB::clearEdgeCollisionStates()
{
    foreach (const SparseEdge e, boost::edges(g_))
        edgeCollisionStatePropertySparse_[e] = NOT_CHECKED;  // each edge has an unknown state
}

void SparseDB::createSPARS()
{
    // Clear the old spars graph
    if (getNumVertices() > 1)
    {
        OMPL_INFORM("Cleaning sparse database, currently has %u states", getNumVertices());
        g_.clear();
        OMPL_INFORM("Now has %u states", getNumVertices());

        // Clear the nearest neighbor search
        if (nn_)
            nn_->clear();

        // Re-add search state
        initializeQueryState();

        // Clear visuals
        visual_->viz2StateCallback(getSparseStateConst(queryVertex_), /* type = deleteAllMarkers */ 0, 0);
    }

    // Reset fractions
    setup();

    // Get the ordering to insert vertices
    std::vector<WeightedVertex> vertexInsertionOrder;

    if (sparseCreationInsertionOrder_ == 0)
        getPopularityOrder(vertexInsertionOrder);  // Create SPARs graph in order of popularity
    else if (sparseCreationInsertionOrder_ == 1)
        getDefaultOrder(vertexInsertionOrder);
    else if (sparseCreationInsertionOrder_ == 2)
        getRandomOrder(vertexInsertionOrder);
    else
    {
        OMPL_ERROR("Unknown insertion order method");
        exit(-1);
    }

    // Limit amount of time generating TODO(davetcoleman): remove this feature
    double seconds = 1000;
    ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(seconds, 0.1);

    // Attempt to insert the verticies multiple times until no more succesful insertions occur
    bool succeededInInserting = true;
    secondSparseInsertionAttempt_ = false;
    std::size_t loopAttempt = 0;
    while (succeededInInserting)
    {
        std::cout << "-------------------------------------------------------" << std::endl;
        std::cout << "Attempting to insert " << vertexInsertionOrder.size() << " vertices for the " << loopAttempt
                  << " loop" << std::endl;

        // Sanity check
        if (loopAttempt > 3)
            OMPL_WARN("Suprising number of loop when attempting to insert nodes into SPARS graph: %u", loopAttempt);

        // Attempt to insert each vertex using the first 3 criteria
        succeededInInserting = false;
        std::size_t sucessfulInsertions = 0;
        for (std::size_t i = 0; i < vertexInsertionOrder.size(); ++i)
        {
            // Attempt to insert into SPARS graph // TODO(davetcoleman): remove this timer
            double seconds = 1000;
            ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(seconds, 0.1);

            // Customize the sparse delta fraction
            // if (!secondSparseInsertionAttempt_)
            // {
            //     std::size_t minDelta = 2;
            //     std::size_t maxDelta = 6;
            //     double invertedPopularity = 100 - vertexInsertionOrder[i].weight_;
            //     sparseDelta_ = invertedPopularity * (maxDelta - minDelta) / 100.0 + minDelta;
            //     OMPL_INFORM("sparseDelta_ is now %f", sparseDelta_);
            // }

            // Run SPARS checks
            if (!addStateToRoadmap(ptc, vertexInsertionOrder[i].v_))
            {
                // std::cout << "Failed AGAIN to add state to roadmap------" << std::endl;
            }
            else
            {
                vertexInsertionOrder.erase(vertexInsertionOrder.begin() + i);
                i--;
                sucessfulInsertions++;
                succeededInInserting = true;
            }
        }
        std::cout << "Succeeded in inserting " << sucessfulInsertions << " vertices on the " << loopAttempt << " loop"
                  << std::endl;
        loopAttempt++;

        // Increase the sparse delta a bit, but only after the first loop
        if (loopAttempt == 1)
        {
            sparseDelta_ = sparseDelta_ * 1.25;
            OMPL_INFORM("sparseDelta_ is now %f", sparseDelta_);
            secondSparseInsertionAttempt_ = true;
        }

        bool debugOverRideJustTwice = false;
        if (debugOverRideJustTwice && loopAttempt == 2)
        {
            OMPL_WARN("Only attempting to add nodes twice for speed");
            break;
        }
    }

    /*
    // Determine every dense node's representative on the sparse graph
    findSparseRepresentatives();

    // Check 4th criteria - are we within our stated asymptotic bounds?
    std::size_t coutIndent = 0;
    OMPL_INFORM("Checking remaining vertices for 4th critera test");
    for (std::size_t i = 0; i < vertexInsertionOrder.size(); ++i)
    {
        if (!checkAsymptoticOptimal(vertexInsertionOrder[i].v_, coutIndent + 4))
        {
            std::cout << "Vertex " << vertexInsertionOrder[i].v_ << " failed asymptotic optimal test " << std::endl;
        }
    }
    */

    // If we haven't animated the creation, just show it all at once
    if (!visualizeSparsCreation_)
    {
        displayDatabase();
    }

    // Statistics
    std::cout << std::endl;
    OMPL_INFORM("Created SPARS graph:");
    OMPL_INFORM("  Vertices:  %u", getNumVertices());
    OMPL_INFORM("  Edges:  %u", getNumEdges());

    std::size_t numSets = getDisjointSets();
    if (numSets > 1)
    {
        OMPL_ERROR("  Disjoint sets: %u", numSets);
        getDisjointSets(true); // show in verbose mode
        exit(-1);
    }
    else
        OMPL_INFORM("  Disjoint sets: %u", numSets);
}

std::size_t SparseDB::getDisjointSets(bool verbose)
{
    std::size_t numSets = 0;
    foreach (SparseVertex v, boost::vertices(g_))
    {
        // Do not count the search vertex within the sets
        if (v == queryVertex_)
            continue;

        if (boost::get(boost::get(boost::vertex_predecessor, g_), v) == v)
        {
            if (verbose)
                OMPL_INFORM("Disjoint set: %u", v);
            ++numSets;
        }
    }
    return numSets;
}

bool SparseDB::findSparseRepresentatives()
{
    bool verbose = false;

    OMPL_INFORM("Calculating representative nodes for each dense vertex");
    foreach (DenseVertex denseV, boost::vertices(boltDB_->g_))
    {
        std::vector<SparseVertex> graphNeighborhood;
        base::State *state = getDenseState(denseV);

        // Skip the query vertex 0
        if (denseV == boltDB_->queryVertex_)
            continue;
        assert(denseV);

        if (verbose)
            std::cout << "Searching for denseV: " << denseV << std::endl;

        // Search
        getSparseState(queryVertex_) = state;
        nn_->nearestR(queryVertex_, sparseDelta_, graphNeighborhood);
        getSparseState(queryVertex_) = NULL;

        // Find the closest sparse node that has a local free path
        bool foundRepresentative = false;
        for (std::size_t i = 0; i < graphNeighborhood.size(); ++i)
        {
            if (verbose)
                std::cout << "  Checking motion for neighbor " << i << std::endl;
            if (si_->checkMotion(state, getSparseState(graphNeighborhood[i])))
            {
                if (verbose)
                    std::cout << "   Found valid sparse vertex that is near: " << graphNeighborhood[i] << std::endl;

                // Assign the dense vertex its representative sparse node
                boltDB_->representativesProperty_[denseV] = graphNeighborhood[i];
                foundRepresentative = true;
                break;
            }
        }

        // Error check
        if (!foundRepresentative)
        {
            OMPL_WARN("Unable to find sparse representative for dense vertex %u", denseV);
            exit(-1);
        }

        // Visualize
        if (visualizeDenseRepresentatives_)
        {
            // Ensure that states are not the same
            SparseVertex &sparseV = boltDB_->representativesProperty_[denseV];
            if (denseV == denseVertexProperty_[sparseV])
            {
                if (verbose)
                    OMPL_WARN("Not visualizing because the dense vertex's representative sparse vertex are the same");
            }
            else
            {
                double visualColor = 100;
                visual_->viz2EdgeCallback(state, getSparseState(boltDB_->representativesProperty_[denseV]),
                                          visualColor);
            }
        }
    }
    visual_->viz2TriggerCallback();

    return true;
}

bool SparseDB::getPopularityOrder(std::vector<WeightedVertex> &vertexInsertionOrder)
{
    bool verbose = false;

    // Error check
    if (!boltDB_->getNumVertices())
    {
        OMPL_ERROR("Unable to get vertices in order of popularity becayse dense graph is empty");
        exit(-1);
    }

    // Sort the verticies by popularity in a queue
    std::priority_queue<WeightedVertex, std::vector<WeightedVertex>, CompareWeightedVertex>
        pqueue;

    // Loop through each popular edge in the dense graph
    foreach (DenseVertex v, boost::vertices(boltDB_->g_))
    {
        // Do not process the search vertex, it is null
        if (v == 0)  // TODO do not assume the search vertex is the first one
            continue;

        if (verbose)
            std::cout << "Vertex: " << v << std::endl;
        double popularity = 0;
        // std::pair<out_edge_iterator, out_edge_iterator> edge
        foreach (DenseEdge edge, boost::out_edges(v, boltDB_->g_))
        {
            if (verbose)
                std::cout << "  Edge: " << edge << std::endl;
            popularity += (100 - boltDB_->edgeWeightProperty_[edge]);
        }
        if (verbose)
            std::cout << "  Total popularity: " << popularity << std::endl;
        pqueue.push(WeightedVertex(v, popularity));
    }

    // Remember which one was the largest
    double largestWeight = pqueue.top().weight_;

    // Convert pqueue into vector
    while (!pqueue.empty())  // Output the vertices in order
    {
        vertexInsertionOrder.push_back(pqueue.top());

        // Modify the weight to be a percentage of the max weight
        const double weightPercent = pqueue.top().weight_ / largestWeight * 100.0;
        vertexInsertionOrder.back().weight_ = weightPercent;

        // Visualize
        if (visualizeSparsCreation_)
        {
            // std::cout << "  Visualizing vertex " << v << " with popularity " << weightPercent
            //<< " queue remaining size " << pqueue.size() << std::endl;
            visual_->viz3StateCallback(boltDB_->stateProperty_[pqueue.top().v_], /*mode=*/7, weightPercent);
        }

        // Remove from priority queue
        pqueue.pop();
    }
    visual_->viz3TriggerCallback();
    usleep(0.01 * 1000000);

    return true;
}

bool SparseDB::getDefaultOrder(std::vector<WeightedVertex> &vertexInsertionOrder)
{
    bool verbose = false;
    double largestWeight = -1 * std::numeric_limits<double>::infinity();
    std::vector<double> weights;

    // Loop through each popular edge in the dense graph
    foreach (DenseVertex v, boost::vertices(boltDB_->g_))
    {
        // Do not process the search vertex, it is null
        if (v == 0)  // TODO do not assume the search vertex is the first one
            continue;

        if (verbose)
            std::cout << "Vertex: " << v << std::endl;
        double popularity = 0;
        // std::pair<out_edge_iterator, out_edge_iterator> edge
        foreach (DenseEdge edge, boost::out_edges(v, boltDB_->g_))
        {
            if (verbose)
                std::cout << "  Edge: " << edge << std::endl;
            popularity += (100 - boltDB_->edgeWeightProperty_[edge]);
        }
        if (verbose)
            std::cout << "  Total popularity: " << popularity << std::endl;

        // Record
        vertexInsertionOrder.push_back(WeightedVertex(v, popularity));

        // Track largest weight
        if (popularity > largestWeight)
            largestWeight = popularity;
    }

    // Update the weights
    foreach (WeightedVertex wv, vertexInsertionOrder)
    {
        // Modify the weight to be a percentage of the max weight
        const double weightPercent = wv.weight_ / largestWeight * 100.0;
        wv.weight_ = weightPercent;

        // Visualize vertices
        if (visualizeSparsCreation_)
        {
            visual_->viz3StateCallback(boltDB_->stateProperty_[wv.v_], /*mode=*/7, weightPercent);
        }
    }

    visual_->viz3TriggerCallback();
    usleep(0.01 * 1000000);

    return true;
}

bool SparseDB::getRandomOrder(std::vector<WeightedVertex> &vertexInsertionOrder)
{
    std::vector<WeightedVertex> defaultVertexInsertionOrder;
    getDefaultOrder(defaultVertexInsertionOrder);

    for (std::size_t i = 0; i < defaultVertexInsertionOrder.size(); ++i)
    {
        // Choose a random vertex to pick out of default structure
        std::size_t randVertex = static_cast<std::size_t>(iRand(0, defaultVertexInsertionOrder.size() - 1));
        std::cout << "Choose: " << randVertex << std::endl;
        assert(randVertex < defaultVertexInsertionOrder.size());

        // Copy random vertex to new structure
        vertexInsertionOrder.push_back(defaultVertexInsertionOrder[randVertex]);

        // Delete that vertex
        defaultVertexInsertionOrder.erase(defaultVertexInsertionOrder.begin() + randVertex);
        randVertex--;
    }
    return true;
}

int SparseDB::iRand(int min, int max)
{
  int n = max - min + 1;
  int remainder = RAND_MAX % n;
  int x;
  do
  {
    x = rand();
  } while (x >= RAND_MAX - remainder);
  return min + x % n;
}

bool SparseDB::addStateToRoadmap(const base::PlannerTerminationCondition &ptc, DenseVertex denseV)
{
    std::size_t coutIndent = 2;
    if (checksVerbose_)
        std::cout << std::string(coutIndent, ' ') + "addStateToRoadmap() DenseV: " << denseV << std::endl;

    bool stateAdded = false;

    // Deep copy
    // base::State *workState = si_->allocState();     // TODO(davetcoleman): do i need this state?

    /* Nodes near our input state */
    std::vector<SparseVertex> graphNeighborhood;
    /* Visible nodes near our input state */
    std::vector<SparseVertex> visibleNeighborhood;

    // Find nearby nodes
    findGraphNeighbors(denseV, graphNeighborhood, visibleNeighborhood, coutIndent + 4);

    // Always add a node if no other nodes around it are visible (GUARD)
    if (checkAddCoverage(denseV, visibleNeighborhood, coutIndent + 4))
    {
        stateAdded = true;
    }
    else if (checkAddConnectivity(denseV, visibleNeighborhood, coutIndent + 8))  // Connectivity criterion
    {
        stateAdded = true;
    }
    else if (checkAddInterface(denseV, graphNeighborhood, visibleNeighborhood, coutIndent + 12))
    {
        stateAdded = true;
    }

    return stateAdded;
}

bool SparseDB::checkAddCoverage(const DenseVertex &denseV, std::vector<SparseVertex> &visibleNeighborhood,
                                std::size_t coutIndent)
{
    if (checksVerbose_)
        std::cout << std::string(coutIndent, ' ') + "checkAddCoverage() Are other nodes around it visible?"
                  << std::endl;

    // Only add a node for coverage if it has no neighbors
    if (visibleNeighborhood.size() > 0)
    {
        if (checksVerbose_)
            std::cout << std::string(coutIndent + 2, ' ') + "NOT adding node for coverage " << std::endl;
        return false;  // has visible neighbors
    }

    // No free paths means we add for coverage
    if (checksVerbose_)
        std::cout << std::string(coutIndent + 2, ' ') + "Adding node for COVERAGE " << std::endl;

    // SparseVertex v =
    addVertex(denseV, COVERAGE);
    // Note: we do not connect this node with any edges because we have already determined
    // it is too far away from any nearby nodes

    return true;
}

bool SparseDB::checkAddConnectivity(const DenseVertex &denseV, std::vector<SparseVertex> &visibleNeighborhood,
                                    std::size_t coutIndent)
{
    if (checksVerbose_)
        std::cout << std::string(coutIndent, ' ') + "checkAddConnectivity() Does this node connect neighboring nodes "
                                                    "that are not connected? " << std::endl;

    // If less than 2 neighbors there is no way to find a pair of nodes in different connected components
    if (visibleNeighborhood.size() < 2)
    {
        if (checksVerbose_)
            std::cout << std::string(coutIndent + 2, ' ') + "NOT adding node for connectivity" << std::endl;
        return false;
    }

    // Identify visibile nodes around our new state that are unconnected (in different connected components)
    // and connect them
    boost::unordered_set<SparseVertex> statesInDiffConnectedComponents;  // TODO(davetcoleman): in C++11 change
                                                                         // to
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
        if (checksVerbose_)
            std::cout << std::string(coutIndent + 2, ' ') + "No states in diff connected components found "
                      << std::endl;
        return false;
    }

    if (checksVerbose_)
        std::cout << std::string(coutIndent + 2, ' ') + "Adding node for CONNECTIVITY " << std::endl;

    // Add the node
    SparseVertex newVertex = addVertex(denseV, CONNECTIVITY);

    for (boost::unordered_set<SparseVertex>::const_iterator vertex_it = statesInDiffConnectedComponents.begin();
         vertex_it != statesInDiffConnectedComponents.end(); ++vertex_it)
    {
        if (checksVerbose_)
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
            std::size_t visualColor = 0;
            if (secondSparseInsertionAttempt_)
                visualColor = 75;
            addEdge(newVertex, *vertex_it, visualColor, coutIndent + 4);
        }
        else
        {
            // This is not a big deal
            // OMPL_WARN("Two states that where not prev in the same component were joined during the same for "
            //"loop");
        }
    }

    return true;
}

bool SparseDB::checkAddInterface(const DenseVertex &denseV, std::vector<SparseVertex> &graphNeighborhood,
                                 std::vector<SparseVertex> &visibleNeighborhood, std::size_t coutIndent)
{
    if (checksVerbose_)
        std::cout << std::string(coutIndent, ' ') + "checkAddInterface() Does this node's neighbor's need it to better "
                                                    "connect them?" << std::endl;

    // If we have at least 2 neighbors
    if (visibleNeighborhood.size() < 2)
    {
        return false;
    }
    // TODO(davetcoleman): why only the first two nodes??
    std::size_t visualColor = 50;
    if (secondSparseInsertionAttempt_)
        visualColor = 100;

    // If the two closest nodes are also visible
    if (graphNeighborhood[0] == visibleNeighborhood[0] && graphNeighborhood[1] == visibleNeighborhood[1])
    {
        // If our two closest neighbors don't share an edge
        if (!boost::edge(visibleNeighborhood[0], visibleNeighborhood[1], g_).second)
        {
            // If they can be directly connected
            if (si_->checkMotion(getSparseState(visibleNeighborhood[0]), getSparseState(visibleNeighborhood[1])))
            {
                if (checksVerbose_)
                    std::cout << std::string(coutIndent + 2, ' ') + "INTERFACE: directly connected nodes" << std::endl;

                // Connect them
                addEdge(visibleNeighborhood[0], visibleNeighborhood[1], visualColor, coutIndent + 4);
            }
            else
            {
                // Add the new node to the graph, to bridge the interface
                if (checksVerbose_)
                    std::cout << std::string(coutIndent + 2, ' ') + "Adding node for INTERFACE" << std::endl;

                SparseVertex v = addVertex(denseV, INTERFACE);
                addEdge(v, visibleNeighborhood[0], visualColor, coutIndent + 4);
                addEdge(v, visibleNeighborhood[1], visualColor, coutIndent + 4);
                if (checksVerbose_)
                    std::cout << std::string(coutIndent + 2, ' ') + "INTERFACE: connected two neighbors through new "
                                                                    "interface node" << std::endl;
            }
            // Report success
            return true;
        }
        else
        {
            if (checksVerbose_)
                std::cout << std::string(coutIndent + 2, ' ') + "Two neighbors already share an edge, not "
                                                                "connecting" << std::endl;
        }
    }

    return false;
}

bool SparseDB::checkAsymptoticOptimal(const DenseVertex &denseV, std::size_t coutIndent)
{
    if (fourthCheckVerbose_)
        std::cout << std::string(coutIndent, ' ') + "checkAsymptoticOptimal()" << std::endl;

    // Storage for the interface neighborhood, populated by getInterfaceNeighborhood()
    std::vector<DenseVertex> interfaceNeighborhood;

    // Check to see if Vertex is on an interface
    getInterfaceNeighborhood(denseV, interfaceNeighborhood, coutIndent + 4);
    if (interfaceNeighborhood.size() > 0)
    {
        if (fourthCheckVerbose_)
            std::cout << std::string(coutIndent + 2, ' ') + "Candidate vertex supports an interface" << std::endl;

        // Check for addition for spanner prop
        if (checkAddPath(denseV, interfaceNeighborhood, coutIndent + 4))
            return true;
    }
    else
    {
        if (fourthCheckVerbose_)
            std::cout << std::string(coutIndent + 2, ' ') + "Candidate vertex does NOT support an interface (no neighbors)"
                      << std::endl;
    }

    // All of the tests have failed.  Report failure for the sample
    return false;
}

void SparseDB::getInterfaceNeighborhood(const DenseVertex &denseV, std::vector<DenseVertex> &interfaceNeighborhood,
                                        std::size_t coutIndent)
{
    if (fourthCheckVerbose_)
        std::cout << std::string(coutIndent, ' ') + "getInterfaceNeighborhood()" << std::endl;

    // Error check
    assert(denseDelta_ > std::numeric_limits<double>::epsilon());

    // Get dense vertex's representative sparse vertex
    SparseVertex rep = boltDB_->representativesProperty_[denseV];

    // For each neighbor we are connected to
    foreach (DenseVertex neighborDenseV, boost::adjacent_vertices(denseV, boltDB_->g_))
    {
        // If neighbor representative is not our own
        if (boltDB_->representativesProperty_[neighborDenseV] != rep)
        {
            std::cout << "          Distance: " << std::setprecision(4) << boltDB_->distanceFunction(denseV, neighborDenseV)
                      << " denseDelta_: " << denseDelta_ << std::endl;

            // If it is within denseDelta_
            if (boltDB_->distanceFunction(denseV, neighborDenseV) < denseDelta_)
            {
                // Append him to the list
                interfaceNeighborhood.push_back(neighborDenseV);
            }
        }
    }
}
void SparseDB::findGraphNeighbors(const DenseVertex &denseV, std::vector<SparseVertex> &graphNeighborhood,
                                  std::vector<SparseVertex> &visibleNeighborhood, std::size_t coutIndent)
{
    if (checksVerbose_)
        std::cout << std::string(coutIndent, ' ') + "findGraphNeighbors() DenseV: " << denseV << std::endl;

    base::State *state = getDenseState(denseV);

    // Search
    getSparseState(queryVertex_) = state;
    nn_->nearestR(queryVertex_, sparseDelta_, graphNeighborhood);
    getSparseState(queryVertex_) = NULL;

    // Now that we got the neighbors from the NN, we must remove any we can't see
    for (std::size_t i = 0; i < graphNeighborhood.size(); ++i)
    {
        if (si_->checkMotion(state, getSparseState(graphNeighborhood[i])))
        {
            visibleNeighborhood.push_back(graphNeighborhood[i]);
        }
    }

    if (checksVerbose_)
        std::cout << std::string(coutIndent + 2, ' ') + "Graph neighborhood: " << graphNeighborhood.size()
                  << " | Visible neighborhood: " << visibleNeighborhood.size() << std::endl;
}

bool SparseDB::checkAddPath(DenseVertex q, const std::vector<DenseVertex> &neigh, std::size_t coutIndent)
{
    if (fourthCheckVerbose_)
        std::cout << std::string(coutIndent, ' ') + "checkAddPath() DenseVertex: " << q << std::endl;

    bool spannerPropertyViolated = false;

    // Get q's representative => v
    SparseVertex v = boltDB_->representativesProperty_[q];

    // Extract the representatives of neigh => n_rep
    std::set<SparseVertex> neighborReps;
    foreach (DenseVertex qp, neigh)
        neighborReps.insert(boltDB_->representativesProperty_[qp]);

    // Feedback
    if (neighborReps.empty())
        if (fourthCheckVerbose_)
            std::cout << std::string(coutIndent+2, ' ') + "neighborReps is empty" << std::endl;

    std::vector<SparseVertex> Xs;
    // for each v' in neighborReps
    for (std::set<SparseVertex>::iterator it = neighborReps.begin();
         it != neighborReps.end() && !spannerPropertyViolated; ++it)
    {
        if (fourthCheckVerbose_)
            std::cout << std::string(coutIndent+2, ' ') + "for neighborRep " << *it << std::endl;

        SparseVertex vp = *it;
        // Identify appropriate v" candidates => vpps
        std::vector<SparseVertex> VPPs;
        computeVPP(v, vp, VPPs);

        foreach (SparseVertex vpp, VPPs)
        {
            if (fourthCheckVerbose_)
                std::cout << std::string(coutIndent+4, ' ') + "for VPPs " << vpp << std::endl;

            double s_max = 0;
            // Find the X nodes to test
            computeX(v, vp, vpp, Xs);

            // For each x in xs
            foreach (SparseVertex x, Xs)
            {
                if (fourthCheckVerbose_)
                    std::cout << std::string(coutIndent+6, ' ') + "for Xs " << x << std::endl;

                // Compute/Retain MAXimum distance path thorugh S
                double dist = (si_->distance(getSparseStateConst(x), getSparseStateConst(v)) +
                               si_->distance(getSparseStateConst(v), getSparseStateConst(vp))) /
                              2.0;
                if (dist > s_max)
                    s_max = dist;
            }

            DensePath bestDPath;
            DenseVertex best_qpp = boost::graph_traits<DenseGraph>::null_vertex();
            double d_min = std::numeric_limits<double>::infinity();  // Insanely big number
            // For each vpp in vpps
            for (std::size_t j = 0; j < VPPs.size() && !spannerPropertyViolated; ++j)
            {
                if (fourthCheckVerbose_)
                    std::cout << std::string(coutIndent+6, ' ') + "for VPPs " << j << std::endl;

                SparseVertex vpp = VPPs[j];
                // For each q", which are stored interface nodes on v for i(vpp,v)
                foreach (DenseVertex qpp, interfaceListsProperty_[v].interfaceHash[vpp])
                {
                    if (fourthCheckVerbose_)
                        std::cout << std::string(coutIndent+8, ' ') + "for interfaceHsh " << qpp << std::endl;

                    // check that representatives are consistent
                    assert(boltDB_->representativesProperty_[qpp] == v);

                    // If they happen to be the one and same node
                    if (q == qpp)
                    {
                        bestDPath.push_front(getDenseState(q));
                        best_qpp = qpp;
                        d_min = 0;
                    }
                    else
                    {
                        // Compute/Retain MINimum distance path on D through q, q"
                        DensePath dPath;
                        boltDB_->computeDensePath(q, qpp, dPath);
                        if (dPath.size() > 0)
                        {
                            // compute path length
                            double length = 0.0;
                            DensePath::const_iterator jt = dPath.begin();
                            for (DensePath::const_iterator it = jt + 1; it != dPath.end(); ++it)
                            {
                                length += si_->distance(*jt, *it);
                                jt = it;
                            }

                            if (length < d_min)
                            {
                                d_min = length;
                                bestDPath.swap(dPath);
                                best_qpp = qpp;
                            }
                        }
                    }
                }

                // If the spanner property is violated for these paths
                if (s_max > stretchFactor_ * d_min)
                {
                    // Need to augment this path with the appropriate neighbor information
                    DenseVertex na = getInterfaceNeighbor(q, vp);
                    DenseVertex nb = getInterfaceNeighbor(best_qpp, vpp);

                    bestDPath.push_front(getDenseState(na));
                    bestDPath.push_back(getDenseState(nb));

                    // check consistency of representatives
                    assert(boltDB_->representativesProperty_[na] == vp && boltDB_->representativesProperty_[nb] == vpp);

                    // Add the dense path to the spanner
                    addPathToSpanner(bestDPath, vpp, vp);

                    // Report success
                    spannerPropertyViolated = true;
                }
            }
        }
    }
    return spannerPropertyViolated;
}

void SparseDB::computeVPP(SparseVertex v, SparseVertex vp, std::vector<SparseVertex> &VPPs)
{
    foreach (SparseVertex cvpp, boost::adjacent_vertices(v, g_))
        if (cvpp != vp)
            if (!boost::edge(cvpp, vp, g_).second)
                VPPs.push_back(cvpp);
}

void SparseDB::computeX(SparseVertex v, SparseVertex vp, SparseVertex vpp, std::vector<SparseVertex> &Xs)
{
    Xs.clear();
    foreach (SparseVertex cx, boost::adjacent_vertices(vpp, g_))
        if (boost::edge(cx, v, g_).second && !boost::edge(cx, vp, g_).second)
            if (interfaceListsProperty_[vpp].interfaceHash[cx].size() > 0)
                Xs.push_back(cx);
    Xs.push_back(vpp);
}

bool SparseDB::addPathToSpanner(const DensePath &densePath, SparseVertex vp, SparseVertex vpp)
{
    // First, check to see that the path has length
    if (densePath.size() <= 1)
    {
        // The path is 0 length, so simply link the representatives
        connectSparsePoints(vp, vpp);
    }
    else
    {
        // We will need to construct a PathGeometric to do this.
        smoothingGeomPath_.getStates().resize(densePath.size());
        std::copy(densePath.begin(), densePath.end(), smoothingGeomPath_.getStates().begin());

        // Attemp tto simplify the path
        psimp_->reduceVertices(smoothingGeomPath_, smoothingGeomPath_.getStateCount() * 2);

        // we are sure there are at least 2 points left on smoothingGeomPath_
        std::vector<SparseVertex> addedNodes;
        addedNodes.reserve(smoothingGeomPath_.getStateCount());
        for (std::size_t i = 0; i < smoothingGeomPath_.getStateCount(); ++i)
        {
            // Add each guard
            OMPL_ERROR("addVertex with state %u", smoothingGeomPath_.getState(i));
            exit(-1);
            //SparseVertex ng = addVertex(si_->cloneState(smoothingGeomPath_.getState(i)), QUALITY);
            //addedNodes.push_back(ng);
        }
        // Link them up
        for (std::size_t i = 1; i < addedNodes.size(); ++i)
        {
            connectSparsePoints(addedNodes[i - 1], addedNodes[i]);
        }
        // Don't forget to link them to their representatives
        connectSparsePoints(addedNodes[0], vp);
        connectSparsePoints(addedNodes[addedNodes.size() - 1], vpp);
    }
    smoothingGeomPath_.getStates().clear();
    return true;
}

void SparseDB::connectSparsePoints(SparseVertex v, SparseVertex vp)
{
    OMPL_ERROR("connectSparsePoints");
    exit(-1);
    // const base::Cost weight(costHeuristic(v, vp));
    // const SpannerGraph::edge_property_type properties(weight);
    // boost::mutex::scoped_lock _(graphMutex_);
    // boost::add_edge(v, vp, properties, g_);
    // sparseDJSets_.union_set(v, vp);
}

DenseVertex SparseDB::getInterfaceNeighbor(DenseVertex q, SparseVertex rep)
{
    foreach (DenseVertex vp, boost::adjacent_vertices(q, g_))
        if (boltDB_->representativesProperty_[vp] == rep)
            if (distanceFunction(q, vp) <= denseDelta_)
                return vp;
    throw Exception("SparseDB", "Vertex has no interface neighbor with given representative");
}

bool SparseDB::sameComponent(SparseVertex m1, SparseVertex m2)
{
    return boost::same_component(m1, m2, disjointSets_);
}

SparseVertex SparseDB::addVertex(DenseVertex denseV, const GuardType &type)
{
    // Create vertex
    SparseVertex v = boost::add_vertex(g_);

    // Add properties
    typePropertySparse_[v] = type;
    denseVertexProperty_[v] = denseV;

    // Connected component tracking
    disjointSets_.make_set(v);

    // Add vertex to nearest neighbor structure
    nn_->add(v);

    // Visualize
    if (visualizeSparsCreation_)
    {
        // visual_->viz2StateCallback(getSparseState(v), /*mode=*/ getVizVertexType(type), sparseDelta_);
        visual_->viz2StateCallback(getSparseState(v), /*mode=*/4, sparseDelta_);
        visual_->viz2TriggerCallback();
        usleep(0.01 * 1000000);
        //usleep(0.1 * 1000000);
    }

    return v;
}

std::size_t SparseDB::getVizVertexType(const GuardType &type)
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

void SparseDB::addEdge(SparseVertex v1, SparseVertex v2, std::size_t visualColor, std::size_t coutIndent)
{
    assert(v1 <= getNumVertices());
    assert(v2 <= getNumVertices());
    assert(v1 != v2);

    //std::cout << std::string(coutIndent, ' ') + "addEdge: Connecting vertex " << v1 << " to vertex " << v2 << std::endl;

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
           75 - connectivity second round
           100 - interface second round
        */
        visual_->viz2EdgeCallback(getSparseState(v1), getSparseState(v2), visualColor);
        visual_->viz2TriggerCallback();
        usleep(0.001*1000000);
    }
}

base::State *&SparseDB::getSparseState(const SparseVertex &v)
{
    return boltDB_->stateProperty_[denseVertexProperty_[v]];
}

const base::State *SparseDB::getSparseStateConst(const SparseVertex &v) const
{
    return boltDB_->stateProperty_[denseVertexProperty_[v]];
}

base::State *&SparseDB::getDenseState(const DenseVertex &denseV)
{
    return boltDB_->stateProperty_[denseV];
}

void SparseDB::displayDatabase(bool showVertices)
{
    OMPL_INFORM("Displaying Sparse database");

    // Error check
    if (getNumVertices() == 0 || getNumEdges() == 0)
    {
        OMPL_ERROR("Unable to show database because no vertices/edges available");
        exit(-1);
    }

    if (showVertices)
    {
        // Loop through each vertex
        std::size_t count = 0;
        std::size_t debugFrequency = getNumVertices() / 10;
        std::cout << "Displaying database: " << std::flush;
        foreach (SparseVertex v, boost::vertices(g_))
        {
            // Check for null states
            if (getSparseStateConst(v))
            {
                visual_->viz2StateCallback(getSparseStateConst(v), 6, 1);
            }
            else
                OMPL_WARN("Null sparse state found on vertex %u", v);

            // Prevent cache from getting too big
            if (count % debugFrequency == 0)
            {
                std::cout << std::fixed << std::setprecision(0)
                          << (static_cast<double>(count + 1) / getNumVertices()) * 100.0 << "% " << std::flush;
                visual_->viz2TriggerCallback();
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
        std::cout << "Displaying database: " << std::flush;
        foreach (SparseEdge e, boost::edges(g_))
        {
            // Add edge
            const SparseVertex &v1 = boost::source(e, g_);
            const SparseVertex &v2 = boost::target(e, g_);

            // Visualize
            visual_->viz2EdgeCallback(getSparseStateConst(v1), getSparseStateConst(v2), edgeWeightPropertySparse_[e]);

            // Prevent cache from getting too big
            if (count % debugFrequency == 0)
            {
                std::cout << std::fixed << std::setprecision(0)
                          << (static_cast<double>(count + 1) / getNumEdges()) * 100.0 << "% " << std::flush;
                visual_->viz2TriggerCallback();
            }

            count++;
        }
        std::cout << std::endl;
    }

    // Publish remaining edges
    visual_->viz2TriggerCallback();
}

}  // namespace bolt
}  // namespace tools
}  // namespace ompl
