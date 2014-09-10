/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick
*  Copyright (c) 2014, University of Colorado, Boulder
*  All Rights Reserved.
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
*   * Neither the name of Rutgers University nor the names of its
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

/* Author: Andrew Dobson, Dave Coleman */

#include "ompl/geometric/planners/prm/SPARStwo.h"
#include "ompl/geometric/planners/prm/ConnectionStrategy.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/tools/debug/Profiler.h" // temp
#include <boost/lambda/bind.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/foreach.hpp>

#include "GoalVisitor.hpp"

#define foreach BOOST_FOREACH
#define foreach_reverse BOOST_REVERSE_FOREACH

// edgeWeightMap methods ////////////////////////////////////////////////////////////////////////////

BOOST_CONCEPT_ASSERT((boost::ReadablePropertyMapConcept<ompl::geometric::SPARStwo::edgeWeightMap, ompl::geometric::SPARStwo::Edge>));

ompl::geometric::SPARStwo::edgeWeightMap::edgeWeightMap (const Graph &graph, 
                                                         const EdgeCollisionStateMap &collisionStates)
    : g_(graph), 
      collisionStates_(collisionStates)
{
}

double ompl::geometric::SPARStwo::edgeWeightMap::get (Edge e) const
{
    // Debug
    if (true) 
    {
        std::cout << "ompl::geometric::SPARStwo::edgeWeightMap::get " << e;
        if (collisionStates_[e] == IN_COLLISION)
            std::cout << " IN COLLSIION ";
        if (collisionStates_[e] == FREE)
            std::cout << " FREE ";
        if (collisionStates_[e] == NOT_CHECKED)
            std::cout << " NOT CHECKED YET ";
        std::cout << "with weight " << boost::get(boost::edge_weight, g_, e) << std::endl;
    }

    // Get the status of collision checking for this edge
    if (collisionStates_[e] == IN_COLLISION)
        return std::numeric_limits<double>::infinity();

    return boost::get(boost::edge_weight, g_, e);
}
        
namespace boost
{
    double get (const ompl::geometric::SPARStwo::edgeWeightMap &m, const ompl::geometric::SPARStwo::Edge &e)
    {
        return m.get(e);
    }
}

/*namespace boost
{
    double get (const ompl::geometric::SPARStwo::Graph &g, const ompl::geometric::SPARStwo::Edge &e)
    {
        return g.get(e);
    }
}
*/

// CustomVisitor methods ////////////////////////////////////////////////////////////////////////////

BOOST_CONCEPT_ASSERT((boost::AStarVisitorConcept<ompl::geometric::SPARStwo::CustomVisitor, ompl::geometric::SPARStwo::Graph>));

ompl::geometric::SPARStwo::CustomVisitor::CustomVisitor (Vertex goal)
: goal(goal)
{
}

void ompl::geometric::SPARStwo::CustomVisitor::examine_vertex (Vertex u, const Graph &) const
{
    if (u == goal)
        throw AStarFoundGoal(); //foundGoalException();
}

// SPARStwo methods ////////////////////////////////////////////////////////////////////////////////////////

ompl::geometric::SPARStwo::SPARStwo(const base::SpaceInformationPtr &si) :
    base::Planner(si, "SPARStwo"),
    // Numeric variables
    stretchFactor_(3.),
    sparseDeltaFraction_(.25),
    denseDeltaFraction_(.001),
    maxFailures_(5000),
    nearSamplePoints_((2*si_->getStateDimension())),
    // Property accessors of edges
    edgeWeightProperty_(boost::get(boost::edge_weight, g_)),
    edgeCollisionStateProperty_(boost::get(edge_collision_state_t(), g_)),
    // Property accessors of vertices
    stateProperty_(boost::get(vertex_state_t(), g_)),
    colorProperty_(boost::get(vertex_color_t(), g_)),
    interfaceDataProperty_(boost::get(vertex_interface_data_t(), g_)),
    // Disjoint set acecssors
    disjointSets_(boost::get(boost::vertex_rank, g_),
                  boost::get(boost::vertex_predecessor, g_)),
    addedSolution_(false),
    consecutiveFailures_(0),
    iterations_(0),
    sparseDelta_(0.),
    denseDelta_(0.)
{
    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.approximateSolutions = false;
    specs_.optimizingPaths = true;

    psimp_.reset(new PathSimplifier(si_));

    Planner::declareParam<double>("stretch_factor", this, &SPARStwo::setStretchFactor, &SPARStwo::getStretchFactor, "1.1:0.1:3.0");
    Planner::declareParam<double>("sparse_delta_fraction", this, &SPARStwo::setSparseDeltaFraction, &SPARStwo::getSparseDeltaFraction, "0.0:0.01:1.0");
    Planner::declareParam<double>("dense_delta_fraction", this, &SPARStwo::setDenseDeltaFraction, &SPARStwo::getDenseDeltaFraction, "0.0:0.0001:0.1");
    Planner::declareParam<unsigned int>("max_failures", this, &SPARStwo::setMaxFailures, &SPARStwo::getMaxFailures, "100:10:3000");
}

ompl::geometric::SPARStwo::~SPARStwo()
{
    freeMemory();
}

void ompl::geometric::SPARStwo::setup()
{
    Planner::setup();
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(si_->getStateSpace()));
    nn_->setDistanceFunction(boost::bind(&SPARStwo::distanceFunction, this, _1, _2));
    double maxExt = si_->getMaximumExtent();
    sparseDelta_ = sparseDeltaFraction_ * maxExt;
    denseDelta_ = denseDeltaFraction_ * maxExt;
}

void ompl::geometric::SPARStwo::setProblemDefinition(const base::ProblemDefinitionPtr &pdef)
{
    Planner::setProblemDefinition(pdef);
    clearQuery();
}

void ompl::geometric::SPARStwo::clearQuery()
{
    startM_.clear();
    goalM_.clear();
    pis_.restart();
}

void ompl::geometric::SPARStwo::clear()
{
    Planner::clear();
    clearQuery();
    resetFailures();
    iterations_ = 0;
    freeMemory();
    if (nn_)
        nn_->clear();
}

void ompl::geometric::SPARStwo::freeMemory()
{
    Planner::clear();
    sampler_.reset();
    simpleSampler_.reset();

    foreach (Vertex v, boost::vertices(g_))
    {
        foreach (InterfaceData &d, interfaceDataProperty_[v].interfaceHash | boost::adaptors::map_values)
            d.clear(si_);
        if( stateProperty_[v] != NULL )
            si_->freeState(stateProperty_[v]);
        stateProperty_[v] = NULL;
    }
    g_.clear();

    if (nn_)
        nn_->clear();
}

bool ompl::geometric::SPARStwo::getSimilarPaths(int nearestK, const base::State* start, const base::State* goal, 
                                                CandidateSolution &candidateSolution,
                                                const base::PlannerTerminationCondition &ptc)
{
    ompl::tools::Profiler::Clear();
    ompl::tools::Profiler::Start();

    // TODO: nearestK unused

    // Get neighbors near start and goal. Note: potentially they are not *visible* - will test for this later

    // Start
    if (!findGraphNeighbors(start, startVertexCandidateNeighbors_))
    {
        OMPL_INFORM("No graph neighbors found for start");                
        return false;
    }
    std::cout << "Found " << startVertexCandidateNeighbors_.size() << " nodes near start" << std::endl;

    // Goal
    if (!findGraphNeighbors(goal, goalVertexCandidateNeighbors_))
    {
        OMPL_INFORM("No graph neighbors found for goal");
        return false;
    }
    std::cout << "Found " << goalVertexCandidateNeighbors_.size() << " nodes near goal" << std::endl;

    // Get paths between start and goal
    ompl::tools::Profiler::Begin("SPARStwo::getSimilarPaths::getPaths");
    bool result = getPaths(startVertexCandidateNeighbors_, goalVertexCandidateNeighbors_, 
                               start, goal, candidateSolution, ptc);
    ompl::tools::Profiler::End("SPARStwo::getSimilarPaths::getPaths");

    // Error check
    if (!result)
    {
        OMPL_INFORM("getSimilarPaths(): SPARStwo returned FALSE for getPaths");
        return false;
    }
    if (!candidateSolution.path_)
    {
        OMPL_ERROR("getSimilarPaths(): SPARStwo returned solution is NULL");
        return false;
    }

    // Debug output
    if (false)
    {
        ompl::geometric::PathGeometric geometricSolution 
            = static_cast<ompl::geometric::PathGeometric&>(*candidateSolution.path_);

        for (std::size_t i = 0; i < geometricSolution.getStateCount(); ++i)
        {
            std::cout << "  getSimilarPaths(): Adding state " << i << " to plannerData"  << std::endl;
            std::cout << "  State: " << geometricSolution.getState(i) << std::endl;
            si_->printState(geometricSolution.getState(i), std::cout);
            std::cout << std::endl;
        }
    }

    return result;
}

bool ompl::geometric::SPARStwo::getPaths(const std::vector<Vertex> &candidateStarts, 
                                         const std::vector<Vertex> &candidateGoals, 
                                         const base::State* actualStart, 
                                         const base::State* actualGoal,
                                         CandidateSolution &candidateSolution,
                                         const base::PlannerTerminationCondition &ptc)
{
    // Vector to store candidate paths in before they are converted to PathPtrs
    std::vector<Vertex> vertexPath;

    base::Goal *g = pdef_->getGoal().get(); // for checking isStartGoalPairValid

    // Try every combination of nearby start and goal pairs
    foreach (Vertex start, candidateStarts)
    {
        std::cout << "Checking motion from  vertex " << actualStart << " to " << stateProperty_[start] << std::endl;

        ompl::tools::Profiler::Begin("SPARStwo::getPaths::checkMotion start");
        // Check if this start is visible from the actual start
        if (!si_->checkMotion(actualStart, stateProperty_[start]))
        {
            std::cout << "FOUND CANDIDATE START THAT IS NOT VISIBLE !!!!!!!!!!!!!!!!!!! " << std::endl;
            continue; // this is actually not visible
        }
        ompl::tools::Profiler::End("SPARStwo::getPaths::checkMotion start");

        foreach (Vertex goal, candidateGoals)
        {
            std::cout << "Checking motion from  " << actualGoal << " to " << stateProperty_[goal] << std::endl;

            ompl::tools::Profiler::Begin("SPARStwo::getPaths::checkMotion goal");
            // Check if this goal is visible from the actual goal
            if (!si_->checkMotion(actualGoal, stateProperty_[goal]))
            {
                std::cout << "FOUND CANDIDATE GOAL THAT IS NOT VISIBLE !!!!!!!!!!!!!!!!!!! " << std::endl;
                continue; // this is actually not visible
            }
            ompl::tools::Profiler::End("SPARStwo::getPaths::checkMotion goal");

            // we lock because the connected components algorithm is incremental and may change disjointSets_
            /*
            ompl::tools::Profiler::Begin("SPARStwo::getPaths::sameComponent");
            graphMutex_.lock();

            // decide if start and goal are connected 
            // TODO this does not compute dynamic graphcs
            // i.e. it will say its the same components even when an edge has been disabled
            bool same_component = sameComponent(start, goal);
            graphMutex_.unlock();
            ompl::tools::Profiler::End("SPARStwo::getPaths::sameComponent");
            */
            bool same_component = true; // hacked for testing

            // Check if the chosen start and goal can be used together to satisfy problem
            if (same_component && g->isStartGoalPairValid(stateProperty_[goal], stateProperty_[start]))
            {
                // Make sure that the start and goal aren't so close together that they find the same vertex
                if (start == goal)
                {
                    std::cout << "Start equals goal!!! " << std::endl;
                    continue; // skip this pair
                }

                std::cout << "getPaths: found connection between vertex " << start << " and " 
                          << goal << std::endl;

                // Keep looking for paths between chosen start and goal until one is found that is valid,
                // or no further paths can be found between them because of disabled edges
                bool havePartialSolution = false;
                while (true)
                {
                    std::cout << "while true... " << std::endl;

                    // Check if our planner is out of time
                    if (ptc == true)
                    {
                        OMPL_DEBUG("getPaths function interrupted because termination condition is true.");
                        std::cout << OMPL_CONSOLE_COLOR_RESET;
                        return false;
                    }

                    // Attempt to find a solution from start to goal
                    if (!constructSolution(start, goal, vertexPath))
                    {
                        std::cout << "unable to construct solution " << std::endl;
                        // no solution path found. check if a previous partially correct solution was found
                        if (havePartialSolution)
                        {
                            std::cout << "has partial solution " << std::endl;
                            convertVertexPathToStatePath(vertexPath, actualStart, actualGoal, candidateSolution);
                            return true; // we previously found a path that is at least partially valid
                        }
                            
                        std::cout << "  no partial solution " << std::endl;
                        // no path found what so ever
                        return false;
                    }
                    havePartialSolution = true; // we have found at least one path at this point. may be invalid

                    std::cout << "has at least a partial solution, maybe exact solution" << std::endl;
                    std::cout << "Solution has " << vertexPath.size() << " vertices" << std::endl;

                    // Check if all the points in the potential solution are valid
                    if (lazyCollisionCheck(vertexPath, ptc))
                    {
                        std::cout << "lazy collision check returned valid " << std::endl;
                            
                        // the path is valid, we are done!
                        convertVertexPathToStatePath(vertexPath, actualStart, actualGoal, candidateSolution);
                        return true;
                    }
                    // else, loop with updated graph that has the invalid edges/states disabled
                } // while
            } // if
        } // foreach
    } // foreach

    return false;
}

bool ompl::geometric::SPARStwo::constructSolution(const Vertex start, const Vertex goal,
                                                  std::vector<Vertex> &vertexPath) const
{
    ompl::tools::Profiler::ScopedBlock("SPARStwo::constructSolution");
    Vertex *vertexPredecessors = new Vertex[boost::num_vertices(g_)];
    bool foundGoal = false;

    double *vertexDistances = new double[boost::num_vertices(g_)];

    try
    {
        boost::astar_search(g_, // graph
                            start, // start state
                            boost::bind(&SPARStwo::distanceFunction, this, _1, goal), // the heuristic
                            // ability to disable edges (set cost to inifinity):
                            boost::weight_map(edgeWeightMap(g_, edgeCollisionStateProperty_)).
                            predecessor_map(vertexPredecessors).
                            distance_map(&vertexDistances[0]).
                            //visitor(AStarGoalVisitor<Vertex>(goal)));
                            visitor(CustomVisitor(goal)));
    }
    //catch (ompl::geometric::SPARStwo::foundGoalException&)
    catch (AStarFoundGoal&) // the default exception for AStarGoalVisitor
    {
        std::cout << std::endl;
        // the custom exception from CustomVisitor      
        std::cout << "constructSolution: Astar found goal vertex ------------------------" << std::endl;
        std::cout << "distance to goal: " << vertexDistances[goal] << std::endl;
        std::cout << std::endl;

        if (vertexDistances[goal] > 1.7e+308) // terrible hack for detecting infinity
        //double diff = d[goal] - std::numeric_limits<double>::infinity();
        //std::cout << "Diff from inifinity is " << diff << std::endl;
        //if ((diff < std::numeric_limits<double>::epsilon()) && (-diff < std::numeric_limits<double>::epsilon()))
        // check if the distance to goal is inifinity. if so, it is unreachable
        //if (d[goal] >= std::numeric_limits<double>::infinity())
        {
            std::cout << "Distance to goal is infinity" << std::endl;
            foundGoal = false;
        }
        else
        {
            // Only clear the vertexPath after we know we have a new solution, otherwise it might have a good
            // previous one
            vertexPath.clear(); // remove any old solutions

            // Trace back the shortest path in reverse and only save the states
            Vertex v;
            for (v = goal; v != vertexPredecessors[v]; v = vertexPredecessors[v])
            {
                vertexPath.push_back(v);
            }
            if (v != goal) // TODO explain this because i don't understand
            {
                vertexPath.push_back(v);
            }

            foundGoal = true;
        }
    }

    delete[] vertexPredecessors;
    delete[] vertexDistances;

    // No solution found from start to goal
    return foundGoal;
}

bool ompl::geometric::SPARStwo::lazyCollisionCheck(std::vector<Vertex> &vertexPath,
                                                   const base::PlannerTerminationCondition &ptc)
{
    ompl::tools::Profiler::ScopedBlock("SPARStwo::lazyCollisionCheck");
    std::cout << OMPL_CONSOLE_COLOR_BROWN << std::endl;
    std::cout << "Lazy collision check " << std::endl;

    bool hasInvalidEdges = false;

    // Initialize
    Vertex fromVertex = vertexPath[0];
    Vertex toVertex;
    
    // Loop through every pair of states and make sure path is valid.
    for (std::size_t toID = 1; toID < vertexPath.size(); ++toID)
    {
        // Increment location on path
        toVertex = vertexPath[toID];

        // Check if our planner is out of time
        if (ptc == true)
        {
            OMPL_DEBUG("Lazy collision check function interrupted because termination condition is true.");
            std::cout << OMPL_CONSOLE_COLOR_RESET;
            return false;
        }

        Edge thisEdge = boost::edge(fromVertex, toVertex, g_).first;
        //std::cout << "On edge  " << thisEdge << std::endl;

        // Has this edge already been checked before?
        if (edgeCollisionStateProperty_[thisEdge] == NOT_CHECKED)
        {
            // Check path between states
            if (!si_->checkMotion(stateProperty_[fromVertex], stateProperty_[toVertex]))
            {
                // Path between (from, to) states not valid, disable the edge
                std::cout << "  DISABLING EDGE from vertex " << fromVertex << " to vertex " << toVertex << std::endl;

                // Disable edge
                edgeCollisionStateProperty_[thisEdge] = IN_COLLISION;
            }
            else
            {
                // Mark edge as free so we no longer need to check for collision
                edgeCollisionStateProperty_[thisEdge] = FREE;
            }
        }
        else
        {
            std::cout << "Skipping motion check for edge because collision state is already " << edgeCollisionStateProperty_[thisEdge] << std::endl;
        }
        
        // Check final result
        if (edgeCollisionStateProperty_[thisEdge] == IN_COLLISION)
        {
            // Remember that this path is no longer valid, but keep checking remainder of path edges
            hasInvalidEdges = true;            
        }

        // switch vertex focus
        fromVertex = toVertex;
    }

    OMPL_INFORM("Done lazy collision checking ---------------------------");
    std::cout << OMPL_CONSOLE_COLOR_RESET << std::endl;

    // TODO: somewhere in the code we need to reset all edges collision status back to NOT_CHECKED for future queries

    // Only return true if nothing was found invalid
    return !hasInvalidEdges;
}

bool ompl::geometric::SPARStwo::sameComponent(Vertex m1, Vertex m2)
{
    return boost::same_component(m1, m2, disjointSets_);
}

bool ompl::geometric::SPARStwo::reachedFailureLimit() const
{
    return consecutiveFailures_ >= maxFailures_;
}

void ompl::geometric::SPARStwo::printDebug(std::ostream &out) const
{
    out << "SPARStwo Debug Output: " << std::endl;
    out << "  Settings: " << std::endl;
    out << "    Max Failures: " << getMaxFailures() << std::endl;
    out << "    Dense Delta Fraction: " << getDenseDeltaFraction() << std::endl;
    out << "    Sparse Delta Fraction: " << getSparseDeltaFraction() << std::endl;
    out << "    Stretch Factor: " << getStretchFactor() << std::endl;
    out << "  Status: " << std::endl;
    out << "    Milestone Count: " << milestoneCount() << std::endl;
    //    out << "    Guard Count: " << guardCount() << std::endl;
    out << "    Iterations: " << getIterations() << std::endl;
    //    out << "    Average Valence: " << averageValence() << std::endl;
    out << "    Consecutive Failures: " << consecutiveFailures_ << std::endl;
    out << "    Number of guards: " << nn_->size() << std::endl << std::endl;
}

bool ompl::geometric::SPARStwo::reachedTerminationCriterion() const
{
    return consecutiveFailures_ >= maxFailures_ || addedSolution_;
}

void ompl::geometric::SPARStwo::addPathToRoadmap(const base::PlannerTerminationCondition &ptc,
                                                 ompl::geometric::PathGeometric& solutionPath)
{
    // Check that the query vertex is initialized (used for internal nearest neighbor searches)
    checkQueryStateInitialization();

    // Error check
    if (solutionPath.getStateCount() < 2)
    {
        OMPL_ERROR("Less than 2 states were passed to addPathToRoadmap in the solution path");
        return;
    }

    // Fill in the gaps of the states
    solutionPath.interpolate();
    std::cout << "Attempting to add  " << solutionPath.getStateCount() << " states to roadmap" << std::endl;

    // Try to add the start and goal first, but don't force it
    addStateToRoadmap(ptc, solutionPath.getState(0));
    addStateToRoadmap(ptc, solutionPath.getState(solutionPath.getStateCount() - 1));

    // Add solution states to SPARStwo one by one

    // Create a vector of shuffled indexes
    std::vector<std::size_t> shuffledIDs;
    for (std::size_t i = 1; i < solutionPath.getStateCount() - 1; ++i)  // skip 0 and last because those are start/goal and are already added
      shuffledIDs.push_back(i); // 1 2 3...
    std::random_shuffle ( shuffledIDs.begin(), shuffledIDs.end() ); // using built-in random generator:

    // Add each state randomly
    for (std::size_t i = 0; i < shuffledIDs.size(); ++i)
    {
        //std::cout << "Adding state " << shuffledIDs[i] << " of " << solutionPath.getStateCount() << std::endl;

      // Add a single state to the roadmap
      addStateToRoadmap(ptc, solutionPath.getState(shuffledIDs[i]));
    }

}

void ompl::geometric::SPARStwo::addStateToRoadmap(const base::PlannerTerminationCondition &ptc, base::State *newState)
{
    bool local_verbose = true;

    // Check that the query vertex is initialized (used for internal nearest neighbor searches)
    checkQueryStateInitialization();

    if (!sampler_)
        sampler_ = si_->allocValidStateSampler();
    if (!simpleSampler_)
        simpleSampler_ = si_->allocStateSampler();

    // Deep copy
    base::State *qNew = si_->cloneState(newState);

    base::State *workState = si_->allocState();

    /* The whole neighborhood set which has been most recently computed */
    std::vector<Vertex> graphNeighborhood;
    /* The visible neighborhood set which has been most recently computed */
    std::vector<Vertex> visibleNeighborhood;

    ++iterations_;
    ++consecutiveFailures_;

    findGraphNeighbors(qNew, graphNeighborhood, visibleNeighborhood);

    if (local_verbose)
        std::cout << "graph neighborhood: " << graphNeighborhood.size() << " | visible neighborhood: " << visibleNeighborhood.size() << std::endl;

    if (local_verbose)
        std::cout << " - checkAddCoverage() Are other nodes around it visible?" << std::endl;
    if (!checkAddCoverage(qNew, visibleNeighborhood)) // Always add a node if no other nodes around it are visible (GUARD)
    {
        if (local_verbose)
            std::cout << " -- checkAddConnectivity() Does this node connect neighboring nodes that are not connected? " << std::endl;
        if (!checkAddConnectivity(qNew, visibleNeighborhood))
        {
            if (local_verbose)
                std::cout << " --- checkAddInterface() Does this node's neighbor's need it to better connect them? " << std::endl;
            if (!checkAddInterface(qNew, graphNeighborhood, visibleNeighborhood))
            {
                if (local_verbose)
                    std::cout << " ---- Ensure SPARS asymptotic optimality" << std::endl;
                if (visibleNeighborhood.size() > 0)
                {
                    std::map<Vertex, base::State*> closeRepresentatives;
                    if (local_verbose)
                        std::cout << " ----- findCloseRepresentatives()" << std::endl;
                    findCloseRepresentatives(workState, qNew, visibleNeighborhood[0], closeRepresentatives, ptc);
                    for (std::map<Vertex, base::State*>::iterator it = closeRepresentatives.begin(); it != closeRepresentatives.end(); ++it)
                    {
                        if (local_verbose)
                            std::cout << " ------ Looping through close representatives" << std::endl;
                        updatePairPoints(visibleNeighborhood[0], qNew, it->first, it->second);
                        updatePairPoints(it->first, it->second, visibleNeighborhood[0], qNew);
                    }
                    if (local_verbose)
                        std::cout << " ----- checkAddPath()" << std::endl;
                    checkAddPath(visibleNeighborhood[0]);
                    for (std::map<Vertex, base::State*>::iterator it = closeRepresentatives.begin(); it != closeRepresentatives.end(); ++it)
                    {
                        if (local_verbose)
                            std::cout << " ------ Looping through close representatives to add path" << std::endl;
                        checkAddPath(it->first);
                        si_->freeState(it->second);
                    }
                }
            }
        }
    }

    si_->freeState(workState);
    si_->freeState(qNew);

    if (local_verbose)
    {
        std::cout << std::endl;
        std::cout << std::endl;
    } 
}

void ompl::geometric::SPARStwo::checkQueryStateInitialization()
{
    boost::mutex::scoped_lock _(graphMutex_);
    if (boost::num_vertices(g_) < 1)
    {
        queryVertex_ = boost::add_vertex( g_ );
        stateProperty_[queryVertex_] = NULL;
        //visualizeCallback();
    }
}

ompl::base::PlannerStatus ompl::geometric::SPARStwo::solve(const base::PlannerTerminationCondition &ptc)
{
    // Disabled
    return base::PlannerStatus::TIMEOUT;
}

bool ompl::geometric::SPARStwo::checkAddCoverage(const base::State *qNew, std::vector<Vertex> &visibleNeighborhood)
{
    if (visibleNeighborhood.size() > 0)
        return false;
    //No free paths means we add for coverage
    std::cout << " --   Adding node for COVERAGE " << std::endl;
    addGuard(si_->cloneState(qNew), COVERAGE);
    return true;
}

bool ompl::geometric::SPARStwo::checkAddConnectivity(const base::State *qNew, std::vector<Vertex> &visibleNeighborhood)
{
    std::vector<Vertex> links;
    if (visibleNeighborhood.size() > 1)
    {
        //For each neighbor
        for (std::size_t i = 0; i < visibleNeighborhood.size(); ++i)
            //For each other neighbor
            for (std::size_t j = i + 1; j < visibleNeighborhood.size(); ++j)
                //If they are in different components
                if (!sameComponent(visibleNeighborhood[i], visibleNeighborhood[j]))
                {
                    links.push_back(visibleNeighborhood[i]);
                    links.push_back(visibleNeighborhood[j]);
                }

        if (links.size() > 0)
        {
            std::cout << " --   Adding node for CONNECTIVITY " << std::endl;
            //Add the node
            Vertex g = addGuard(si_->cloneState(qNew), CONNECTIVITY);

            for (std::size_t i = 0; i < links.size() ; ++i)
                //If there's no edge
                if (!boost::edge(g, links[i], g_).second)
                    //And the components haven't been united by previous links
                    if (!sameComponent(links[i], g))
                        connectGuards(g, links[i]);
            return true;
        }
    }
    return false;
}

bool ompl::geometric::SPARStwo::checkAddInterface(const base::State *qNew, std::vector<Vertex> &graphNeighborhood, std::vector<Vertex> &visibleNeighborhood)
{
    //If we have more than 1 or 0 neighbors
    if (visibleNeighborhood.size() > 1)
        if (graphNeighborhood[0] == visibleNeighborhood[0] && graphNeighborhood[1] == visibleNeighborhood[1])
            //If our two closest neighbors don't share an edge
            if (!boost::edge(visibleNeighborhood[0], visibleNeighborhood[1], g_).second)
            {
                //If they can be directly connected
                if (si_->checkMotion(stateProperty_[visibleNeighborhood[0]], stateProperty_[visibleNeighborhood[1]]))
                {
                    //Connect them
                    std::cout << " ---   INTERFACE: directly connected nodes " << std::endl;
                    connectGuards(visibleNeighborhood[0], visibleNeighborhood[1]);
                    //And report that we added to the roadmap
                    resetFailures();
                    //Report success
                    return true;
                }
                else
                {
                    //Add the new node to the graph, to bridge the interface
                    std::cout << " --   Adding node for INTERFACE  " << std::endl;
                    Vertex v = addGuard(si_->cloneState(qNew), INTERFACE);
                    connectGuards(v, visibleNeighborhood[0]);
                    connectGuards(v, visibleNeighborhood[1]);
                    std::cout << " ---   INTERFACE: connected two neighbors through new interface node " << std::endl;
                    //Report success
                    return true;
                }
            }
    return false;
}

bool ompl::geometric::SPARStwo::checkAddPath( Vertex v )
{
    bool ret = false;

    std::vector< Vertex > rs;
    foreach( Vertex r, boost::adjacent_vertices( v, g_ ) )
        rs.push_back(r);

    /* Candidate x vertices as described in the method, filled by function computeX(). */
    std::vector<Vertex> Xs;

    /* Candidate v" vertices as described in the method, filled by function computeVPP(). */
    std::vector<Vertex> VPPs;

    for (std::size_t i = 0; i < rs.size() && !ret; ++i)
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
            if (rm_dist > stretchFactor_ * d.d_)
            {
                ret = true; //Report that we added for the path
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
                            std::cout << " --   Adding node for QUALITY" << std::endl;
                            vnew = addGuard(st , QUALITY);

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

    return ret;
}

void ompl::geometric::SPARStwo::resetFailures()
{
    consecutiveFailures_ = 0;
}

void ompl::geometric::SPARStwo::findGraphNeighbors(base::State *st, std::vector<Vertex> &graphNeighborhood, 
                                                   std::vector<Vertex> &visibleNeighborhood)
{
    visibleNeighborhood.clear();
    stateProperty_[ queryVertex_ ] = st;
    nn_->nearestR( queryVertex_, sparseDelta_, graphNeighborhood);
    stateProperty_[ queryVertex_ ] = NULL;

    //Now that we got the neighbors from the NN, we must remove any we can't see
    for (std::size_t i = 0; i < graphNeighborhood.size() ; ++i )
        if (si_->checkMotion(st, stateProperty_[graphNeighborhood[i]]))
            visibleNeighborhood.push_back(graphNeighborhood[i]);
}

bool ompl::geometric::SPARStwo::findGraphNeighbors(const base::State *state, std::vector<Vertex> &graphNeighborhood)
{
    base::State* stateCopy = si_->cloneState(state);

    // Don't check for visibility
    graphNeighborhood.clear();
    stateProperty_[ queryVertex_ ] = stateCopy;

    // Double the range of sparseDelta_ up to 3 times until at least 1 neighbor is found
    std::size_t expandNeighborhoodSearchAttempts = 1;
    for (std::size_t i = 0; i < expandNeighborhoodSearchAttempts; ++i)
    {
        std::cout << "Attempt " << i+1 << " to find neighborhood at range " <<  sparseDelta_ * (i+1) << std::endl;
        nn_->nearestR( queryVertex_, sparseDelta_ * (i+1), graphNeighborhood);

        // Check if at least one neighbor found
        if (graphNeighborhood.size() > 0)
            break;
    }
    stateProperty_[ queryVertex_ ] = NULL;

    // Check if no neighbors found
    if (!graphNeighborhood.size())
    {
        return false;
    }    
    return true;
}

void ompl::geometric::SPARStwo::approachGraph( Vertex v )
{
    std::vector< Vertex > hold;
    nn_->nearestR( v, sparseDelta_, hold );

    std::vector< Vertex > neigh;
    for (std::size_t i = 0; i < hold.size(); ++i)
        if (si_->checkMotion( stateProperty_[v], stateProperty_[hold[i]]))
            neigh.push_back( hold[i] );

    foreach (Vertex vp, neigh)
        connectGuards(v, vp);
}

ompl::geometric::SPARStwo::Vertex ompl::geometric::SPARStwo::findGraphRepresentative(base::State *st)
{
    std::vector<Vertex> nbh;
    stateProperty_[ queryVertex_ ] = st;
    nn_->nearestR( queryVertex_, sparseDelta_, nbh);
    stateProperty_[queryVertex_] = NULL;

    Vertex result = boost::graph_traits<Graph>::null_vertex();

    for (std::size_t i = 0 ; i< nbh.size() ; ++i)
        if (si_->checkMotion(st, stateProperty_[nbh[i]]))
        {
            result = nbh[i];
            break;
        }
    return result;
}

void ompl::geometric::SPARStwo::findCloseRepresentatives(base::State *workArea, const base::State *qNew, const Vertex qRep,
                                                         std::map<Vertex, base::State*> &closeRepresentatives,
                                                         const base::PlannerTerminationCondition &ptc)
{
    for (std::map<Vertex, base::State*>::iterator it = closeRepresentatives.begin(); it != closeRepresentatives.end(); ++it)
        si_->freeState(it->second);
    closeRepresentatives.clear();

    // Then, begin searching the space around him
    for (unsigned int i = 0 ; i < nearSamplePoints_ ; ++i)
    {
        do
        {
            sampler_->sampleNear(workArea, qNew, denseDelta_);
        } while ((!si_->isValid(workArea) || si_->distance(qNew, workArea) > denseDelta_ || !si_->checkMotion(qNew, workArea)) && ptc == false);

        // if we were not successful at sampling a desirable state, we are out of time
        if (ptc == false)
            break;

        // Compute who his graph neighbors are
        Vertex representative = findGraphRepresentative(workArea);

        // Assuming this sample is actually seen by somebody (which he should be in all likelihood)
        if (representative != boost::graph_traits<Graph>::null_vertex())
        {
            //If his representative is different than qNew
            if (qRep != representative)
                //And we haven't already tracked this representative
                if (closeRepresentatives.find(representative) == closeRepresentatives.end())
                    //Track the representative
                    closeRepresentatives[representative] = si_->cloneState(workArea);
        }
        else
        {
            //This guy can't be seen by anybody, so we should take this opportunity to add him
            std::cout << " --   Adding node for COVERAGE" << std::endl;
            addGuard(si_->cloneState(workArea), COVERAGE);

            //We should also stop our efforts to add a dense path
            for (std::map<Vertex, base::State*>::iterator it = closeRepresentatives.begin(); it != closeRepresentatives.end(); ++it)
                si_->freeState(it->second);
            closeRepresentatives.clear();
            break;
        }
    }
}

void ompl::geometric::SPARStwo::updatePairPoints(Vertex rep, const base::State *q, Vertex r, const base::State *s)
{
    //First of all, we need to compute all candidate r'
    std::vector<Vertex> VPPs;
    computeVPP(rep, r, VPPs);

    //Then, for each pair Pv(r,r')
    foreach (Vertex rp, VPPs)
        //Try updating the pair info
        distanceCheck(rep, q, r, s, rp);
}

void ompl::geometric::SPARStwo::computeVPP(Vertex v, Vertex vp, std::vector<Vertex> &VPPs)
{
    VPPs.clear();
    foreach( Vertex cvpp, boost::adjacent_vertices( v, g_ ) )
        if( cvpp != vp )
            if( !boost::edge( cvpp, vp, g_ ).second )
                VPPs.push_back( cvpp );
}

void ompl::geometric::SPARStwo::computeX(Vertex v, Vertex vp, Vertex vpp, std::vector<Vertex> &Xs)
{
    Xs.clear();

    foreach (Vertex cx, boost::adjacent_vertices(vpp, g_))
        if (boost::edge(cx, v, g_).second && !boost::edge(cx, vp, g_).second)
        {
            InterfaceData& d = getData( v, vpp, cx );
            if ((vpp < cx && d.pointA_) || (cx < vpp && d.pointB_))
                Xs.push_back( cx );
        }
    Xs.push_back(vpp);
}

ompl::geometric::SPARStwo::VertexPair ompl::geometric::SPARStwo::index( Vertex vp, Vertex vpp )
{
    if( vp < vpp )
        return VertexPair( vp, vpp );
    else if( vpp < vp )
        return VertexPair( vpp, vp );
    else
        throw Exception( name_, "Trying to get an index where the pairs are the same point!");
}

ompl::geometric::SPARStwo::InterfaceData& ompl::geometric::SPARStwo::getData( Vertex v, Vertex vp, Vertex vpp )
{
    return interfaceDataProperty_[v].interfaceHash[index( vp, vpp )];
}

void ompl::geometric::SPARStwo::distanceCheck(Vertex rep, const base::State *q, Vertex r, const base::State *s, Vertex rp)
{
    //Get the info for the current representative-neighbors pair
    InterfaceData& d = getData( rep, r, rp );

    if (r < rp) // FIRST points represent r (the guy discovered through sampling)
    {
        if (d.pointA_ == NULL) // If the point we're considering replacing (P_v(r,.)) isn't there
            //Then we know we're doing better, so add it
            d.setFirst(q, s, si_);
        else //Otherwise, he is there,
        {
            if (d.pointB_ == NULL) //But if the other guy doesn't exist, we can't compare.
            {
                //Should probably keep the one that is further away from rep?  Not known what to do in this case.
                // TODO: is this not part of the algorithm?
            }
            else //We know both of these points exist, so we can check some distances
                if (si_->distance(q, d.pointB_) < si_->distance(d.pointA_, d.pointB_))
                    //Distance with the new point is good, so set it.
                    d.setFirst( q, s, si_ );
        }
    }
    else // SECOND points represent r (the guy discovered through sampling)
    {
        if (d.pointB_ == NULL) //If the point we're considering replacing (P_V(.,r)) isn't there...
            //Then we must be doing better, so add it
            d.setSecond(q, s, si_);
        else //Otherwise, he is there
        {
            if (d.pointA_ == NULL) //But if the other guy doesn't exist, we can't compare.
            {
                //Should we be doing something cool here?
            }
            else
                if (si_->distance(q, d.pointA_) < si_->distance(d.pointB_, d.pointA_))
                    //Distance with the new point is good, so set it
                    d.setSecond(q, s, si_);
        }
    }

    // Lastly, save what we have discovered
    interfaceDataProperty_[rep].interfaceHash[index(r, rp)] = d;
}

void ompl::geometric::SPARStwo::abandonLists(base::State *st)
{
    stateProperty_[ queryVertex_ ] = st;

    std::vector< Vertex > hold;
    nn_->nearestR( queryVertex_, sparseDelta_, hold );

    stateProperty_[queryVertex_] = NULL;

    //For each of the vertices
    foreach (Vertex v, hold)
    {
        foreach (VertexPair r, interfaceDataProperty_[v].interfaceHash | boost::adaptors::map_keys)
            interfaceDataProperty_[v].interfaceHash[r].clear(si_);
    }
}

ompl::geometric::SPARStwo::Vertex ompl::geometric::SPARStwo::addGuard(base::State *state, GuardType type)
{
    boost::mutex::scoped_lock _(graphMutex_);

    Vertex m = boost::add_vertex(g_);
    stateProperty_[m] = state;
    colorProperty_[m] = type;

    //assert(si_->isValid(state));
    abandonLists(state);

    disjointSets_.make_set(m);
    nn_->add(m);
    resetFailures();

    //std::cout << " -> addGuard() of type " << type << std::endl;
    visualizeCallback();
    return m;
}

void ompl::geometric::SPARStwo::connectGuards(Vertex v, Vertex vp)
{
    //std::cout << "connectGuards called ---------------------------------------------------------------- " << std::endl;
    assert(v <= milestoneCount());
    assert(vp <= milestoneCount());

    //std::cout << "Connecting vertex " << v << " to vertex " << vp <<  " ++++++++++++++++++" << std::endl;

    // Lock to prevent corruption
    boost::mutex::scoped_lock _(graphMutex_);

    // Create the new edge
    Edge e = (boost::add_edge(v, vp, g_)).first;

    // Add associated properties to the edge
    edgeWeightProperty_[e] = distanceFunction(v, vp); // TODO: use this value with astar
    edgeCollisionStateProperty_[e] = NOT_CHECKED;

    // Add the edge to the incrementeal connected components datastructure
    disjointSets_.union_set(v, vp);

    // Debug in Rviz
    visualizeCallback();
}

bool ompl::geometric::SPARStwo::convertVertexPathToStatePath(std::vector<Vertex> &vertexPath, 
                                                             const base::State* actualStart, 
                                                             const base::State* actualGoal,
                                                             CandidateSolution &candidateSolution)
{
    ompl::geometric::PathGeometric *pathGeometric = new ompl::geometric::PathGeometric(si_);
    candidateSolution.isApproximate_ = false; // assume path is valid
    
    // Add original start if it is different than the first state
    if (actualStart != stateProperty_[vertexPath.back()])
    {
        pathGeometric->append(actualStart);
        
        // Add the edge status
        // the edge from actualStart to start is always valid otherwise we would not have used that start
        candidateSolution.edgeCollisionStatus_.push_back(FREE);
    }

    // Reverse the vertexPath and convert to state path
    for (std::size_t i = vertexPath.size(); i > 0; --i)
    {
        pathGeometric->append(stateProperty_[vertexPath[i-1]]);

        // Add the edge status
        if (i > 1) // skip the last vertex (its reversed)
        {
            Edge thisEdge = boost::edge(vertexPath[i-1], vertexPath[i-2], g_).first;
            std::cout << "On edge  " << thisEdge << std::endl;
            

            // Check if any edges in path are not free (then it an approximate path)
            if (edgeCollisionStateProperty_[thisEdge] == IN_COLLISION)
            {    
                candidateSolution.isApproximate_ = true;
                candidateSolution.edgeCollisionStatus_.push_back(IN_COLLISION);
            }
            else if (edgeCollisionStateProperty_[thisEdge] == NOT_CHECKED)
            {
                OMPL_ERROR("A chosen path has an edge that has not been checked for collision. This should not happen");
                candidateSolution.edgeCollisionStatus_.push_back(NOT_CHECKED);
            }
            else 
            {
                candidateSolution.edgeCollisionStatus_.push_back(FREE);
            }
        }
    }

    // Add original goal if it is different than the last state
    if (actualGoal != stateProperty_[vertexPath.front()])
    {
        pathGeometric->append(actualGoal);

        // Add the edge status
        // the edge from actualGoal to goal is always valid otherwise we would not have used that goal
        candidateSolution.edgeCollisionStatus_.push_back(FREE);
    }

    candidateSolution.path_ = base::PathPtr(pathGeometric);

    return true;
}

void ompl::geometric::SPARStwo::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    // Explicitly add start and goal states:
    for (size_t i = 0; i < startM_.size(); ++i)
        data.addStartVertex(base::PlannerDataVertex(stateProperty_[startM_[i]], (int)START));

    for (size_t i = 0; i < goalM_.size(); ++i)
        data.addGoalVertex(base::PlannerDataVertex(stateProperty_[goalM_[i]], (int)GOAL));

    // I'm curious:
    if (goalM_.size() > 0)
    {
        throw Exception(name_, "SPARS2 has goal states?");
    }
    if (startM_.size() > 0)
    {
        throw Exception(name_, "SPARS2 has start states?");
    }

    // If there are even edges here
    if (boost::num_edges( g_ ) > 0)
    {
        // Adding edges and all other vertices simultaneously
        foreach (const Edge e, boost::edges(g_))
        {
            const Vertex v1 = boost::source(e, g_);
            const Vertex v2 = boost::target(e, g_);

            // TODO save weights!
            data.addEdge(base::PlannerDataVertex(stateProperty_[v1], (int)colorProperty_[v1]),
                         base::PlannerDataVertex(stateProperty_[v2], (int)colorProperty_[v2]));

            // Add the reverse edge, since we're constructing an undirected roadmap
            //data.addEdge(base::PlannerDataVertex(stateProperty_[v2], (int)colorProperty_[v2]),
            //             base::PlannerDataVertex(stateProperty_[v1], (int)colorProperty_[v1]));

            //OMPL_INFORM("Adding edge from vertex of type %d to vertex of type %d", colorProperty_[v1], colorProperty_[v2]);
        }
    }
    //else
    //    OMPL_INFORM("%s: There are no edges in the graph!", getName().c_str());

    // Make sure to add edge-less nodes as well
    foreach (const Vertex n, boost::vertices(g_))
        if (boost::out_degree(n, g_) == 0)
            data.addVertex(base::PlannerDataVertex(stateProperty_[n], (int)colorProperty_[n]));

    data.properties["iterations INTEGER"] = boost::lexical_cast<std::string>(iterations_);
}

void ompl::geometric::SPARStwo::setPlannerData(const base::PlannerData &data)
{
    // Check that the query vertex is initialized (used for internal nearest neighbor searches)
    checkQueryStateInitialization();

    // Add all vertices
    std::cout << std::endl;
    std::cout << "SPARS::setPlannerData: numVertices=" << data.numVertices() << std::endl;
    std::vector<Vertex> idToVertex;

    //std::cout << "Adding vertex \n";
    for (std::size_t vertexID = 0; vertexID < data.numVertices(); ++vertexID)
    {
        // Get the state from loaded planner data
        const base::State *oldState = data.getVertex(vertexID).getState();
        base::State *state = si_->cloneState(oldState);

        // Add the state to the graph and remember its ID
        //std::cout << "  " << vertexID << " of address " << state << " and tag " << data.getVertex(vertexID).getTag() << std::endl;

        // Get the tag, which in this application represents the vertex type
        GuardType type = static_cast<GuardType>( data.getVertex(vertexID).getTag() );

        idToVertex.push_back(addGuard(state, type )); // TODO: save the guard type
    }
    std::cout << std::endl;

    // Add the corresponding edges to the graph ------------------------------------

    std::vector<unsigned int> edgeList;
    //std::cout << "Adding edges " << std::endl;
    for (std::size_t fromVertex = 0; fromVertex < data.numVertices(); ++fromVertex)
    {
        edgeList.clear();
        
        // Get the edges
        data.getEdges(fromVertex, edgeList); // returns num of edges

        Vertex m = idToVertex[fromVertex];

        // Process edges
        for (std::size_t edgeId = 0; edgeId < edgeList.size(); ++edgeId)
        {
            std::size_t toVertex = edgeList[edgeId];
            Vertex n = idToVertex[toVertex];

            // Add the edge to the graph
            const base::Cost weight(0);
            if (false)
            {
                std::cout << "    Adding edge from vertex id " << fromVertex << " to id " <<  toVertex << " into edgeList" << std::endl;
                std::cout << "      Vertex " << m << " to " << n << std::endl;
            }
            connectGuards(m, n);
        }
    } // for

    if (false)
    {
        const char* name = "1234567890ABCDEFGHIJKLMNOPQRSTUVWXYZ";

        // Debug graph to terminal  
        std::cout << "edge set: ";
        print_edges(g_, name);

        std::cout << "out-edges:" << std::endl;
        print_graph(g_, name);
    } 
}

void ompl::geometric::SPARStwo::clearEdgeCollisionStates()
{
    foreach (const Edge e, boost::edges(g_))
        edgeCollisionStateProperty_[e] = NOT_CHECKED; // each edge has an unknown state
}
