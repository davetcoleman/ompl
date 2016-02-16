/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, University of Colorado, Boulder
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

/* Author: Dave Coleman */

// OMPL
#include <ompl/geometric/planners/experience/BoltRetrieveRepair.h>
//#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/util/Console.h>
//#include "ompl/tools/config/MagicConstants.h"

// Boost
#include <boost/thread.hpp>
#include <boost/foreach.hpp>

// C++
#include <limits>

namespace ompl
{

namespace geometric
{

BoltRetrieveRepair::BoltRetrieveRepair(const base::SpaceInformationPtr &si, const BoltDBPtr &boltDB)
    : base::Planner(si, "Bolt_Retrieve_Repair")
    , boltDB_(boltDB)
    , smoothingEnabled_(false) // makes understanding recalled paths more difficult if enabled
    , sparseDelta_(2.0)
    , verbose_(true)
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    path_simplifier_.reset(new PathSimplifier(si_));
}

BoltRetrieveRepair::~BoltRetrieveRepair(void)
{
    freeMemory();
}

void BoltRetrieveRepair::clear(void)
{
    Planner::clear();
    freeMemory();
}

void BoltRetrieveRepair::setExperienceDB(const BoltDBPtr &boltDB)
{
    boltDB_ = boltDB;
}

void BoltRetrieveRepair::setup(void)
{
    Planner::setup();
}

void BoltRetrieveRepair::freeMemory(void)
{
}

base::PlannerStatus BoltRetrieveRepair::solve(const base::PlannerTerminationCondition &ptc)
{
    bool solved = false;
    double approxdif = std::numeric_limits<double>::infinity();

    // Check if the database is empty
    if (boltDB_->isEmpty())
    {
        OMPL_INFORM("Experience database is empty so unable to run BoltRetrieveRepair algorithm.");

        return base::PlannerStatus::ABORT;
    }

    // Restart the Planner Input States so that the first start and goal state can be fetched
    pis_.restart();

    // Get a single start and goal state TODO: more than one
    const base::State *startState = pis_.nextStart();
    const base::State *goalState = pis_.nextGoal(ptc);

    // Create solution path struct
    BoltDB::CandidateSolution candidateSolution;

    // Search for previous solution in database
    if (!getPathOffGraph(startState, goalState, candidateSolution, ptc))
    {
        OMPL_INFORM("RetrieveRepair::solve() No nearest start or goal found");
        return base::PlannerStatus::TIMEOUT; // The planner failed to find a solution
    }

    OMPL_INFORM("BoltDB::getPathOffGraph() returned true - found a solution of size %d",
                candidateSolution.getStateCount());

    // Save this for future debugging
    foundPath_.reset(new PathGeometric(candidateSolution.getGeometricPath()));

    // All save trajectories should be at least 2 states long, then we append the start and goal states, for min of 4
    assert(candidateSolution.getStateCount() >= 4);

    // Smooth the result
    if (smoothingEnabled_)
    {
      OMPL_INFORM("BoltRetrieveRepair solve: Simplifying solution (smoothing)...");
      time::point simplifyStart = time::now();
      std::size_t numStates = candidateSolution.getGeometricPath().getStateCount();
      path_simplifier_->simplify(candidateSolution.getGeometricPath(), ptc);
      double simplifyTime = time::seconds(time::now() - simplifyStart);
      OMPL_INFORM("BoltRetrieveRepair: Path simplification took %f seconds and removed %d states",
                  simplifyTime, numStates - candidateSolution.getGeometricPath().getStateCount());
    }

    // Finished
    approxdif = 0;
    bool approximate = candidateSolution.isApproximate_;

    pdef_->addSolutionPath(candidateSolution.path_, approximate, approxdif, getName());
    solved = true;
    return base::PlannerStatus(solved, approximate);
}

void BoltRetrieveRepair::getPlannerData(base::PlannerData &data) const
{
    OMPL_INFORM("BoltRetrieveRepair getPlannerData");

    for (std::size_t j = 1; j < foundPath_->getStateCount(); ++j)
    {
        data.addEdge(
            base::PlannerDataVertex(foundPath_->getState(j-1) ),
            base::PlannerDataVertex(foundPath_->getState(j)   ));
    }
}

const PathGeometric& BoltRetrieveRepair::getChosenRecallPath() const
{
    return *foundPath_;
}

std::size_t BoltRetrieveRepair::checkMotionScore(const base::State *s1, const base::State *s2) const
{
    int segmentCount = si_->getStateSpace()->validSegmentCount(s1, s2);

    std::size_t invalidStatesScore = 0; // count number of interpolated states in collision

    // temporary storage for the checked state
    base::State *test = si_->allocState();

    // Linerarly step through motion between state 0 to state 1
    double iteration_step = 1.0/double(segmentCount);
    for (double location = 0.0; location <= 1.0; location += iteration_step )
    {
        si_->getStateSpace()->interpolate(s1, s2, location, test);

        if (!si_->isValid(test))
        {
            //OMPL_DEBUG("Found INVALID location between states at gradient %f", location);
            invalidStatesScore ++;
        }
        else
        {
            //OMPL_DEBUG("Found valid location between states at gradient %f", location);
        }
    }
    si_->freeState(test);

    return invalidStatesScore;
}

bool BoltRetrieveRepair::getPathOffGraph(const base::State* start, const base::State* goal,
                                               BoltDB::CandidateSolution &candidateSolution,
                                               const base::PlannerTerminationCondition &ptc)
{
    // Get neighbors near start and goal. Note: potentially they are not *visible* - will test for this later

    // Start
    OMPL_INFORM("Looking for a node near the problem start");
    if (!findGraphNeighbors(start, startVertexCandidateNeighbors_))
    {
        OMPL_INFORM("No graph neighbors found for start within radius %f", sparseDelta_);
        return false;
    }
    if (verbose_)
        OMPL_INFORM("Found %d nodes near start", startVertexCandidateNeighbors_.size());

    // Goal
    OMPL_INFORM("Looking for a node near the problem goal");
    if (!findGraphNeighbors(goal, goalVertexCandidateNeighbors_))
    {
        OMPL_INFORM("No graph neighbors found for goal within radius %f", sparseDelta_);
        return false;
    }
    if (verbose_)
        OMPL_INFORM("Found %d nodes near goal", goalVertexCandidateNeighbors_.size());

    // Get paths between start and goal
    bool result = getPathOnGraph(startVertexCandidateNeighbors_, goalVertexCandidateNeighbors_,
                           start, goal, candidateSolution, ptc);

    // Error check
    if (!result)
    {
        OMPL_INFORM("getPathOffGraph(): BoltRetrieveRepair returned FALSE for getPathOnGraph");
        return false;
    }
    if (!candidateSolution.path_)
    {
        OMPL_ERROR("getPathOffGraph(): BoltRetrieveRepair returned solution is NULL");
        return false;
    }

    // Debug output
    if (false)
    {
        ompl::geometric::PathGeometric geometricSolution
            = static_cast<ompl::geometric::PathGeometric&>(*candidateSolution.path_);

        for (std::size_t i = 0; i < geometricSolution.getStateCount(); ++i)
        {
            OMPL_INFORM("  getPathOffGraph(): Adding state %f to plannerData", i );
            si_->printState(geometricSolution.getState(i), std::cout);
        }
    }

    return result;
}

bool BoltRetrieveRepair::getPathOnGraph(const std::vector<BoltDB::Vertex> &candidateStarts,
                                        const std::vector<BoltDB::Vertex> &candidateGoals,
                                        const base::State* actualStart,
                                        const base::State* actualGoal,
                                        BoltDB::CandidateSolution &candidateSolution,
                                        const base::PlannerTerminationCondition &ptc)
{
    // Try every combination of nearby start and goal pairs
    BOOST_FOREACH (BoltDB::Vertex start, candidateStarts)
    {
        // Check if this start is visible from the actual start
        if (!si_->checkMotion(actualStart, boltDB_->stateProperty_[start]))
        {
            if (verbose_)
                OMPL_WARN("FOUND CANDIDATE START THAT IS NOT VISIBLE ");
            continue; // this is actually not visible
        }

        BOOST_FOREACH (BoltDB::Vertex goal, candidateGoals)
        {
            if (verbose_)
                OMPL_INFORM("  foreach_goal: Checking motion from  %d to %d", actualGoal, boltDB_->stateProperty_[goal]);

            // Check if our planner is out of time
            if (ptc == true)
            {
                OMPL_DEBUG("getPathOnGraph function interrupted because termination condition is true.");
                return false;
            }

            // Check if this goal is visible from the actual goal
            if (!si_->checkMotion(actualGoal, boltDB_->stateProperty_[goal]))
            {
                if (verbose_)
                    OMPL_INFORM("FOUND CANDIDATE GOAL THAT IS NOT VISIBLE! ");
                continue; // this is actually not visible
            }

            // Repeatidly search through graph for connection then check for collisions then repeat
            if (lazyCollisionSearch( start, goal, actualStart, actualGoal, candidateSolution, ptc))
            {
                // Found a path
                return true;
            }
            else
            {
                // Did not find a path
                OMPL_INFORM("Did not find a path, looking for other start/goal combinations ");
            }

        } // foreach
    } // foreach

    return false;
}

bool BoltRetrieveRepair::lazyCollisionSearch(const BoltDB::Vertex &start,
                                                   const BoltDB::Vertex &goal,
                                                   const base::State* actualStart,
                                                   const base::State* actualGoal,
                                                   BoltDB::CandidateSolution &candidateSolution,
                                                   const base::PlannerTerminationCondition &ptc)
{
    // Vector to store candidate paths in before they are converted to PathPtrs
    std::vector<BoltDB::Vertex> vertexPath;

    // Make sure that the start and goal aren't so close together that they find the same vertex
    if (start == goal)
    {
        if (verbose_)
            OMPL_INFORM("    Start equals goal, skipping ");
        return false; // TODO(davetcoleman): should this be skipped, or maybe returned as the solution?
    }

    // Keep looking for paths between chosen start and goal until one is found that is valid,
    // or no further paths can be found between them because of disabled edges
    // this is necessary for lazy collision checking i.e. rerun after marking invalid edges we found
    while (true)
    {
        if (verbose_)
            OMPL_INFORM("      while true: look for valid paths between start and goal");

        // Check if our planner is out of time
        if (ptc == true)
        {
            OMPL_DEBUG("lazyCollisionSearch: function interrupted because termination condition is true.");
            return false;
        }

        // Attempt to find a solution from start to goal
        if (!boltDB_->astarSearch(start, goal, vertexPath))
        {
            // We will stop looking through this start-goal combination, but perhaps this partial solution is good
           if (verbose_)
                OMPL_INFORM("        unable to construct solution between start and goal using astar");

            // no path found what so ever
            return false;
        }

        if (verbose_)
        {
            OMPL_INFORM("        has at least a partial solution, maybe exact solution");
            OMPL_INFORM("        Solution has %d vertices", vertexPath.size());
        }

        // Check if all the points in the potential solution are valid
        if (lazyCollisionCheck(vertexPath, ptc))
        {
            if (verbose_)
            {
                OMPL_INFORM("---------- lazy collision check returned valid ");
            }

            // the path is valid, we are done!
            convertVertexPathToStatePath(vertexPath, actualStart, actualGoal, candidateSolution);
            return true;
        }
        // else, loop with updated graph that has the invalid edges/states disabled
    } // end while

    // we never found a valid path
    return false;
}

bool BoltRetrieveRepair::lazyCollisionCheck(std::vector<BoltDB::Vertex> &vertexPath,
                                                  const base::PlannerTerminationCondition &ptc)
{
    OMPL_DEBUG("Starting lazy collision checking");

    bool hasInvalidEdges = false;

    // Initialize
    BoltDB::Vertex fromVertex = vertexPath[0];
    BoltDB::Vertex toVertex;

    // Loop through every pair of states and make sure path is valid.
    for (std::size_t toID = 1; toID < vertexPath.size(); ++toID)
    {
        // Increment location on path
        toVertex = vertexPath[toID];

        // Check if our planner is out of time
        if (ptc)
        {
            OMPL_DEBUG("Lazy collision check function interrupted because termination condition is true.");
            return false;
        }

        BoltDB::Edge thisEdge = boost::edge(fromVertex, toVertex, boltDB_->g_).first;

        // Has this edge already been checked before?
        if (boltDB_->edgeCollisionStateProperty_[thisEdge] == BoltDB::NOT_CHECKED)
        {
            // Check path between states
            if (!si_->checkMotion(boltDB_->stateProperty_[fromVertex], boltDB_->stateProperty_[toVertex]))
            {
                // Path between (from, to) states not valid, disable the edge
                OMPL_INFORM("  DISABLING EDGE from vertex %f to vertex %f", fromVertex, toVertex);

                // Disable edge
                boltDB_->edgeCollisionStateProperty_[thisEdge] = BoltDB::IN_COLLISION;
            }
            else
            {
                // Mark edge as free so we no longer need to check for collision
                boltDB_->edgeCollisionStateProperty_[thisEdge] = BoltDB::FREE;
            }
        }

        // Check final result
        if (boltDB_->edgeCollisionStateProperty_[thisEdge] == BoltDB::IN_COLLISION)
        {
            // Remember that this path is no longer valid, but keep checking remainder of path edges
            hasInvalidEdges = true;
        }

        // switch vertex focus
        fromVertex = toVertex;
    }

    OMPL_INFORM("Done lazy collision checking");

    // Only return true if nothing was found invalid
    return !hasInvalidEdges;
}

bool BoltRetrieveRepair::findGraphNeighbors(const base::State *state, std::vector<BoltDB::Vertex> &graphNeighborhood)
{
    // Reset
    graphNeighborhood.clear();

    // Setup search by getting a non-const version of the focused state
    base::State* stateCopy = si_->cloneState(state);
    boltDB_->stateProperty_[ boltDB_->queryVertex_ ] = stateCopy;

    // Double the range of sparseDelta_ up to 3 times until at least 1 neighbor is found
    std::size_t expandNeighborhoodSearchAttempts = 3;
    double neighborSearchRadius;
    static const double EXPAND_NEIGHBORHOOD_RATE = 0.25; // speed to which we look outside the original sparse delta neighborhood

    // Begin search outward
    for (std::size_t i = 0; i < expandNeighborhoodSearchAttempts; ++i)
    {
        neighborSearchRadius = sparseDelta_ + i*EXPAND_NEIGHBORHOOD_RATE*sparseDelta_;
        if (verbose_)
        {
            OMPL_INFORM("-------------------------------------------------------");
            OMPL_INFORM("Attempt %d to find neighborhood at radius %f", i+1, neighborSearchRadius);
            OMPL_INFORM("-------------------------------------------------------");
        }

        boltDB_->nn_->nearestR( boltDB_->queryVertex_, neighborSearchRadius, graphNeighborhood);

        // Check if at least one neighbor found
        if (graphNeighborhood.size() > 0)
            break;
    }
    boltDB_->stateProperty_[ boltDB_->queryVertex_ ] = NULL;

    // Check if no neighbors found
    if (!graphNeighborhood.size())
    {
        return false;
    }
    return true;
}

bool BoltRetrieveRepair::convertVertexPathToStatePath(std::vector<BoltDB::Vertex> &vertexPath,
                                                            const base::State* actualStart,
                                                            const base::State* actualGoal,
                                                            BoltDB::CandidateSolution &candidateSolution,
                                                            bool disableCollisionWarning)
{
    // Ensure the input path is not empty
    if (!vertexPath.size())
        return false;

    ompl::geometric::PathGeometric *pathGeometric = new ompl::geometric::PathGeometric(si_);
    candidateSolution.isApproximate_ = false; // assume path is valid

    // Add original start if it is different than the first state
    if (actualStart != boltDB_->stateProperty_[vertexPath.back()])
    {
        pathGeometric->append(actualStart);

        // Add the edge status
        // the edge from actualStart to start is always valid otherwise we would not have used that start
        candidateSolution.edgeCollisionStatus_.push_back(BoltDB::FREE);
    }

    // Reverse the vertexPath and convert to state path
    for (std::size_t i = vertexPath.size(); i > 0; --i)
    {
        pathGeometric->append(boltDB_->stateProperty_[vertexPath[i-1]]);

        // Add the edge status
        if (i > 1) // skip the last vertex (its reversed)
        {
            BoltDB::Edge thisEdge = boost::edge(vertexPath[i-1], vertexPath[i-2], boltDB_->g_).first;

            // Check if any edges in path are not free (then it an approximate path)
            if (boltDB_->edgeCollisionStateProperty_[thisEdge] == BoltDB::IN_COLLISION)
            {
                candidateSolution.isApproximate_ = true;
                candidateSolution.edgeCollisionStatus_.push_back(BoltDB::IN_COLLISION);
            }
            else if (boltDB_->edgeCollisionStateProperty_[thisEdge] == BoltDB::NOT_CHECKED)
            {
                if (!disableCollisionWarning)
                    OMPL_ERROR("A chosen path has an edge that has not been checked for collision. This should not happen");
                candidateSolution.edgeCollisionStatus_.push_back(BoltDB::NOT_CHECKED);
            }
            else
            {
                candidateSolution.edgeCollisionStatus_.push_back(BoltDB::FREE);
            }
        }
    }

    // Add original goal if it is different than the last state
    if (actualGoal != boltDB_->stateProperty_[vertexPath.front()])
    {
        pathGeometric->append(actualGoal);

        // Add the edge status
        // the edge from actualGoal to goal is always valid otherwise we would not have used that goal
        candidateSolution.edgeCollisionStatus_.push_back(BoltDB::FREE);
    }

    candidateSolution.path_ = base::PathPtr(pathGeometric);

    return true;
}


} // namespace geometric
} // namespace ompl
