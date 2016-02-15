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

#include <ompl/geometric/planners/experience/BoltRetrieveRepair.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/util/Console.h>
#include <ompl/tools/bolt/BoltDB.h>
#include "ompl/tools/config/MagicConstants.h"

#include <boost/thread.hpp>

#include <limits>

namespace ompl
{

namespace geometric
{

BoltRetrieveRepair::BoltRetrieveRepair(const base::SpaceInformationPtr &si, const tools::BoltDBPtr &experienceDB)
    : base::Planner(si, "Bolt_Retrieve_Repair")
    , experienceDB_(experienceDB)
    , smoothingEnabled_(false) // makes understanding recalled paths more difficult if enabled
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

void BoltRetrieveRepair::setExperienceDB(const tools::BoltDBPtr &experienceDB)
{
    experienceDB_ = experienceDB;
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
    if (experienceDB_->isEmpty())
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
    PRMdb::CandidateSolution candidateSolution;

    // Search for previous solution in database
    if (!experienceDB_->findNearestStartGoal(startState, goalState, candidateSolution, ptc))
    {
        OMPL_INFORM("RetrieveRepair::solve() No nearest start or goal found");
        return base::PlannerStatus::TIMEOUT; // The planner failed to find a solution
    }

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

} // namespace geometric
} // namespace ompl
