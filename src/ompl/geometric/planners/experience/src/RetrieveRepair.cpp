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

#include "ompl/geometric/planners/experience/RetrieveRepair.h"
//#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/goals/GoalState.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>

ompl::geometric::RetrieveRepair::RetrieveRepair(const base::SpaceInformationPtr &si, ompl::tools::ExperienceDBPtr experienceDB)
    : base::Planner(si, "RetrieveRepair"),
      experienceDB_(experienceDB)
{
    specs_.approximateSolutions = true;
    specs_.directed = true;
}

ompl::geometric::RetrieveRepair::~RetrieveRepair(void)
{
    freeMemory();
}

void ompl::geometric::RetrieveRepair::clear(void)
{
    Planner::clear();
    freeMemory();
}

void ompl::geometric::RetrieveRepair::setExperienceDB(ompl::tools::ExperienceDBPtr experienceDB)
{
    experienceDB_ = experienceDB;
}

void ompl::geometric::RetrieveRepair::setup(void)
{
    Planner::setup();
}

void ompl::geometric::RetrieveRepair::freeMemory(void)
{

}

ompl::base::PlannerStatus ompl::geometric::RetrieveRepair::solve(const base::PlannerTerminationCondition &ptc)
{
    bool solved = false;
    bool approximate = false;
    double approxdif = std::numeric_limits<double>::infinity();

    bool use_database = true;
    if (use_database)
    {
        OMPL_INFORM("Using database for RetrieveRepair planner.");

        // Get a single start state TODO: more than one
        const base::State *startState = pis_.nextStart();
        //si_->copyState(motion->state, st);

        // Get a single goal state TODO: more than one
        base::Goal *goal   = pdef_->getGoal().get();
        // Check that we have the correct type of goal
        if (!goal || !goal->hasType(ompl::base::GOAL_STATE))
        {
            OMPL_ERROR("Goal cannot be converted into a goal state");
            OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
            return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
        }
        const base::GoalState *goalStateClass = dynamic_cast<base::GoalState*>(goal);
        const base::State *goalState = goalStateClass->getState();

        // Search for previous solution in database
        int nearestK = 10; // TODO: make this 10 then check candidate solutions for amount of invalidness
        nearestPaths_ = experienceDB_->findNearestStartGoal(nearestK, startState, goalState);
        OMPL_INFORM("Found %d similar paths", nearestPaths_.size());

        // Filter down to just 1 chosen path
        // TODO       
        ob::PlannerDataPtr chosenPath;
        if (nearestPaths_.empty())
        {
            OMPL_WARN("No similar path founds in nearest neighbor tree, unable to retrieve repair");
            return base::PlannerStatus::TIMEOUT; // The planner failed to find a solution
        }
        else
        {
            chosenPath = nearestPaths_[0];
        }
       
        // Check if we have a solution
        if (!chosenPath->numVertices() || chosenPath->numVertices() == 1)
        {
            OMPL_ERROR("Only %d verticies found in PlannerData loaded from file. This is a bug.", chosenPath->numVertices());
            return base::PlannerStatus::CRASH;
        }
        else
        {
            // Create the solution path
            PathGeometric *path = new PathGeometric(si_);
            for (int i = chosenPath->numVertices() - 1 ; i >= 0 ; --i)
            {
                path->append(chosenPath->getVertex(i).getState());
            }
            approxdif = 0; // ??
            pdef_->addSolutionPath(base::PathPtr(path), approximate, approxdif, getName());
            solved = true;
        }
    }
    else
    {
        OMPL_INFORM("NOT using database for RetrieveRepair planner.");

        while (ptc == false)
        {
            // Dummy work
            usleep(0.1 * 1000000);
        }
        OMPL_INFORM("DONE doing dummy work in RetrieveRepair thread");
    }

    return base::PlannerStatus(solved, approximate);
}

void ompl::geometric::RetrieveRepair::getPlannerData(base::PlannerData &data) const
{
    OMPL_INFORM("RetrieveRepair getPlannerData: including %d similar paths", nearestPaths_.size());

    Planner::getPlannerData(data);

    // Visualize the n candidate paths that we recalled from the database
    for (std::size_t i = 0 ; i < nearestPaths_.size() ; ++i)
    {
        ob::PlannerDataPtr pd = nearestPaths_[i];
        for (std::size_t j = 1; j < pd->numVertices(); ++j)
        {
            data.addEdge(
                base::PlannerDataVertex(pd->getVertex(j-1).getState() ),
                base::PlannerDataVertex(pd->getVertex(j).getState()   ));
        }
    }
}
