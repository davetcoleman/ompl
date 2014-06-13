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

    // Repair Planner Specific:
    repairProbleDef_.reset(new base::ProblemDefinition(si)_);
}

ompl::geometric::RetrieveRepair::~RetrieveRepair(void)
{
    freeMemory();
}

void ompl::geometric::RetrieveRepair::clear(void)
{
    Planner::clear();
    freeMemory();

    // Clear the inner planner
    if (rePlanner_)
        rePlanner_->clear();    
}

void ompl::geometric::RetrieveRepair::setExperienceDB(ompl::tools::ExperienceDBPtr experienceDB)
{
    experienceDB_ = experienceDB;
}

void ompl::geometric::RetrieveRepair::setup(void)
{
    Planner::setup();

    // Create an inner, secondary planner for replanning
    rePlanner_ = new og::TRRT( si_ ); // \todo Make this planner be the same as the primary planning from scratch one. automatically
    rePlanner_->setProblemDefinition(pdef_);
    if (!rePlanner_->isSetup())
        rePlanner_->setup();    
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

        // Error check
        if (nearestPaths_.empty())
        {
            OMPL_WARN("No similar path founds in nearest neighbor tree, unable to retrieve repair");
            return base::PlannerStatus::TIMEOUT; // The planner failed to find a solution
        }

        ob::PlannerDataPtr chosenPath;

        // Filter top n paths to 1
        if (!findBestPath(chosenPath))
        {
            return base::PlannerStatus::CRASH;
        }

        // Convert chosen PlanningData experience to an actual path
        PathGeometricPtr path(new PathGeometric(si_));
        // Add start
        path->append(startState);
        // Add old states
        for (int i = chosenPath->numVertices() - 1 ; i >= 0 ; --i)
        {
            path->append(chosenPath->getVertex(i).getState());
        }
        // Add goal
        path->append(goalState);

        // Repair chosen path
        if (!repairPath(path))
        {
            return base::PlannerStatus::CRASH;
        }        


        approxdif = 0; // ??
        pdef_->addSolutionPath(base::PathPtr(path), approximate, approxdif, getName());
        solved = true;
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

bool ompl::geometric::RetrieveRepair::findBestPath(ob::PlannerDataPtr& chosenPath)
{
    // Filter down to just 1 chosen path
    ob::PlannerDataPtr bestPath = nearestPaths_.front();
    //std::size_t bestPathScore = std::numeric_limits<std::size_t>::infinity(); // we want to find the path with the minimal score
    std::size_t bestPathScore = 2147483647; // above is not working

    for (std::size_t path_id = 0; path_id < nearestPaths_.size(); ++path_id)
    {
        ob::PlannerDataPtr currentPath = nearestPaths_[path_id];
        std::size_t invalidCount = 0; // the score

        // Error check
        if (currentPath->numVertices() < 2) // needs at least a start and a goal
        {
            OMPL_ERROR("A path was recalled that somehow has less than 2 vertices, which shouldn't happen");
            return false;
        }

        // Check the validity between our start location and the path's start
        // TODO: this might bias the score to be worse for the little connecting segment
        invalidCount += checkMotionScore( startState, currentPath->getVertex(0).getState() );

        // Score current path for validity
        // Check the recalled path
        std::size_t invalidStates = 0;
        for (std::size_t vertex_id = 0; vertex_id < currentPath->numVertices(); ++vertex_id)
        {
            // Check if the sampled points are valid
            if( !si_->isValid( currentPath->getVertex(vertex_id).getState() ) )
            {
                invalidStates ++;
            }
        }
        // Track sperate for debugging
        invalidCount += invalidStates;

        // Check the validity between our goal location and the path's goal
        // TODO: this might bias the score to be worse for the little connecting segment
        invalidCount += checkMotionScore( goalState, currentPath->getVertex(currentPath->numVertices()-1).getState() );

        // Factor in the distance between start/goal and our new start/goal
        OMPL_INFORM("Path %d has %d total verticies, of which %d are invalid giving a total score of %d",
            int(path_id), currentPath->numVertices(), invalidStates, invalidCount );

        // Check if this is the best score we've seen so far
        if (invalidCount < bestPathScore)
        {
            OMPL_DEBUG("This path is the best we've seen so far. Previous best: %d", bestPathScore);
            bestPathScore = invalidCount;
            bestPath = currentPath;
        }
        else
            OMPL_DEBUG("Best score: %d", bestPathScore);
    }

    // Check if we have a solution
    if (!bestPath)
    {
        OMPL_ERROR("No best path found");
        return false;
    }
    else if(!bestPath->numVertices() || bestPath->numVertices() == 1)
    {
        OMPL_ERROR("Only %d verticies found in PlannerData loaded from file. This is a bug.", bestPath->numVertices());
        return false;
    }

    // Set result
    chosenPath = bestPath;
    return true;
}

bool ompl::geometric::RetrieveRepair::repairPath(PathGeometricPtr path) // \todo is this the best way to pass around a path?
{
    // \todo: we could reuse our collision checking from the previous step to make this faster
    //        but that complicates everything and I'm not suppose to be spending too much time
    //        on this prototype - DTC
    
    OMPL_INFORM("Repairing path");

    // Error check
    if (path->getStateCount() < 2)
    {
        OMPL_ERROR("Cannot repair a path with less than 2 states");
        return false;
    }

    // Loop through every pair of states and make sure path is valid. 
    // If not, replan between those states
    for (std::size_t i = 1; i < path->getStateCount(); ++i)
    {
        ob::State* from = path->getState(i-1);
        ob::State* to = path->getState(i);
        
        if (!si_->checkMotion(from, to))
        {
            PathGeometricPtr newPathSegment;

            // Not valid motion, replan
            if (!replan(from, to, newPathSegment))
            {
                OMPL_ERROR("Unable to repair path between state %d and %d", i-1, i);
                return false;
            }

            // Insert new path segment into current path
            // TODO
        }
    }

    return true;
}

bool ompl::geometric::RetrieveRepair::replan(ob::State* start, ob::State goal, PathGeometricPtr& newPathSegment)
{


    repairPlanner_->setProblemDefinition(pdef_);
    rePlanner_->

    return true;
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

std::size_t ompl::geometric::RetrieveRepair::checkMotionScore(const ob::State *s1, const ob::State *s2) const
{
    int segmentCount = si_->getStateSpace()->validSegmentCount(s1, s2);
    OMPL_INFORM("Checking motion between two states, segment count is: %d so we will check every %f distance",
        segmentCount, 1.0/double(segmentCount));

    std::size_t invalidStatesScore = 0; // count number of interpolated states in collision

    // temporary storage for the checked state
    ob::State *test = si_->allocState();

    // Linerarly step through motion between state 0 to state 1
    for (double location = 0.0; location <= 1.0; location += 1.0/double(segmentCount) )
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
