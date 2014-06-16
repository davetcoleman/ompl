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
#include "ompl/geometric/SimpleSetup.h" // use their implementation of getDefaultPlanner
#include "ompl/base/goals/GoalState.h"
#include "ompl/tools/config/SelfConfig.h"

#include <limits>
#include <boost/make_shared.hpp>

ompl::geometric::RetrieveRepair::RetrieveRepair(const base::SpaceInformationPtr &si, ompl::tools::ExperienceDBPtr experienceDB)
    : base::Planner(si, "RetrieveRepair"),
      experienceDB_(experienceDB)
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    // Repair Planner Specific:
    repairProblemDef_.reset(new base::ProblemDefinition(si_));
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
    if (repairPlanner_)
        repairPlanner_->clear();
}

void ompl::geometric::RetrieveRepair::setExperienceDB(ompl::tools::ExperienceDBPtr experienceDB)
{
    experienceDB_ = experienceDB;
}

void ompl::geometric::RetrieveRepair::setRepairPlanner(const base::PlannerPtr &planner)
{
    if (planner && planner->getSpaceInformation().get() != si_.get())
        throw Exception("Repair planner instance does not match space information");
    repairPlanner_ = planner;
    setup_ = false; // Not sure if this is necessary
}

void ompl::geometric::RetrieveRepair::setup(void)
{
    Planner::setup();

    // Setup repair planner (for use by the rrPlanner)
    // Note: does not use the same pdef as the main planner in this class
    if (!repairPlanner_)
    {
        OMPL_INFORM("No repairing planner specified. Using default.");
        repairPlanner_ = ompl::geometric::getDefaultPlanner(pdef_->getGoal()); // we could use the repairProblemDef_ here but that isn't setup yet
    }
    // Setup the problem definition for the repair planner
    repairProblemDef_->setOptimizationObjective(pdef_->getOptimizationObjective()); // copy primary problem def

    // Setup repair planner
    repairPlanner_->setProblemDefinition(repairProblemDef_);
    if (!repairPlanner_->isSetup())
        repairPlanner_->setup();


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

        // Check if the database is empty
        if (!experienceDB_->getExperiencesCount())
        {
            OMPL_INFORM("Experience database is empty so unable to run RetrieveRepair algorithm.");
            return base::PlannerStatus::TIMEOUT; // The planner failed to find a solution     
        }

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

        // Check if there are any solutions
        if (nearestPaths_.empty())
        {
            OMPL_INFORM("No similar path founds in nearest neighbor tree, unable to retrieve repair");
            return base::PlannerStatus::TIMEOUT; // The planner failed to find a solution
        }

        ob::PlannerDataPtr chosenPath;

        // Filter top n paths to 1
        if (!findBestPath(startState, goalState, chosenPath))
        {
            return base::PlannerStatus::CRASH;
        }

        // Convert chosen PlanningData experience to an actual path
        og::PathGeometricPtr primaryPath(new og::PathGeometric(si_));
        // Add goal
        primaryPath->append(goalState);
        // Add old states
        for (int i = chosenPath->numVertices() - 1 ; i >= 0 ; --i)
        {
            primaryPath->append(chosenPath->getVertex(i).getState());
        }
        // Add start
        primaryPath->append(startState);

        // Repair chosen path
        if (!repairPath(primaryPath))
        {
            return base::PlannerStatus::CRASH;
        }

        // Finished
        approxdif = 0; // ??
        pdef_->addSolutionPath(base::PathPtr(primaryPath), approximate, approxdif, getName());
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

bool ompl::geometric::RetrieveRepair::findBestPath(const base::State *startState, const base::State *goalState, ob::PlannerDataPtr& chosenPath)
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
            nearestPathsChosenID_ = path_id;
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

bool ompl::geometric::RetrieveRepair::repairPath(og::PathGeometricPtr primaryPath) // \todo is this the best way to pass around a path?
{
    // \todo: we could reuse our collision checking from the previous step to make this faster
    //        but that complicates everything and I'm not suppose to be spending too much time
    //        on this prototype - DTC

    OMPL_INFORM("Repairing path");

    // Error check
    if (primaryPath->getStateCount() < 2)
    {
        OMPL_ERROR("Cannot repair a path with less than 2 states");
        return false;
    }

    // Loop through every pair of states and make sure path is valid.
    // If not, replan between those states
    for (std::size_t to_id = 1; to_id < primaryPath->getStateCount(); ++to_id)
    {
        std::size_t from_id = to_id-1; // this is our last known valid state
        ob::State* from_state = primaryPath->getState(from_id);
        ob::State* to_state = primaryPath->getState(to_id);

        if (!si_->checkMotion(from_state, to_state))
        {
            // Path between (from, to) states not valid, but perhaps to STATE is
            // Search until next valid STATE is found in existing path
            std::size_t subsearch_id = to_id;
            ob::State* new_to;
            OMPL_DEBUG("Searching for next valid state, because state %d to %d was not valid out a total path length of %d.",
                from_id,to_id,primaryPath->getStateCount());
            while (subsearch_id < primaryPath->getStateCount())
            {
                OMPL_DEBUG("Checking state %d", subsearch_id);

                new_to = primaryPath->getState(subsearch_id);
                if (si_->isValid(new_to))
                {
                    OMPL_DEBUG("State %d was found to valid, we are done searching", subsearch_id);
                    // This future state is valid, we can stop searching
                    to_id = subsearch_id;
                    to_state = new_to;
                    break;
                }
                ++subsearch_id; // keep searching for a new state to plan to
            }
            // Check if we ever found a next state that is valid
            if (subsearch_id >= primaryPath->getStateCount())
            {
                // We never found a valid state to plan to, instead we reached the goal state and it too wasn't valid. This is bad.
                // I think this is a bug.
                OMPL_ERROR("No state was found valid in the remainder of the path. Invalid goal state. This should not happen.");
                return false;
            }

            // Plan between our two valid states
            PathGeometricPtr newPathSegment;

            // Not valid motion, replan
            OMPL_DEBUG("Planning from %d to %d", from_id, to_id);
            if (!replan(from_state, to_state, newPathSegment))
            {
                OMPL_ERROR("Unable to repair path between state %d and %d", from_id, to_id);
                return false;
            }

            // TODO make sure not approximate solution

            // Reference to the path
            std::vector<base::State*>& primaryPathStates = primaryPath->getStates();

            OMPL_DEBUG("Before deleting invalid state part:");
            primaryPath->print(std::cout);

            // Remove all invalid states between (from_id, to_id) - not including those states themselves
            while (from_id != to_id - 1)
            {
                OMPL_INFORM("Deleting state %d", from_id + 1);
                primaryPathStates.erase(primaryPathStates.begin() + from_id + 1);
                --to_id; // because vector has shrunk
                OMPL_INFORM("to_id is now %d", to_id);
            }

            OMPL_DEBUG("After deleting invalid state part:");
            primaryPath->print(std::cout);

            // Deep copy the states in the newPathSegement from the repair problem def
            // so that they are not unloaded when we repair a different segement

            // Insert new path segment into current path
            OMPL_DEBUG("Inserting new %d states into old path. Previous length: %d", newPathSegment->getStateCount(), primaryPathStates.size());

            // Note: skip first and last states because they should be same as our start and goal state, same as `from_id` and `to_id`
            for (std::size_t i = 1; i < newPathSegment->getStateCount() - 1; ++i)                
            {
                OMPL_DEBUG("Inserting newPathSegment state %d into old path at position %d", i, to_id + i);
                primaryPathStates.insert( primaryPathStates.begin() + to_id + i, si_->cloneState(newPathSegment->getStates()[i]) );                    
            }
            //primaryPathStates.insert( primaryPathStates.begin() + to_id, newPathSegment->getStates().begin(), newPathSegment->getStates().end() );
            OMPL_DEBUG("Inserted new states into old path. New length: %d", primaryPathStates.size());

            // Set the to_id to jump over the newly inserted states to the next unchecked state
            to_id = to_id + newPathSegment->getStateCount();
            OMPL_DEBUG("Continuing searching at state %d", to_id);
        }
    }

    OMPL_INFORM("Done repairing path");

    return true;
}

bool ompl::geometric::RetrieveRepair::replan(const ob::State* start, const ob::State* goal, PathGeometricPtr& newPathSegment)
{
    // Reset problem definition
    repairProblemDef_->clearSolutionPaths();
    repairProblemDef_->clearStartStates();
    repairProblemDef_->clearGoal();

    // Configure problem definition
    repairProblemDef_->setStartAndGoalStates(start, goal);

    // Solve
    OMPL_INFORM("Preparing to repair path-----------------------------------------");
    base::PlannerStatus lastStatus = base::PlannerStatus::UNKNOWN;
    time::point startTime = time::now();

    double seconds = 0.2; // TODO move this somewhere
    base::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition( seconds ); // TODO: this does not address pre-empting a planner
    lastStatus = repairPlanner_->solve(ptc);

    // Results
    double planTime = time::seconds(time::now() - startTime);
    if (!lastStatus)
    {
        OMPL_INFORM("Replan Solve: No solution found after %f seconds", planTime);
        return false;
    }

    // Convert solution into a PathGeometric path
    base::PathPtr p = repairProblemDef_->getSolutionPath();
    if (!p)
    {
        OMPL_ERROR("Unable to get solution path from problem definition");
        return false;
    }

    newPathSegment = boost::make_shared<og::PathGeometric>( static_cast<og::PathGeometric&>(*p) );
    
    // Save the planner data for debugging purposes
    repairPlannerDatas_.push_back(ob::PlannerDataPtr( new ob::PlannerData(si_) ));
    repairPlanner_->getPlannerData( *repairPlannerDatas_.back() );

    // Return success
    OMPL_INFORM("Replan Solve: solution found in %f seconds with %d states", planTime, newPathSegment->getStateCount() );

    return true;
}

void ompl::geometric::RetrieveRepair::getPlannerData(base::PlannerData &data) const
{
    OMPL_INFORM("RetrieveRepair getPlannerData: including %d similar paths", nearestPaths_.size());

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


void ompl::geometric::RetrieveRepair::getRecalledPlannerDatas(std::vector<base::PlannerDataPtr> &data, std::size_t &chosenID) const
{
    data = nearestPaths_; // list of candidate paths
    chosenID = nearestPathsChosenID_;
}

void ompl::geometric::RetrieveRepair::getRepairPlannerDatas(std::vector<base::PlannerDataPtr> &data) const
{
    data = repairPlannerDatas_;
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
