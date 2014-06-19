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

ompl::geometric::RetrieveRepair::RetrieveRepair(const base::SpaceInformationPtr &si, ompl::tools::ExperienceDBPtr experienceDB)
    : base::Planner(si, "RetrieveRepair"),
      experienceDB_(experienceDB)
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    // Repair Planner Specific:
    repairProblemDef_.reset(new base::ProblemDefinition(si_));

    psk_.reset(new og::PathSimplifier(si_));
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

    // Check if the database is empty
    if (!experienceDB_->getExperiencesCount())
    {
        OMPL_INFORM("Experience database is empty so unable to run RetrieveRepair algorithm.");
        return base::PlannerStatus::TIMEOUT; // The planner failed to find a solution
    }

    // Get a single start state TODO: more than one
    const base::State *startState = pis_.nextStart();

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
    int nearestK = 10;
    nearestPaths_ = experienceDB_->findNearestStartGoal(nearestK, startState, goalState);

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

    // All saved trajectories should be at least 2 states long
    assert(chosenPath->numVertices() >= 2);

    // Convert chosen PlanningData experience to an actual path
    og::PathGeometric *primaryPath = new PathGeometric(si_);
    // Add start
    primaryPath->append(startState);
    // Add old states
    for (std::size_t i = 0; i < chosenPath->numVertices(); ++i)
    {
        primaryPath->append(chosenPath->getVertex(i).getState());
    }
    // Add goal
    primaryPath->append(goalState);

    // All save trajectories should be at least 2 states long, and then we append the start and goal states
    assert(primaryPath->getStateCount() >= 4);

    // Repair chosen path
    if (!repairPath(*primaryPath, ptc))
    {
        return base::PlannerStatus::CRASH;
    }

    // Smooth the result
    OMPL_INFORM("RetrieveRepair solve: Simplifying solution (smoothing)...");
    time::point simplifyStart = time::now();
    std::size_t numStates = primaryPath->getStateCount();
    psk_->simplify(*primaryPath, ptc);
    double simplifyTime = time::seconds(time::now() - simplifyStart);
    OMPL_INFORM("Path simplification took %f seconds and removed %d states", simplifyTime, numStates - primaryPath->getStateCount());

    // Finished
    approxdif = 0;
    pdef_->addSolutionPath(base::PathPtr(primaryPath), approximate, approxdif, getName());
    solved = true;
    return base::PlannerStatus(solved, approximate);
}

bool ompl::geometric::RetrieveRepair::findBestPath(const base::State *startState, const base::State *goalState, ob::PlannerDataPtr& chosenPath)
{
    // TODO this implemention assumes the plan can go forward or backwards. I forgot the fancy word for that property
    std::cout << OMPL_CONSOLE_COLOR_CYAN << std::endl;
    OMPL_INFORM("Found %d similar paths. Filtering ---------------", nearestPaths_.size());

    // Filter down to just 1 chosen path
    ob::PlannerDataPtr bestPath = nearestPaths_.front();
    std::size_t bestPathScore = std::numeric_limits<std::size_t>::max();
    std::cout << "Best Path Score " << bestPathScore << std::endl;

    // Track which path has the shortest distance
    std::vector<double> distances(nearestPaths_.size(), 0);
    std::vector<bool> isReversed(nearestPaths_.size());

    assert(isReversed.size() == nearestPaths_.size());

    for (std::size_t pathID = 0; pathID < nearestPaths_.size(); ++pathID)
    {
        ob::PlannerDataPtr currentPath = nearestPaths_[pathID];

        // Error check
        if (currentPath->numVertices() < 2) // needs at least a start and a goal
        {
            OMPL_ERROR("A path was recalled that somehow has less than 2 vertices, which shouldn't happen");
            return false;
        }

        const ob::State* pathStartState = currentPath->getVertex(0).getState();
        const ob::State* pathGoalState = currentPath->getVertex(currentPath->numVertices()-1).getState();

        double regularDistance  = si_->distance(startState,pathStartState) + si_->distance(goalState,pathGoalState);
        double reversedDistance = si_->distance(startState,pathGoalState) + si_->distance(goalState,pathStartState);

        // Check if path is reversed from normal [start->goal] direction and cache the distance
        if ( regularDistance > reversedDistance )
        {
            // The distance between starts and goals is less when in reverse
            isReversed[pathID] = true;
            distances[pathID] = reversedDistance;
            // We won't actually flip it until later to save memory operations and not alter our NN tree in the ExperienceDB
        }
        else
        {
            isReversed[pathID] = false;
            distances[pathID] = regularDistance;
        }

        std::size_t pathScore = 0; // the score

        // Check the validity between our start location and the path's start
        // TODO: this might bias the score to be worse for the little connecting segment
        if (!isReversed[pathID])
            pathScore += checkMotionScore( startState, pathStartState );
        else
            pathScore += checkMotionScore( startState, pathGoalState );

        // Score current path for validity
        std::size_t invalidStates = 0;
        for (std::size_t vertex_id = 0; vertex_id < currentPath->numVertices(); ++vertex_id)
        {
            // Check if the sampled points are valid
            if( !si_->isValid( currentPath->getVertex(vertex_id).getState() ) )
            {
                invalidStates ++;
            }
        }
        // Track separate for debugging
        pathScore += invalidStates;

        // Check the validity between our goal location and the path's goal
        // TODO: this might bias the score to be worse for the little connecting segment
        if (!isReversed[pathID])
            pathScore += checkMotionScore( goalState, pathGoalState );
        else
            pathScore += checkMotionScore( goalState, pathStartState );

        // Factor in the distance between start/goal and our new start/goal
        OMPL_INFORM("Path %d | %d verticies | %d invalid | score %d | reversed: %s | distance: %f",
            int(pathID), currentPath->numVertices(), invalidStates, pathScore, isReversed[pathID] ? "true" : "false", distances[pathID]);

        // Check if we have a perfect score (0) and this is the shortest path (the first one)
        if (pathID == 0 && pathScore == 0)
        {
            OMPL_DEBUG(" --> The shortest path (path 0) has a perfect score (0), ending filtering early.");
            bestPathScore = pathScore;
            bestPath = currentPath;
            nearestPathsChosenID_ = pathID;            
            break; // end the for loop
        }

        // Check if this is the best score we've seen so far
        if (pathScore < bestPathScore)
        {
            OMPL_DEBUG(" --> This path is the best we've seen so far. Previous best: %d", bestPathScore);
            bestPathScore = pathScore;
            bestPath = currentPath;
            nearestPathsChosenID_ = pathID;
        }
        // if the best score is the same as a previous one we've seen,
        // choose the one that has the shortest connecting component
        else if (pathScore == bestPathScore && distances[nearestPathsChosenID_] > distances[pathID])
        {
            // This new path is a shorter distance
            OMPL_DEBUG(" --> This path is as good as the best we've seen so far, but its path is shorter. Previous best score: %d from index %d",
                bestPathScore, nearestPathsChosenID_);
            bestPathScore = pathScore;
            bestPath = currentPath;
            nearestPathsChosenID_ = pathID;
        }
        else
            OMPL_DEBUG(" --> Not best. Best score: %d from index %d", bestPathScore, nearestPathsChosenID_);
    }

    // Check if we have a solution
    if (!bestPath)
    {
        OMPL_ERROR("No best path found from k filtered paths");
        return false;
    }
    else if(!bestPath->numVertices() || bestPath->numVertices() == 1)
    {
        OMPL_ERROR("Only %d verticies found in PlannerData loaded from file. This is a bug.", bestPath->numVertices());
        return false;
    }

    // Reverse the path if necessary. We allocate memory for this so that we don't alter the database
    if (isReversed[nearestPathsChosenID_])
    {
        OMPL_DEBUG("Reversing planner data verticies count %d", bestPath->numVertices());
        ob::PlannerDataPtr newPath(new ob::PlannerData(si_));
        for (std::size_t i = bestPath->numVertices(); i > 0; --i) // size_t can't go negative so subtact 1 instead
        {
            //OMPL_INFORM("add vertex %d", i-1 );
            newPath->addVertex( bestPath->getVertex(i-1) );
        }
        // Set result
        chosenPath = newPath;
    }
    else
    {
        // Set result
        chosenPath = bestPath;
    }
    OMPL_DEBUG("Done Filtering --------------------------------------\n");
    std::cout << OMPL_CONSOLE_COLOR_RESET;

    return true;
}

bool ompl::geometric::RetrieveRepair::repairPath(og::PathGeometric &primaryPath, const base::PlannerTerminationCondition &ptc)
{
    // \todo: we could reuse our collision checking from the previous step to make this faster
    //        but that complicates everything and I'm not suppose to be spending too much time
    //        on this prototype - DTC

    std::cout << OMPL_CONSOLE_COLOR_BROWN << std::endl;
    OMPL_INFORM("Repairing path ----------------------------------");

    // Error check
    if (primaryPath.getStateCount() < 2)
    {
        OMPL_ERROR("Cannot repair a path with less than 2 states");
        std::cout << OMPL_CONSOLE_COLOR_RESET;
        return false;
    }

    OMPL_DEBUG("Initial primary path before modification (includes start and goal attached):");
    primaryPath.print(std::cout);

    // Loop through every pair of states and make sure path is valid.
    // If not, replan between those states
    for (std::size_t toID = 1; toID < primaryPath.getStateCount(); ++toID)
    {
        std::size_t fromID = toID-1; // this is our last known valid state
        ob::State* fromState = primaryPath.getState(fromID);
        ob::State* toState = primaryPath.getState(toID);

        // Check if our planner is out of time
        if (ptc == true)
        {
            OMPL_DEBUG("Repair path function interrupted because termination condition is true.");
            std::cout << OMPL_CONSOLE_COLOR_RESET;
            return false;
        }

        // Check path between states
        if (!si_->checkMotion(fromState, toState))
        {
            // Path between (from, to) states not valid, but perhaps to STATE is
            // Search until next valid STATE is found in existing path
            std::size_t subsearch_id = toID;
            ob::State* new_to;
            OMPL_DEBUG("Searching for next valid state, because state %d to %d was not valid out  %d total states",
                fromID,toID,primaryPath.getStateCount());
            while (subsearch_id < primaryPath.getStateCount())
            {
                new_to = primaryPath.getState(subsearch_id);
                if (si_->isValid(new_to))
                {
                    OMPL_DEBUG("State %d was found to valid, we can now repair between states", subsearch_id);
                    // This future state is valid, we can stop searching
                    toID = subsearch_id;
                    toState = new_to;
                    break;
                }
                ++subsearch_id; // keep searching for a new state to plan to
            }
            // Check if we ever found a next state that is valid
            if (subsearch_id >= primaryPath.getStateCount())
            {
                // We never found a valid state to plan to, instead we reached the goal state and it too wasn't valid. This is bad.
                // I think this is a bug.
                OMPL_ERROR("No state was found valid in the remainder of the path. Invalid goal state. This should not happen.");
                std::cout << OMPL_CONSOLE_COLOR_RESET;
                return false;
            }

            // Plan between our two valid states
            PathGeometric newPathSegment(si_);

            // Not valid motion, replan
            OMPL_DEBUG("Planning from %d to %d", fromID, toID);
            //si_->printState(fromState, std::cout);
            //si_->printState(toState, std::cout);
            if (!replan(fromState, toState, newPathSegment, ptc))
            {
                OMPL_DEBUG("Unable to repair path between state %d and %d", fromID, toID);
                std::cout << OMPL_CONSOLE_COLOR_RESET;
                return false;
            }

            // TODO make sure not approximate solution

            // Reference to the path
            std::vector<base::State*>& primaryPathStates = primaryPath.getStates();

            OMPL_DEBUG("Before deleting invalid state part:");
            primaryPath.print(std::cout);

            // Remove all invalid states between (fromID, toID) - not including those states themselves
            while (fromID != toID - 1)
            {
                OMPL_INFORM("Deleting state %d", fromID + 1);
                primaryPathStates.erase(primaryPathStates.begin() + fromID + 1);
                --toID; // because vector has shrunk
                OMPL_INFORM("toID is now %d", toID);
            }

            OMPL_DEBUG("After deleting invalid state part:");
            primaryPath.print(std::cout);

            // Insert new path segment into current path
            OMPL_DEBUG("Inserting new %d states into old path. Previous length: %d", newPathSegment.getStateCount()-2, primaryPathStates.size());

            // Note: skip first and last states because they should be same as our start and goal state, same as `fromID` and `toID`
            for (std::size_t i = 1; i < newPathSegment.getStateCount() - 1; ++i)
            {
                std::size_t insertLocation = toID + i - 1;
                OMPL_DEBUG("Inserting newPathSegment state %d into old path at position %d", i, insertLocation);
                primaryPathStates.insert( primaryPathStates.begin() + insertLocation, si_->cloneState(newPathSegment.getStates()[i]) );
            }
            //primaryPathStates.insert( primaryPathStates.begin() + toID, newPathSegment.getStates().begin(), newPathSegment.getStates().end() );
            OMPL_DEBUG("Inserted new states into old path. New length: %d", primaryPathStates.size());
            OMPL_DEBUG("After inserting states:");
            primaryPath.print(std::cout);

            // Set the toID to jump over the newly inserted states to the next unchecked state. Subtract 2 because we ignore start and goal
            toID = toID + newPathSegment.getStateCount() - 2;
            OMPL_DEBUG("Continuing searching at state %d", toID);
        }
    }

    OMPL_INFORM("Done repairing ---------------------------------");
    std::cout << OMPL_CONSOLE_COLOR_RESET;

    return true;
}

bool ompl::geometric::RetrieveRepair::replan(const ob::State* start, const ob::State* goal, PathGeometric &newPathSegment, 
    const base::PlannerTerminationCondition &ptc)
{
    // Reset problem definition
    repairProblemDef_->clearSolutionPaths();
    repairProblemDef_->clearStartStates();
    repairProblemDef_->clearGoal();

    // Reset planner
    repairPlanner_->clear();

    // Configure problem definition
    repairProblemDef_->setStartAndGoalStates(start, goal);

    // Configure planner
    repairPlanner_->setProblemDefinition(repairProblemDef_);

    // Solve
    OMPL_INFORM("Preparing to repair path-----------------------------------------");
    base::PlannerStatus lastStatus = base::PlannerStatus::UNKNOWN;
    time::point startTime = time::now();

    // TODO: if we use replanner like RRT* the ptc will allow it to run too long and no time will be left for the rest of algorithm
    lastStatus = repairPlanner_->solve(ptc);

    // Results
    double planTime = time::seconds(time::now() - startTime);
    if (!lastStatus)
    {
        OMPL_INFORM("Replan Solve: No solution found after %f seconds", planTime);
        return false;
    }

    // Check if approximate
    if (repairProblemDef_->hasApproximateSolution() || repairProblemDef_->getSolutionDifference() > std::numeric_limits<double>::epsilon())
    {
        OMPL_INFORM("Replan Solve: Solution is approximate, not using");
        return false;
    }

    // Convert solution into a PathGeometric path
    base::PathPtr p = repairProblemDef_->getSolutionPath();
    if (!p)
    {
        OMPL_ERROR("Unable to get solution path from problem definition");
        return false;
    }

    newPathSegment = static_cast<og::PathGeometric&>(*p);

    // Smooth the result
    OMPL_INFORM("Repair: Simplifying solution (smoothing)...");
    time::point simplifyStart = time::now();
    std::size_t numStates = newPathSegment.getStateCount();
    psk_->simplify(newPathSegment, ptc);
    double simplifyTime = time::seconds(time::now() - simplifyStart);
    OMPL_INFORM("Path simplification took %f seconds and removed %d states", simplifyTime, numStates - newPathSegment.getStateCount());

    // Save the planner data for debugging purposes
    repairPlannerDatas_.push_back(ob::PlannerDataPtr( new ob::PlannerData(si_) ));
    repairPlanner_->getPlannerData( *repairPlannerDatas_.back() );
    repairPlannerDatas_.back()->decoupleFromPlanner(); // copy states so that when planner unloads/clears we don't lose them

    // Return success
    OMPL_INFORM("Replan Solve: solution found in %f seconds with %d states", planTime, newPathSegment.getStateCount() );

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

ob::PlannerDataPtr ompl::geometric::RetrieveRepair::getChosenRecallPath() const
{
    return nearestPaths_[nearestPathsChosenID_];
}

void ompl::geometric::RetrieveRepair::getRepairPlannerDatas(std::vector<base::PlannerDataPtr> &data) const
{
    data = repairPlannerDatas_;
}

std::size_t ompl::geometric::RetrieveRepair::checkMotionScore(const ob::State *s1, const ob::State *s2) const
{
    int segmentCount = si_->getStateSpace()->validSegmentCount(s1, s2);
    //OMPL_INFORM("Checking motion between two states, segment count is: %d so we will check every %f distance",
    //    segmentCount, 1.0/double(segmentCount));

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
