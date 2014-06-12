/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK, The University of Tokyo.
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
 *   * Neither the name of the JSK, The University of Tokyo nor the names of its
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


#include "ompl/tools/lightning/Lightning.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/geometric/planners/experience/RetrieveRepair.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/geometric/SimpleSetup.h" // use their implementation of getDefaultPlanner
#include "ompl/base/StateSpace.h" // for storing to file

ompl::tools::Lightning::Lightning(const base::StateSpacePtr &space) :
    configured_(false),
    planTime_(0.0),
    simplifyTime_(0.0),
    lastStatus_(base::PlannerStatus::UNKNOWN),
    recallEnabled_(true)
{
    si_.reset(new base::SpaceInformation(space));
    pdef_.reset(new base::ProblemDefinition(si_));
    psk_.reset(new og::PathSimplifier(si_));
    params_.include(si_->params());

    // Load the experience database
    experienceDB_.reset(new ompl::tools::ExperienceDB(si_->getStateSpace()));
    experienceDB_->load(OMPL_STORAGE_PATH); // load from file
}

void ompl::tools::Lightning::setup(void)
{
    if (!configured_ || !si_->isSetup() || !planner_->isSetup() || !rrPlanner_->isSetup())
    {
        if (!si_->isSetup())
            si_->setup();

        // Setup planning from scratch planner
        if (!planner_)
        {
            if (pa_)
                planner_ = pa_(si_);
            if (!planner_)
            {
                OMPL_INFORM("No planner specified. Using default.");
                planner_ = ompl::geometric::getDefaultPlanner(getGoal());
            }
        }
        planner_->setProblemDefinition(pdef_);
        if (!planner_->isSetup())
            planner_->setup();

        // Setup planning from experience planner
        if (!rrPlanner_)
        {
            rrPlanner_ = ob::PlannerPtr(new og::RetrieveRepair(si_, experienceDB_)); // TODO: change this planner to a custom one
        }
        rrPlanner_->setProblemDefinition(pdef_);
        if (!rrPlanner_->isSetup())
            rrPlanner_->setup();

        // Setup parameters
        params_.clear();
        params_.include(si_->params());
        params_.include(planner_->params());
        configured_ = true;

        // Create the parallel component for splitting into two threads
        pp_ = ot::ParallelPlanPtr(new ot::ParallelPlan(pdef_) );
        pp_->addPlanner(planner_);   // Always add the planning from scratch planner
        if (recallEnabled_)
        {
            pp_->addPlanner(rrPlanner_);  // Add the planning from experience planner if desired
        }
    }
}

void ompl::tools::Lightning::clear(void)
{
    if (planner_)
        planner_->clear();
    if (pdef_)
        pdef_->clearSolutionPaths();
    if (pp_)
    {
        pp_->clearHybridizationPaths();
        pp_->clearPlanners();
    }
}

// we provide a duplicate implementation here to allow the planner to choose how the time is turned into a planner termination condition
ompl::base::PlannerStatus ompl::tools::Lightning::solve(const base::PlannerTerminationCondition &ptc)
{
    OMPL_INFORM("Lightning Framework: Starting solve()");

    // Setup again in case it has not been done yet
    setup();

    lastStatus_ = base::PlannerStatus::UNKNOWN;
    time::point start = time::now();

    // Start both threads
    bool hybridize = false;
    lastStatus_ = pp_->solve(ptc, hybridize);

    // Results
    planTime_ = time::seconds(time::now() - start);
    if (lastStatus_)
        OMPL_INFORM("Lightning Solve: solution found in %f seconds", planTime_);
    else
        OMPL_INFORM("Lightning Solve: No solution found after %f seconds", planTime_);

    // TESTING:
    // Save solution path to file

    // Get information about the exploration data structure the motion planner used. Used later in visualizing
    og::PathGeometric solution_path = getSolutionPath();

    OMPL_INFORM("Solution path has %d states and was generated from planner %s", solution_path.getStateCount(), getSolutionPlannerName().c_str());
    //solution_path.print(std::cout);

    // Save to database if the solution is not from the experience database
    if (getSolutionPlannerName() != rrPlanner_->getName())
    {
        OMPL_INFORM("Adding to database because best solution was not from database");
        experienceDB_->addPath(solution_path);
    }
    else
    {
        OMPL_INFORM("NOT saving to database because best solution was not from database");
    }

    return lastStatus_;
}

ompl::base::PlannerStatus ompl::tools::Lightning::solve(double time)
{
    ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition( time );
    return solve(ptc);
}

bool ompl::tools::Lightning::save()
{
    return experienceDB_->save(OMPL_STORAGE_PATH);
}

bool ompl::tools::Lightning::saveIfChanged()
{
    return experienceDB_->saveIfChanged(OMPL_STORAGE_PATH);
}

void ompl::tools::Lightning::simplifySolution(const base::PlannerTerminationCondition &ptc)
{
    if (pdef_)
    {
        const base::PathPtr &p = pdef_->getSolutionPath();
        if (p)
        {
            time::point start = time::now();
            psk_->simplify(static_cast<og::PathGeometric&>(*p), ptc);
            simplifyTime_ = time::seconds(time::now() - start);
            OMPL_INFORM("Path simplification took %f seconds", simplifyTime_);
            return;
        }
    }
    OMPL_WARN("No solution to simplify");
}

void ompl::tools::Lightning::simplifySolution(double duration)
{
    ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition( duration );
    simplifySolution(ptc);
}

const std::string ompl::tools::Lightning::getSolutionPlannerName(void) const
{
    if (pdef_)
    {
        ob::PathPtr path(new og::PathGeometric(si_)); // convert to a generic path ptr
        ob::PlannerSolution solution(path); // a dummy solution

        // Get our desired solution
        pdef_->getSolution(solution);
        return solution.plannerName_;
    }
    throw Exception("No problem definition found");
}

ompl::geometric::PathGeometric& ompl::tools::Lightning::getSolutionPath(void) const
{
    if (pdef_)
    {
        const base::PathPtr &p = pdef_->getSolutionPath();
        if (p)
            return static_cast<og::PathGeometric&>(*p);
    }
    throw Exception("No solution path");
}

bool ompl::tools::Lightning::haveExactSolutionPath(void) const
{
    return haveSolutionPath() && (!pdef_->hasApproximateSolution() || pdef_->getSolutionDifference() < std::numeric_limits<double>::epsilon());
}

void ompl::tools::Lightning::getPlannerData(base::PlannerData &pd) const
{
    pd.clear();
    /*
      if (planner_)
      planner_->getPlannerData(pd);
    */
    // Get the planner data from our experience-based planner
    if (rrPlanner_)
        rrPlanner_->getPlannerData(pd);
}

void ompl::tools::Lightning::print(std::ostream &out) const
{
    if (si_)
    {
        si_->printProperties(out);
        si_->printSettings(out);
    }
    if (planner_)
    {
        planner_->printProperties(out);
        planner_->printSettings(out);
    }
    if (pdef_)
        pdef_->print(out);
}

void ompl::tools::Lightning::enableRecall(bool enable)
{    
    // Remember state
    recallEnabled_ = enable;

    // Flag the planners as possibly misconfigured
    configured_ = false;
}
