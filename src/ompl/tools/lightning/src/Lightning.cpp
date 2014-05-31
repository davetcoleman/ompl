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
#include "ompl/geometric/SimpleSetup.h" // use their implementation of getDefaultPlanner
#include "ompl/base/StateSpace.h" // for storing to file

ompl::tools::Lightning::Lightning(const base::StateSpacePtr &space) :
    configured_(false), 
    planTime_(0.0), 
    simplifyTime_(0.0), 
    lastStatus_(base::PlannerStatus::UNKNOWN)
{
    si_.reset(new base::SpaceInformation(space));
    pdef_.reset(new base::ProblemDefinition(si_));
    psk_.reset(new og::PathSimplifier(si_));
    params_.include(si_->params());
}

void ompl::tools::Lightning::setup(void)
{
    if (!configured_ || !si_->isSetup() || !planner_->isSetup())
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
        if (!eplanner_)
        {
            eplanner_ = ob::PlannerPtr(new og::RetrieveRepair(si_)); // TODO: change this planner to a custom one
        }
        eplanner_->setProblemDefinition(pdef_);
        if (!eplanner_->isSetup())
            eplanner_->setup();

        // Setup parameters
        params_.clear();
        params_.include(si_->params());
        params_.include(planner_->params());
        configured_ = true;

        // Create the parallel component for splitting into two threads
        pp_ = ot::ParallelPlanPtr(new ot::ParallelPlan(pdef_) );
        pp_->addPlanner(planner_);   // Add the planning from scratch planner
        pp_->addPlanner(eplanner_);  // Add the planning from experience planner
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
    OMPL_INFORM("EXPERIENCED-BASED SOLVING starts now:");

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
        OMPL_INFORM("Solution found in %f seconds", planTime_);
    else
        OMPL_INFORM("No solution found after %f seconds", planTime_);

    // TESTING:
    // Save solution path to file

    // Get information about the exploration data structure the motion planner used. Used later in visualizing
    og::PathGeometric solution_path = getSolutionPath();

    OMPL_INFORM("Solution path is: ");
    getSolutionPath().print(std::cout);

    // Save the entire graph for testing
    ob::GraphStateStorage storage(si_->getStateSpace()); // TODO: don't use Graph, just regular?
    for (std::size_t i = 0; i < solution_path.getStates().size(); ++i)
    {
      storage.addState( solution_path.getStates()[i] );
    }

    storage.store(ompl::tools::OMPL_STORAGE_PATH.c_str());

    return lastStatus_;
}

ompl::base::PlannerStatus ompl::tools::Lightning::solve(double time)
{
    ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition( time );
    return solve(ptc);
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
    if (planner_)
        planner_->getPlannerData(pd);
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
