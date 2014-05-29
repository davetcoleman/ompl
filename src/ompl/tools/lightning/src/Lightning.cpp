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
//#include "ompl/geometric/planners/rrt/RRTConnect.h"
#include "ompl/geometric/planners/rrt/RRT.h"
#include "ompl/geometric/planners/kpiece/LBKPIECE1.h" // TODO remove this planner
//#include "ompl/geometric/planners/kpiece/KPIECE1.h"

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
            eplanner_ = ob::PlannerPtr(new og::LBKPIECE1(si_)); // TODO: change this planner to a custom one
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

    // ZEBRA TODO
    lastStatus_ = pp_->solve(ptc, true);

    // Results
    planTime_ = time::seconds(time::now() - start);
    if (lastStatus_)
        OMPL_INFORM("Solution found in %f seconds", planTime_);
    else
        OMPL_INFORM("No solution found after %f seconds", planTime_);
    return lastStatus_;
}

ompl::base::PlannerStatus ompl::tools::Lightning::solve(double time)
{
    OMPL_ERROR("This version of solve for Lightning is not implemented yet");
    exit(2);

    setup();
    lastStatus_ = base::PlannerStatus::UNKNOWN;
    time::point start = time::now();
    lastStatus_ = planner_->solve(time);
    planTime_ = time::seconds(time::now() - start);
    if (lastStatus_)
        OMPL_INFORM("Solution found in %f seconds", planTime_);
    else
        OMPL_INFORM("No solution found after %f seconds", planTime_);
    return lastStatus_;
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
    if (pdef_)
    {
        const base::PathPtr &p = pdef_->getSolutionPath();
        if (p)
        {
            time::point start = time::now();
            if (duration < std::numeric_limits<double>::epsilon())
              psk_->simplifyMax(static_cast<og::PathGeometric&>(*p));
            else
              psk_->simplify(static_cast<og::PathGeometric&>(*p), duration);
            simplifyTime_ = time::seconds(time::now() - start);
            OMPL_INFORM("Path simplification took %f seconds", simplifyTime_);
            return;
        }
    }
    OMPL_WARN("No solution to simplify");
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
