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
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>

ompl::geometric::RetrieveRepair::RetrieveRepair(const base::SpaceInformationPtr &si)
    : base::Planner(si, "RetrieveRepair"),
      storage_(si->getStateSpace()) // TODO: don't use Graph, just regular?
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    lastGoalMotion_ = NULL;

    // Load database from file, track loading time
    time::point start = time::now();
    OMPL_INFORM("Loading database from file: %s", OMPL_STORAGE_PATH.c_str());

    // Note: the StateStorage class checks if the states match for us
    storage_.load(OMPL_STORAGE_PATH.c_str());

    // Check how many states are stored
    OMPL_INFORM("Loaded %d states", storage_.size());

    double loadTime = time::seconds(time::now() - start);
    OMPL_INFORM("Loaded database from file in %f sec", loadTime);
}

ompl::geometric::RetrieveRepair::~RetrieveRepair(void)
{
    freeMemory();
}

void ompl::geometric::RetrieveRepair::clear(void)
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = NULL;
}

void ompl::geometric::RetrieveRepair::setup(void)
{
    Planner::setup();

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(si_->getStateSpace()));
    nn_->setDistanceFunction(boost::bind(&RetrieveRepair::distanceFunction, this, _1, _2));
}

void ompl::geometric::RetrieveRepair::freeMemory(void)
{
    if (nn_)
    {
        std::vector<Motion*> motions;
        nn_->list(motions);
        for (unsigned int i = 0 ; i < motions.size() ; ++i)
        {
            if (motions[i]->state)
                si_->freeState(motions[i]->state);
            delete motions[i];
        }
    }
}

ompl::base::PlannerStatus ompl::geometric::RetrieveRepair::solve(const base::PlannerTerminationCondition &ptc)
{
    /*
      checkValidity();
      base::Goal                 *goal   = pdef_->getGoal().get();
      //base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);

      while (const base::State *st = pis_.nextStart())
      {
      Motion *motion = new Motion(si_);
      si_->copyState(motion->state, st);
      nn_->add(motion);
      }

      if (nn_->size() == 0)
      {
      OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
      return base::PlannerStatus::INVALID_START;
      p  }

      if (!sampler_)
      sampler_ = si_->allocStateSampler();

      OMPL_INFORM("%s: Starting with %u states", getName().c_str(), nn_->size());

      Motion *solution  = NULL;
      Motion *approxsol = NULL;
      double  approxdif = std::numeric_limits<double>::infinity();
      Motion *rmotion   = new Motion(si_);
      base::State *rstate = rmotion->state;
      base::State *xstate = si_->allocState();

      while (ptc == false)
      {


      // find closest state in the tree
      Motion *nmotion = nn_->nearest(rmotion);
      base::State *dstate = rstate;

      // find state to add
      //double d = si_->distance(nmotion->state, rstate);

      if (si_->checkMotion(nmotion->state, dstate))
      {
      // create a motion
      Motion *motion = new Motion(si_);
      si_->copyState(motion->state, dstate);
      motion->parent = nmotion;

      nn_->add(motion);
      double dist = 0.0;
      bool sat = goal->isSatisfied(motion->state, &dist);
      if (sat)
      {
      approxdif = dist;
      solution = motion;
      break;
      }
      if (dist < approxdif)
      {
      approxdif = dist;
      approxsol = motion;
      }
      }
      }

      bool solved = false;
      bool approximate = false;
      if (solution == NULL)
      {
      solution = approxsol;
      approximate = true;
      }

      if (solution != NULL)
      {
      lastGoalMotion_ = solution;

      // construct the solution path
      std::vector<Motion*> mpath;
      while (solution != NULL)
      {
      mpath.push_back(solution);
      solution = solution->parent;
      }

      // set the solution path
      PathGeometric *path = new PathGeometric(si_);
      for (int i = mpath.size() - 1 ; i >= 0 ; --i)
      path->append(mpath[i]->state);
      pdef_->addSolutionPath(base::PathPtr(path), approximate, approxdif);
      solved = true;
      }

      si_->freeState(xstate);
      if (rmotion->state)
      si_->freeState(rmotion->state);
      delete rmotion;

      OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

      return base::PlannerStatus(solved, approximate);
    */

    bool solved = false;
    bool approximate = false;
    double  approxdif = std::numeric_limits<double>::infinity();

    bool use_database = true;
    if (use_database)
    {
        // Search for previous solution in database
        OMPL_INFORM("Getting states:");
        std::vector<const ompl::base::State*> states = storage_.getStates();
        for (std::size_t i = 0; i < states.size(); ++i)
        {
            si_->printState(states[i], std::cout);
        }

        // Check if we have a solution
        if (!states.empty())
        {
            // Create the solution path
            PathGeometric *path = new PathGeometric(si_);
            for (int i = states.size() - 1 ; i >= 0 ; --i)
            {
                path->append(states[i]);
            }
            approxdif = 0; // ??
            pdef_->addSolutionPath(base::PathPtr(path), approximate, approxdif);
            solved = true;
        }
    }
    else
    {
        while (ptc == false)
        {
            // Dummy work
            usleep(0.1 * 1000000);
        }
        OMPL_INFORM("DONE doing dummy work in RetrieveRepair thread");
    }

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return base::PlannerStatus(solved, approximate);
}

void ompl::geometric::RetrieveRepair::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion*> motions;
    if (nn_)
        nn_->list(motions);

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (unsigned int i = 0 ; i < motions.size() ; ++i)
    {
        if (motions[i]->parent == NULL)
            data.addStartVertex(base::PlannerDataVertex(motions[i]->state));
        else
            data.addEdge(base::PlannerDataVertex(motions[i]->parent->state),
                base::PlannerDataVertex(motions[i]->state));
    }
}
