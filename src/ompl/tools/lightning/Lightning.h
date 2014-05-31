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

/* Author: Dave Coleman
   Desc:   Implementation of the Lightning Framework for experienced-based planning

   Paper:  Berenson, Dmitry, Pieter Abbeel, and Ken Goldberg. 
           "A robot path planning framework that learns from experience." 
           Robotics and Automation (ICRA), 2012 IEEE International Conference on. IEEE, 2012.
*/

#ifndef OMPL_TOOLS_LIGHTNING_LIGHTNING_
#define OMPL_TOOLS_LIGHTNING_LIGHTNING_

#include "ompl/base/Planner.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/ProblemDefinition.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/StateStorage.h"
#include "ompl/base/ProblemDefinition.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/geometric/PathSimplifier.h"
#include "ompl/util/Console.h"
#include "ompl/util/Exception.h"
#include "ompl/tools/multiplan/ParallelPlan.h"

namespace og = ompl::geometric;
namespace ob = ompl::base;
namespace ot = ompl::tools;

namespace ompl
{

    namespace tools
    {
        /**
           @anchor Lightning
           @par Short description
           The Lightning Framework is a experienced-based motion planner that recalls from a databse of
           previous actions the most similar one to the current planning problem and attempts to repair it,
           while at the same time planning from scratch in a different thread
           @par External documentation
           Berenson, Dmitry, Pieter Abbeel, and Ken Goldberg: A robot path planning framework that learns from experience, in <em>Robotics and Automation (ICRA), 2012 IEEE International Conference on. IEEE</em>, 2012. 
           <a href="http://users.wpi.edu/~dberenson/lightning.pdf">[PDF]</a>
        */

        /// @cond IGNORE
        OMPL_CLASS_FORWARD(Lightning);
        /// @endcond

        /** \class ompl::geometric::LightningPtr
            \brief A boost shared pointer wrapper for ompl::geometric::Lightning */

        // TODO: move
        static const std::string OMPL_STORAGE_PATH = "/home/dave/ros/ompl_storage/file1";

        /** \brief Built off of SimpleSetup but provides support for planning from experience */
        class Lightning
        {
        public:

            /** \brief Constructor needs the state space used for planning. */
            explicit
            Lightning(const base::StateSpacePtr &space);

            virtual ~Lightning(void)
            {
            }

            /** \brief Get the current instance of the space information */
            const base::SpaceInformationPtr& getSpaceInformation(void) const
            {
                return si_;
            }

            /** \brief Get the current instance of the problem definition */
            const base::ProblemDefinitionPtr& getProblemDefinition(void) const
            {
                return pdef_;
            }

            /** \brief Get the current instance of the state space */
            const base::StateSpacePtr& getStateSpace(void) const
            {
                return si_->getStateSpace();
            }

            /** \brief Get the current instance of the state validity checker */
            const base::StateValidityCheckerPtr& getStateValidityChecker(void) const
            {
                return si_->getStateValidityChecker();
            }

            /** \brief Get the current goal definition */
            const base::GoalPtr& getGoal(void) const
            {
                return pdef_->getGoal();
            }

            /** \brief Get the current planner */
            const base::PlannerPtr& getPlanner(void) const
            {
                return planner_;
            }

            /** \brief Get the planner allocator */
            const base::PlannerAllocator& getPlannerAllocator(void) const
            {
                return pa_;
            }

            /** \brief Get the path simplifier */
            const og::PathSimplifierPtr& getPathSimplifier(void) const
            {
                return psk_;
            }

            /** \brief Get the path simplifier */
            og::PathSimplifierPtr& getPathSimplifier(void)
            {
                return psk_;
            }

            /** \brief Return true if a solution path is available (previous call to solve() was successful) and the solution is exact (not approximate) */
            bool haveExactSolutionPath(void) const;

            /** \brief Return true if a solution path is available (previous call to solve() was successful). The solution may be approximate. */
            bool haveSolutionPath(void) const
            {
                return pdef_->getSolutionPath().get();
            }

            /** \brief Get the solution path. Throw an exception if no solution is available */
            og::PathGeometric& getSolutionPath(void) const;

            /** \brief Get information about the exploration data structure the motion planner used. */
            void getPlannerData(base::PlannerData &pd) const;

            /** \brief Set the state validity checker to use */
            void setStateValidityChecker(const base::StateValidityCheckerPtr &svc)
            {
                si_->setStateValidityChecker(svc);
            }

            /** \brief Set the state validity checker to use */
            void setStateValidityChecker(const base::StateValidityCheckerFn &svc)
            {
                si_->setStateValidityChecker(svc);
            }

            /** \brief Set the state validity checker to use */
            void setOptimizationObjective(const base::OptimizationObjectivePtr &optimizationObjective)
            {
                pdef_->setOptimizationObjective(optimizationObjective);
            }

            /** \brief Set the start and goal states to use. */
            void setStartAndGoalStates(const base::ScopedState<> &start, const base::ScopedState<> &goal,
                                       const double threshold = std::numeric_limits<double>::epsilon())
            {
                pdef_->setStartAndGoalStates(start, goal, threshold);
            }

            /** \brief Add a starting state for planning. This call is not
                needed if setStartAndGoalStates() has been called. */
            void addStartState(const base::ScopedState<> &state)
            {
                pdef_->addStartState(state);
            }

            /** \brief Clear the currently set starting states */
            void clearStartStates(void)
            {
                pdef_->clearStartStates();
            }

            /** \brief Clear the currently set starting states and add \e state as the starting state */
            void setStartState(const base::ScopedState<> &state)
            {
                clearStartStates();
                addStartState(state);
            }

            /** \brief A simple form of setGoal(). The goal will be an instance of ompl::base::GoalState */
            void setGoalState(const base::ScopedState<> &goal, const double threshold = std::numeric_limits<double>::epsilon())
            {
                pdef_->setGoalState(goal, threshold);
            }

            /** \brief Set the goal for planning. This call is not
                needed if setStartAndGoalStates() has been called. */
            void setGoal(const base::GoalPtr &goal)
            {
                pdef_->setGoal(goal);
            }

            /** \brief Set the planner to use. If the planner is not
                set, an attempt is made to use the planner
                allocator. If no planner allocator is available
                either, a default planner is set. */
            void setPlanner(const base::PlannerPtr &planner)
            {
                if (planner && planner->getSpaceInformation().get() != si_.get())
                    throw Exception("Planner instance does not match space information");
                planner_ = planner;
                configured_ = false;
            }

            /** \brief Set the planner allocator to use. This is only
                used if no planner has been set. This is optional -- a default
                planner will be used if no planner is otherwise specified. */
            void setPlannerAllocator(const base::PlannerAllocator &pa)
            {
                pa_ = pa;
                planner_.reset();
                configured_ = false;
            }

            /** \brief Run the planner for up to a specified amount of time (default is 1 second) */
            virtual base::PlannerStatus solve(double time = 1.0);

            /** \brief Run the planner until \e ptc becomes true (at most) */
            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            /** \brief Return the status of the last planning attempt */
            base::PlannerStatus getLastPlannerStatus(void) const
            {
                return lastStatus_;
            }

            /** \brief Get the amount of time (in seconds) spent during the last planning step */
            double getLastPlanComputationTime(void) const
            {
                return planTime_;
            }

            /** \brief Get the amount of time (in seconds) spend during the last path simplification step */
            double getLastSimplificationTime(void) const
            {
                return simplifyTime_;
            }

            /** \brief Attempt to simplify the current solution path. Spent at most \e duration seconds in the simplification process.
                If \e duration is 0 (the default), a default simplification procedure is executed. */
            void simplifySolution(double duration = 0.0);

            /** \brief Attempt to simplify the current solution path. Stop computation when \e ptc becomes true at the latest. */
            void simplifySolution(const base::PlannerTerminationCondition &ptc);

            /** \brief Clear all planning data. This only includes
                data generated by motion plan computation. Planner
                settings, start & goal states are not affected. */
            virtual void clear(void);

            /** \brief Print information about the current setup */
            virtual void print(std::ostream &out = std::cout) const;

            /** \brief This method will create the necessary classes
                for planning. The solve() method will call this
                function automatically. */
            virtual void setup(void);

            /** \brief Get the  parameters for this planning context */
            base::ParamSet& params(void)
            {
                return params_;
            }

            /** \brief Get the  parameters for this planning context */
            const base::ParamSet& params(void) const
            {
                return params_;
            }

        protected:

            /// The created space information
            base::SpaceInformationPtr     si_;

            /// The created problem definition
            base::ProblemDefinitionPtr    pdef_;

            /// The maintained from-scratch planner instance
            base::PlannerPtr              planner_;

            /// The maintained experience planner instance
            base::PlannerPtr              eplanner_;

            /// The optional planner allocator
            base::PlannerAllocator        pa_;

            /// The instance of the path simplifier
            og::PathSimplifierPtr         psk_;

            /// Flag indicating whether the classes needed for planning are set up
            bool                          configured_;

            /// The amount of time the last planning step took
            double                        planTime_;

            /// The amount of time the last path simplification step took
            double                        simplifyTime_;

            /// The status of the last planning request
            base::PlannerStatus           lastStatus_;

            /// The parameters that describe the planning context
            base::ParamSet                params_;

            /** \brief Instance of parallel planning to use for computing solutions in parallel */
            ot::ParallelPlanPtr           pp_;
          
        }; // end of class Lightning

    } // end of namespace

} // end of namespace
#endif
