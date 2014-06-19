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

#ifndef OMPL_GEOMETRIC_PLANNERS_EXPERIENCE_RETRIEVEREPAIR_
#define OMPL_GEOMETRIC_PLANNERS_EXPERIENCE_RETRIEVEREPAIR_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/geometric/PathSimplifier.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/tools/lightning/ExperienceDB.h"

namespace ompl
{

    namespace geometric
    {

        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::RetrieveRepair */
        OMPL_CLASS_FORWARD(RetrieveRepair);
        /// @endcond

        /** \class ompl::base::RetrieveRepairPtr
            \brief A boost shared pointer wrapper for ompl::base::RetrieveRepair */

        /**
           @anchor RetrieveRepair
           @par Short description
           RetrieveRepair is a experienced-based motion planner that recalls from a database of
           previous actions the most similar one to the current planning problem and attempts to repair it
           @par External documentation
           Berenson, Dmitry, Pieter Abbeel, and Ken Goldberg: A robot path planning framework that learns from experience, in <em>Robotics and Automation (ICRA), 2012 IEEE International Conference on. IEEE</em>, 2012.
           <a href="http://users.wpi.edu/~dberenson/lightning.pdf">[PDF]</a>
        */

        /** \brief The Lightning Framework's Retrieve-Repair component */
        class RetrieveRepair : public base::Planner
        {
        public:

            /** \brief Constructor */
            RetrieveRepair(const base::SpaceInformationPtr &si, ompl::tools::ExperienceDBPtr experienceDB);

            virtual ~RetrieveRepair(void);

            /** \brief Get information about the exploration data structure the planning from scratch motion planner used. */
            virtual void getPlannerData(base::PlannerData &data) const;

            /** \brief Get information about the recalled paths
             *  \param data - vector of PlannerData objects that each hold a single path
             *  \param chosenID - the index of the PlannerData object that was chosen for repair
             */
            void getRecalledPlannerDatas(std::vector<base::PlannerDataPtr> &data, std::size_t &chosenID) const;

            /**
             * \brief Get the chosen path used from database for repair
             * \return PlannerData of chosen path
             */
            ob::PlannerDataPtr getChosenRecallPath() const;

            /** \brief Get information about the exploration data structure the repair motion planner used each call. */
            void getRepairPlannerDatas(std::vector<base::PlannerDataPtr> &data) const;

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            virtual void clear(void);

            void setExperienceDB(ompl::tools::ExperienceDBPtr experienceDB);

            /** \brief Set the planner that will be used for repairing invalid paths recalled from experience */
            void setRepairPlanner(const base::PlannerPtr &planner);

            virtual void setup(void);

            /**
             * \brief Filters the top n paths in nearestPaths_ to the top 1, based on state validity with current environment
             * \return true if no error
             */
            bool findBestPath(const base::State *startState, const base::State *goalState, ob::PlannerDataPtr& chosenPath);

            /**
             * \brief Repairs a path to be valid in the current planning environment
             * \param oldPath - from experience
             * \return true if no error
             */
            bool repairPath(og::PathGeometric &path, const base::PlannerTerminationCondition &ptc);

            /**
             * \brief Use our secondary planner to find a valid path between start and goal, and return that path
             * \param start
             * \param goal
             * \param newPathSegment - the solution
             * \return true if path found
             */
            bool replan(const ob::State* start, const ob::State* goal, og::PathGeometric &newPathSegment,
                const base::PlannerTerminationCondition &ptc);

            /**
             * \brief Count the number of states along the discretized path that are in collision
             * // TODO: move this into DiscreteMotionValidator??
             */
            std::size_t checkMotionScore(const ob::State *s1, const ob::State *s2) const;

        protected:

            /** \brief Free the memory allocated by this planner */
            void freeMemory(void);

            /** \brief The database of motions to search through */
            ompl::tools::ExperienceDBPtr                   experienceDB_;

            /** \brief Recall the nearest paths and store this in planner data for introspection later */
            std::vector<ob::PlannerDataPtr>                nearestPaths_;

            /** \brief the ID within nearestPaths_ of the path that was chosen for repair */
            std::size_t                                    nearestPathsChosenID_;

            /** \brief A secondary planner for replanning */
            ob::PlannerPtr                                 repairPlanner_;

            /** \brief A secondary problem definition for the repair planner to use */
            ob::ProblemDefinitionPtr                       repairProblemDef_;

            /** \brief Debug the repair planner by saving its planner data each time it is used */
            std::vector<ob::PlannerDataPtr>                repairPlannerDatas_;

            /** \brief The instance of the path simplifier */
            og::PathSimplifierPtr                          psk_;
        };

    }
}

#endif
