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

#ifndef OMPL_GEOMETRIC_PLANNERS_EXPERIENCE_RETRIEVEREPAIRPRM_
#define OMPL_GEOMETRIC_PLANNERS_EXPERIENCE_RETRIEVEREPAIRPRM_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/geometric/PathSimplifier.h"
#include "ompl/datastructures/NearestNeighbors.h"

namespace ompl
{
    namespace tools
    {
        OMPL_CLASS_FORWARD(ExperienceDB2);
    }

    namespace geometric
    {

        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::RetrieveRepairPRM */
        OMPL_CLASS_FORWARD(RetrieveRepairPRM);
        /// @endcond

        /** \class ompl::base::RetrieveRepairPRMPtr
            \brief A boost shared pointer wrapper for ompl::base::RetrieveRepairPRM */

        /**
           @anchor RetrieveRepairPRM
           @par Short description
           RetrieveRepairPRM is a experienced-based motion planner that recalls from a database of
           previous actions the most similar one to the current planning problem and attempts to repair it
           @par External documentation
           Berenson, Dmitry, Pieter Abbeel, and Ken Goldberg: A robot path planning framework that learns from experience, in <em>Robotics and Automation (ICRA), 2012 IEEE International Conference on. IEEE</em>, 2012.
           <a href="http://users.wpi.edu/~dberenson/lightning.pdf">[PDF]</a>
        */

        /** \brief The Thunder Framework's Retrieve-Repair component */
        class RetrieveRepairPRM : public base::Planner
        {
        public:

            /** \brief Constructor */
            RetrieveRepairPRM(const base::SpaceInformationPtr &si, const ompl::tools::ExperienceDB2Ptr &experienceDB);

            virtual ~RetrieveRepairPRM(void);

            /** \brief Get information about the exploration data structure the planning from scratch motion planner used. */
            virtual void getPlannerData(base::PlannerData &data) const;

            /** \brief Get information about the recalled paths
             *  \param data - vector of PlannerData objects that each hold a single path
             *  \param chosenID - the index of the PlannerData object that was chosen for repair
             */
            //void getRecalledPlannerDatas(std::vector<base::PlannerDataPtr> &data, std::size_t &chosenID) const;

            /**
             *  \brief Get debug information about the top recalled paths that were chosen for further filtering
             *  \return data - vector of PlannerData objects that each hold a single path
             */
            const std::vector<ompl::geometric::PathGeometric>& getLastRecalledNearestPaths() const;

            /**
             *  \brief Get debug information about the top recalled paths that were chosen for further filtering
             *  \return chosenID - the index of the PlannerData object that was chosen for repair
             */
            const std::size_t& getLastRecalledNearestPathChosen() const;

            /**
             * \brief Get the chosen path used from database for repair
             * \return PlannerData of chosen path
             */
            ompl::geometric::PathGeometric getChosenRecallPath() const;

            /** \brief Get information about the exploration data structure the repair motion planner used each call. */
            void getRepairPlannerDatas(std::vector<base::PlannerDataPtr> &data) const;

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            virtual void clear(void);

            /**
             * \brief Pass a pointer of the database from the thunder framework
             */
            void setExperienceDB(ompl::tools::ExperienceDB2Ptr experienceDB);

            /** \brief Set the planner that will be used for repairing invalid paths recalled from experience */
            void setRepairPlanner(const base::PlannerPtr &planner);

            virtual void setup(void);

            /**
             * \brief Repairs a path to be valid in the current planning environment
             * \param oldPath - from experience
             * \return true if no error
             */
            bool repairPath(ompl::geometric::PathGeometric &path, const base::PlannerTerminationCondition &ptc);

            /**
             * \brief Use our secondary planner to find a valid path between start and goal, and return that path
             * \param start
             * \param goal
             * \param newPathSegment - the solution
             * \return true if path found
             */
            bool replan(const ompl::base::State* start, const ompl::base::State* goal, ompl::geometric::PathGeometric &newPathSegment,
                        const base::PlannerTerminationCondition &ptc);
          
            /**
             * \brief Getter for number of 'k' close solutions to choose from database for further filtering 
             */ 
            int getNearestK()
            {
              return nearestK_;
            }

            /**
             * \brief Setter for number of 'k' close solutions to choose from database for further filtering
             */
            void setNearestK(int nearestK)
            {
              nearestK_ = nearestK;
            }

            void debugState(const ompl::base::State* state)
            {
                si_->printState(state, std::cout);
            }   
                  
        protected:

            /**
             * \brief Count the number of states along the discretized path that are in collision
             *        Note: This is kind of an ill-defined score though. It depends on the resolution of collision checking. 
             *        I am more inclined to try to compute the percent of the length of the motion that is valid. 
             *        That could go in SpaceInformation, as a utility function.
             */
            std::size_t checkMotionScore(const ompl::base::State *s1, const ompl::base::State *s2) const;

            /**
             * \brief Filters the top n paths in nearestPaths_ to the top 1, based on state validity with current environment
             * \return true if no error
             */
            bool findBestPath(const base::State *startState, const base::State *goalState, ompl::base::PlannerDataPtr& chosenPath);

            /** \brief Free the memory allocated by this planner */
            void freeMemory(void);

            /** \brief The database of motions to search through */
            ompl::tools::ExperienceDB2Ptr                          experienceDB_;

            /** \brief Recall the nearest paths and store this in planner data for introspection later */
            std::vector<ompl::geometric::PathGeometric>         nearestPaths_;

            /** \brief the ID within nearestPaths_ of the path that was chosen for repair */
            std::size_t                                            nearestPathsChosenID_;

            /** \brief A secondary planner for replanning */
            ompl::base::PlannerPtr                                 repairPlanner_;

            /** \brief A secondary problem definition for the repair planner to use */
            ompl::base::ProblemDefinitionPtr                       repairProblemDef_;

            /** \brief Debug the repair planner by saving its planner data each time it is used */
            std::vector<ompl::base::PlannerDataPtr>                repairPlannerDatas_;

            /** \brief The instance of the path simplifier */
            ompl::geometric::PathSimplifierPtr                     psk_;

            /** \brief Number of 'k' close solutions to choose from database for further filtering */
            int                                                    nearestK_;
        };

    }
}

#endif
