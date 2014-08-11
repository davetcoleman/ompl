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

#ifndef OMPL_TOOLS_LIGHTNING_EXPERIENCEDB_
#define OMPL_TOOLS_LIGHTNING_EXPERIENCEDB_

#include "ompl/base/StateSpace.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerDataStorage.h"
#include "ompl/base/State.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/geometric/planners/prm/PRM.h"

namespace ompl
{

    namespace tools
    {
        /**
           @anchor ExperienceDB2
           @par Short description
           Database for storing and retrieving past plans
        */

        /// @cond IGNORE
        OMPL_CLASS_FORWARD(ExperienceDB2);
        /// @endcond

        /** \class ompl::geometric::ExperienceDB2Ptr
            \brief A boost shared pointer wrapper for ompl::tools::ExperienceDB2 */

        /** \brief Save and load entire paths from file */
        class ExperienceDB2
        {
        public:

            /** \brief Constructor needs the state space used for planning.
             *  \param space - state space
             */
            ExperienceDB2(const base::StateSpacePtr &space);

            /**
             * \brief Deconstructor
             */
            virtual ~ExperienceDB2(void);

            /**
             * \brief Load database from file
             * \param fileName - name of database file
             * \return true if file loaded successfully
             */
            bool load(const std::string& fileName);

            /**
             * \brief Add a new solution path to our database. Des not actually save to file so
             *        experience will be lost if save() is not called
             */
            void addPath(ompl::geometric::PathGeometric& solutionPath);

            /**
             * \brief Save loaded database to file, except skips saving if no paths have been added
             * \param fileName - name of database file
             * \return true if file saved successfully
             */
            bool saveIfChanged(const std::string& fileName);

            /**
             * \brief Save loaded database to file
             * \param fileName - name of database file
             * \return true if file saved successfully
             */
            bool save(const std::string& fileName);

            /**
             * \brief Get a vector of all the paths in the nearest neighbor tree
             */
            void getAllPaths(std::vector<ompl::base::PlannerDataPtr> &plannerDatas) const;

            /**
             * \brief Find the k nearest paths to our queries one
             */
            std::vector<ompl::base::PlannerDataPtr> findNearestStartGoal(int nearestK, const base::State* start, const base::State* goal);

            /**
             * \brief Print info to screen
             */
            void debugVertex(const ompl::base::PlannerDataVertex& vertex);
            void debugState(const ompl::base::State* state);

            /** \brief Get the total number of paths stored in the database */
            std::size_t getExperiencesCount() const;

            /** \brief Get number of unsaved paths */
            int getNumUnsavedPaths() const
            {
                return numUnsavedPaths_;
            }

        private:

            /** 
             * \brief Add the distance between both path's starts and the distance between both path's ends together
             */
            double distanceFunction(const ompl::base::PlannerDataPtr a, const ompl::base::PlannerDataPtr b) const;

        protected:

            /// The created space information
            base::SpaceInformationPtr     si_; // TODO: is this even necessary?

            /// Helper class for storing each plannerData instance
            ompl::base::PlannerDataStorage plannerDataStorage_;

            // A nearest-neighbors datastructure containing the tree of start/goal states combined
            //boost::shared_ptr< NearestNeighbors<ompl::base::PlannerDataPtr> > nn_;

            // Reusable plannerData instance for filling in start and goal and performing searches on the tree
            //ompl::base::PlannerDataPtr nnSearchKey_;

            // Track unsaved paths to determine if a save is required
            int numUnsavedPaths_;

            // Use PRM's graph datastructure to store experience
            boost::shared_ptr<ompl::geometric::PRM> prm_;

            /** \brief A secondary problem definition for the prm planner to use */
            ompl::base::ProblemDefinitionPtr                       prmProblemDef_;

        }; // end of class ExperienceDB2

    } // end of namespace

} // end of namespace
#endif
