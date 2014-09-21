/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, University of Colorado, Boulder
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

/* Author: Dave Coleman
   Desc:   A pure virtual class with common functions for experience-based planning. Used by lighting and thunder
*/


#ifndef OMPL_TOOLS_EXPERIENCE__EXPERIENCE_SETUP_
#define OMPL_TOOLS_EXPERIENCE__EXPERIENCE_SETUP_

#include "ompl/geometric/SimpleSetup.h"

namespace ompl
{

    namespace tools
    {

        /// @cond IGNORE
        OMPL_CLASS_FORWARD(ExperienceSetup);
        /// @endcond

        /** \class ompl::geometric::ExperienceSetupPtr
            \brief A boost shared pointer wrapper for ompl::geometric::ExperienceSetup */

        /** \brief Create the set of classes typically needed to solve a
            geometric problem */
        class ExperienceSetup : public ompl::geometric::SimpleSetup
        {
        public:

            /** \brief Constructor needs the state space used for planning. */
            explicit
            ExperienceSetup(const base::SpaceInformationPtr &si)
                : ompl::geometric::SimpleSetup(si)
            {
            };

            /** \brief Constructor needs the state space used for planning. */
            explicit
            ExperienceSetup(const base::StateSpacePtr &space)
                : ompl::geometric::SimpleSetup(space)
            {
            };

            /** \brief Set the database file to load. Actual loading occurs when setup() is called
             *  \param databaseName - used to name the database file, should be something like 'left_arm' or 'whole_body'
             *  \param databaseDirecotry - the directory to save the database to, relative to the user directory $HOME
             */
            virtual bool setFile(const std::string &databaseName = "lightning_default_group", const std::string &databaseDirectory = "ompl_storage") = 0;

            /** \brief Display debug data about potential available solutions */
            virtual void printResultsInfo(std::ostream &out = std::cout) const = 0;

            /** \brief Display debug data about overall results from Lightning since being loaded */
            virtual void printLogs(std::ostream &out = std::cout) const = 0;

            /** \brief Save debug data about overall results from Lightning since being loaded */
            virtual void saveDataLog(std::ostream &out = std::cout) = 0;

            /** \brief Set the planner to use for repairing experience paths
                inside the RetrieveRepair planner. If the planner is not
                set, a default planner is set. */
            virtual void setRepairPlanner(const base::PlannerPtr &planner) = 0;

            /** \brief Save the experience database to file */
            virtual bool save() = 0;

            /** \brief Save the experience database to file if there has been a change */
            virtual bool saveIfChanged() = 0;

            /** \brief Optionally disable the ability to use previous plans in solutions (but will still save them) */
            void enablePlanningFromRecall(bool enable);

            /** \brief Optionally disable the ability to plan from scratch
             *         Note: Lightning can still save modified experiences if they are different enough
             */
            void enablePlanningFromScratch(bool enable);

            /** \brief Get a vector of all the planning data in the database */
            virtual void getAllPlannerDatas(std::vector<ompl::base::PlannerDataPtr> &plannerDatas) const = 0;

            /** \brief Get the total number of paths stored in the database */
            virtual std::size_t getExperiencesCount() const = 0;

            virtual bool getFilePath(const std::string &databaseName, const std::string &databaseDirectory);
    
            /** \brief After setFile() is called, access the generated file path for loading and saving the experience database */
            virtual const std::string& getFilePath() const;

        protected:

            /// Flag indicating whether recalled plans should be used to find solutions. Enabled by default.
            bool                              recallEnabled_;

            /// Flag indicating whether planning from scratch should be used to find solutions. Enabled by default.
            bool                              scratchEnabled_;

            /** \brief File location of database */
            std::string                       filePath_;

        };
    }

}
#endif
