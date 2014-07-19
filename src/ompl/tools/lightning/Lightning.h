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

#include "ompl/geometric/SimpleSetup.h"
#include "ompl/base/Planner.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/ProblemDefinition.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/ProblemDefinition.h"

#include "ompl/geometric/PathGeometric.h"
#include "ompl/geometric/PathSimplifier.h"
#include "ompl/geometric/planners/experience/RetrieveRepair.h"

#include "ompl/util/Console.h"
#include "ompl/util/Exception.h"

#include "ompl/tools/lightning/DynamicTimeWarp.h"

namespace ompl
{

    namespace tools
    {
        namespace og = ompl::geometric;
        namespace ob = ompl::base;
        namespace ot = ompl::tools;

        //class ExperienceDB; // forward declaration
        OMPL_CLASS_FORWARD(ExperienceDB);
        OMPL_CLASS_FORWARD(ParallelPlan);

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
            \brief A boost shared pointer wrapper for ompl::tools::Lightning */

        /** \brief Built off of SimpleSetup but provides support for planning from experience */
        class Lightning : public ompl::geometric::SimpleSetup
        {
        public:

            /** \brief Constructor needs the state space used for planning. */
            explicit
            Lightning(const base::SpaceInformationPtr &si);

            /** \brief Constructor needs the state space used for planning.
             *  \param space - the state space to plan in
             */
            explicit
            Lightning(const base::StateSpacePtr &space);

            /** \brief Load the experience database from file
             *  \param planningGroupName - used to name the database file, should be something like 'left_arm' or 'whole_body'
             *  \param databaseDirecotry - the directory to save the database to, relative to the user directory $HOME
             */
            bool load(const std::string &planningGroupName = "lightning_default_group", const std::string &databaseDirectory = "ompl_storage");

            /** \brief Display debug data about potential available solutions */
            void printResultsInfo(void) const;

            /**
             * \brief Get a pointer to the retrieve repair planner
             */
            og::RetrieveRepair& getRetrieveRepairPlanner()
            {
                if (!rrPlanner_)
                    throw Exception("RetrieveRepair planner not loaded yet");
                return static_cast<og::RetrieveRepair&>(*rrPlanner_);
            }

            /** \brief Set the planner to use for repairing experience paths
                inside the RetrieveRepair planner. If the planner is not
                set, a default planner is set. */
            void setRepairPlanner(const base::PlannerPtr &planner)
            {
                if (!rrPlanner_)
                    throw Exception("Retrieve repair planner has not been initialized yet");

                static_cast<og::RetrieveRepair&>(*rrPlanner_).setRepairPlanner(planner);
            }

            /** \brief Set the planner allocator to use. This is only
                used if no planner has been set. This is optional -- a default
                planner will be used if no planner is otherwise specified. */
            void setPlannerAllocator(const base::PlannerAllocator &pa)
            {
                pa_ = pa;
                planner_.reset();
                // note: the rrPlanner_ never uses the allocator so does not need to be reset
                configured_ = false;
            }

            /** \brief Run the planner for up to a specified amount of time (default is 1 second) */
            virtual base::PlannerStatus solve(double time = 1.0);

            /** \brief Run the planner until \e ptc becomes true (at most) */
            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            /** \brief Save the experience database to file */
            bool save();

            /** \brief Save the experience database to file if there has been a change */
            bool saveIfChanged();

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

            /**
             * \brief Convert the planning group name and database directory into a file path to open and save to
             */
            bool getFilePath(const std::string &planningGroupName, const std::string &databaseDirectory);

            /** \brief Optionally disable the ability to use previous plans in solutions (but will still save them) */
            void enableRecall(bool enable);

            /** \brief Optionally disable the ability to plan from scratch (but will still save them) */
            void enableScratch(bool enable);

            /** \brief Get a vector of all the paths in the database */
            void getAllPaths(std::vector<ob::PlannerDataPtr> &plannerDatas) const;

            /** \brief Get the total number of paths stored in the database */
            std::size_t getExperiencesCount() const;

            /**
             * \brief Convert PlannerData to PathGeometric. Assume ordering of verticies is order of path
             * \param PlannerData
             * \param PathGeometric
             */
            void convertPlannerData(const ob::PlannerDataPtr plannerData, og::PathGeometric &path);

            /** \brief Tool for comparing two paths and scoring them */
            ot::DynamicTimeWarpPtr getDynamicTimeWarp()
            {
                return dtw_;
            }

            /** \brief After load() is called, access the generated file path for loading and saving the experience database */
            const std::string& getFilePath() const
            {
                return filePath_;
            }

        protected:

            /// The maintained experience planner instance
            base::PlannerPtr              rrPlanner_;

            /// Flag indicating whether recalled plans should be used to find solutions. Enabled by default.
            bool                          recallEnabled_;

            /// Flag indicating whether planning from scratch should be used to find solutions. Enabled by default.
            bool                          scratchEnabled_;

            /** \brief Instance of parallel planning to use for computing solutions in parallel */
            ot::ParallelPlanPtr           pp_;

            /** \brief A shared object between all the planners for saving and loading previous experience */
            ot::ExperienceDBPtr           experienceDB_;

            /** \brief Tool for comparing two paths and scoring them */
            ot::DynamicTimeWarpPtr        dtw_;

            /** \brief File location of database */
            std::string                   filePath_;

        }; // end of class Lightning

    } // end of namespace

} // end of namespace
#endif
