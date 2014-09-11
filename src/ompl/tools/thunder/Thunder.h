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
   Desc:   Implementation of the Thunder Framework for experienced-based planning

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
#include "ompl/geometric/planners/experience/RetrieveRepairPRM.h"

#include "ompl/util/Console.h"
#include "ompl/util/Exception.h"

namespace og = ompl::geometric;
namespace ob = ompl::base;

namespace ompl
{

    namespace tools
    {
        //class ExperienceDB2; // forward declaration
        OMPL_CLASS_FORWARD(ExperienceDB2);
        OMPL_CLASS_FORWARD(ParallelPlan);

        /**
           @anchor Thunder
           @par Short description
           The Thunder Framework is a experienced-based motion planner that recalls from a databse of
           previous actions the most similar one to the current planning problem and attempts to repair it,
           while at the same time planning from scratch in a different thread
           @par External documentation
           Berenson, Dmitry, Pieter Abbeel, and Ken Goldberg: A robot path planning framework that learns from experience,
           in <em>Robotics and Automation (ICRA), 2012 IEEE International Conference on. IEEE</em>, 2012.
           DOI: <a href="http://dx.doi.org/10.1109/ICRA.2012.6224742">10.1109/ICRA.2012.6224742</a><br>
           <a href="http://users.wpi.edu/~dberenson/lightning.pdf">[PDF]</a>
        */

        /// @cond IGNORE
        OMPL_CLASS_FORWARD(Thunder);
        /// @endcond

        /** \class ompl::geometric::ThunderPtr
            \brief A boost shared pointer wrapper for ompl::tools::Thunder */

        /** \brief Built off of SimpleSetup but provides support for planning from experience */
        class Thunder : public ompl::geometric::SimpleSetup
        {
        public:
            /**
             * \brief Simple logging functionality encapsled in a struct
             */
            struct ThunderLogs
            {
                ThunderLogs()
                    : numSolutionsFromRecall_(0)
                    , numSolutionsFromRecallSaved_(0)
                    , numSolutionsFromScratch_(0)
                    , numSolutionsFailed_(0)
                    , numSolutionsApproximate_(0)
                    , numSolutionsTooShort_(0)
                    , numProblems_(0)
                    , totalPlanningTime_(0)
                {
                    // Header of CSV file
                    csvDataLogStream_ << "time,states,planner,result,is_saved,score,total_scratch,total_recall,total_recall_saved,"
                                      << "total_recall_discarded,total_failed,total_approximate,total_too_short,total_experiences,"
                                      << "avg_planning_time" << std::endl;
                }

                double getAveragePlanningTime() const
                {
                    return totalPlanningTime_ / numProblems_;
                }

                double numSolutionsFromRecall_;
                double numSolutionsFromRecallSaved_;
                double numSolutionsFromScratch_;
                double numSolutionsFailed_;
                double numSolutionsApproximate_;
                double numSolutionsTooShort_; // less than 3 states
                double numProblems_; // input requests
                double totalPlanningTime_; // of all input requests, used for averaging
                std::stringstream csvDataLogStream_; // output data to file to analyze performance externally
            };

            /** \brief Constructor needs the state space used for planning. */
            explicit
            Thunder(const base::SpaceInformationPtr &si);

            /** \brief Constructor needs the state space used for planning.
             *  \param space - the state space to plan in
             */
            explicit
            Thunder(const base::StateSpacePtr &space);

        private:

            /**
             * \brief Shared constructor functions
             */
            void initialize();

        public:
            /** \brief Load the experience database from file
             *  \param databaseName - used to name the database file, should be something like 'left_arm' or 'whole_body'
             *  \param databaseDirecotry - the directory to save the database to, relative to the user directory $HOME
             */
            bool load(const std::string &databaseName = "lightning_default_group", const std::string &databaseDirectory = "ompl_storage");

            /** \brief Display debug data about potential available solutions */
            void printResultsInfo(std::ostream &out = std::cout) const;

            /** \brief Display debug data about overall results from Thunder since being loaded */
            void printLogs(std::ostream &out = std::cout) const;

            /** \brief Save debug data about overall results from Thunder since being loaded */
            void saveDataLog(std::ostream &out = std::cout);

            /** \brief Get the current planner */
            ompl::base::PlannerPtr& getPlanner()
            {
                return planner_;
            }

            /**
             * \brief Get a pointer to the retrieve repair planner
             */
            ompl::geometric::RetrieveRepairPRM& getRetrieveRepairPlanner() const
            {
                return static_cast<ompl::geometric::RetrieveRepairPRM&>(*rrPlanner_);
            }

            /** \brief Set the planner to use for repairing experience paths
                inside the RetrieveRepairPRM planner. If the planner is not
                set, a default planner is set. */
            void setRepairPlanner(const base::PlannerPtr &planner)
            {
                static_cast<og::RetrieveRepairPRM&>(*rrPlanner_).setRepairPlanner(planner);
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

            /** \brief Optionally disable the ability to use previous plans in solutions (but will still save them) */
            void enablePlanningFromRecall(bool enable);

            /** \brief Optionally disable the ability to plan from scratch
             *         Note: Thunder can still save modified experiences if they are different enough
             */
            void enablePlanningFromScratch(bool enable);

            /** \brief Get a vector of all the paths in the database */
            void getAllPlannerDatas(std::vector<ompl::base::PlannerDataPtr> &plannerDatas) const;

            /** \brief Get the total number of paths stored in the database */
            std::size_t getExperiencesCount() const;

            /**
             * \brief Convert PlannerData to PathGeometric. Assume ordering of verticies is order of path
             * \param PlannerData
             * \param PathGeometric
             */
            void convertPlannerData(const ompl::base::PlannerDataPtr plannerData, ompl::geometric::PathGeometric &path);

            /** \brief After load() is called, access the generated file path for loading and saving the experience database */
            const std::string& getFilePath() const
            {
                return filePath_;
            }

          /**
           * \brief If path1 and path2 have a better start/goal match when reverse, then reverse path2
           * \param path to test against
           * \param path to reverse
           * \return true if reverse was necessary
           */
          bool reversePathIfNecessary(ompl::geometric::PathGeometric &path1, ompl::geometric::PathGeometric &path2);

          /**
           * \brief Getter for logging data
           */
          const ThunderLogs& getLogs() const
          {
              return logs_;
          }

          // TEMP
          ompl::tools::ExperienceDB2Ptr getExperienceDB();

        protected:

            /**
             * \brief Convert the planning group name and database directory into a file path to open and save to
             */
            bool getFilePath(const std::string &databaseName, const std::string &databaseDirectory);

            /// The maintained experience planner instance
            base::PlannerPtr                  rrPlanner_;

            /// Flag indicating whether recalled plans should be used to find solutions. Enabled by default.
            bool                              recallEnabled_;

            /// Flag indicating whether planning from scratch should be used to find solutions. Enabled by default.
            bool                              scratchEnabled_;

            /** \brief Instance of parallel planning to use for computing solutions in parallel */
            ompl::tools::ParallelPlanPtr      pp_;

            /** \brief A shared object between all the planners for saving and loading previous experience */
            ompl::tools::ExperienceDB2Ptr      experienceDB_;

            /** \brief File location of database */
            std::string                       filePath_;

            /** \brief Logging data for debugging  */
            ThunderLogs                     logs_;

        }; // end of class Thunder

    } // end of namespace

} // end of namespace
#endif
