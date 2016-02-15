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

#ifndef OMPL_TOOLS_BOLT_BOLT_RETRIEVE_REPAIR_
#define OMPL_TOOLS_BOLT_BOLT_RETRIEVE_REPAIR_

#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/datastructures/NearestNeighbors.h>

namespace ompl
{
    namespace tools
    {
        OMPL_CLASS_FORWARD(BoltDB);
    }

    namespace geometric
    {

        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::BoltRetrieveRepair */
        OMPL_CLASS_FORWARD(BoltRetrieveRepair);
        /// @endcond

        /** \class ompl::base::BoltRetrieveRepairPtr
            \brief A boost shared pointer wrapper for ompl::base::BoltRetrieveRepair */

        /**
           @anchor BoltRetrieveRepair
           @par Short description
           Bolt is an experience-based planning framework that learns to reduce computation time
           required to solve high-dimensional planning problems in varying environments.
           @par External documentation
           Berenson, Dmitry, Pieter Abbeel, and Ken Goldberg: A robot path planning framework that learns from experience, in <em>Robotics and Automation (ICRA), 2012 IEEE International Conference on. IEEE</em>, 2012.
           David Coleman, Ioan A. Sucan, Mark Moll, Kei Okada, Nikolaus Correll, "Experience-Based Planning with Sparse Roadmap Spanners"
           <a href="http://arxiv.org/pdf/1410.1950.pdf">[PDF]</a>
        */

        /** \brief The Bolt Framework's Retrieve-Repair component */
        class BoltRetrieveRepair : public base::Planner
        {
        public:

            /** \brief Constructor */
            BoltRetrieveRepair(const base::SpaceInformationPtr &si, const tools::BoltDBPtr &experienceDB);

            virtual ~BoltRetrieveRepair(void);

            /** \brief Get information about the exploration data structure the planning from scratch motion planner used. */
            virtual void getPlannerData(base::PlannerData &data) const;

            /**
             *  \brief Get debug information about the top recalled paths that were chosen for further filtering
             *  \return data - vector of PlannerData objects that each hold a single path
             */
            const std::vector<PathGeometric>& getLastRecalledNearestPaths() const;

            /**
             *  \brief Get debug information about the top recalled paths that were chosen for further filtering
             *  \return chosenID - the index of the PlannerData object that was chosen for repair
             */
            std::size_t getLastRecalledNearestPathChosen() const;

            /**
             * \brief Get the chosen path used from database for repair
             * \return PlannerData of chosen path
             */
            const PathGeometric& getChosenRecallPath() const;

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            virtual void clear(void);

            /**
             * \brief Pass a pointer of the database from the bolt framework
             */
            void setExperienceDB(const tools::BoltDBPtr &experienceDB);

            virtual void setup(void);

            /** \brief Optionally smooth retrieved and repaired paths from database */
            void enableSmoothing(bool enable)
            {
                smoothingEnabled_ =  enable;
            }

        protected:

            /**
             * \brief Count the number of states along the discretized path that are in collision
             *        Note: This is kind of an ill-defined score though. It depends on the resolution of collision checking.
             *        I am more inclined to try to compute the percent of the length of the motion that is valid.
             *        That could go in SpaceInformation, as a utility function.
             */
            std::size_t checkMotionScore(const base::State *s1, const base::State *s2) const;

            /** \brief Free the memory allocated by this planner */
            void freeMemory(void);

            /** \brief The database of motions to search through */
            tools::BoltDBPtr                             experienceDB_;

            /** \brief Save the recalled path before smoothing for introspection later */
            boost::shared_ptr<PathGeometric>             foundPath_;

            /** \brief The instance of the path simplifier */
            PathSimplifierPtr                            path_simplifier_;

            /** \brief Optionally smooth retrieved and repaired paths from database */
            bool                                         smoothingEnabled_;
        };

    } // namespace geometric
} // namespace ompl

#endif
