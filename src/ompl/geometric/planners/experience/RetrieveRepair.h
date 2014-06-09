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
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/tools/lightning/ExperienceDB.h"

namespace ompl
{

    namespace geometric
    {

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
            RetrieveRepair(const base::SpaceInformationPtr &si, ompl::tools::ExperienceDBPtr experience_db);

            virtual ~RetrieveRepair(void);

            virtual void getPlannerData(base::PlannerData &data) const;

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            virtual void clear(void);

            /** \brief Set a different nearest neighbors datastructure */
            template<template<typename T> class NN>
            void setNearestNeighbors(void)
            {
                nn_.reset(new NN<Motion*>());
            }

            void setExperienceDB(ompl::tools::ExperienceDBPtr experience_db);

            virtual void setup(void);

        protected:


            /** \brief Representation of a motion

                This only contains pointers to parent motions as we
                only need to go backwards in the tree. */
            class Motion
            {
            public:

                Motion(void) : state(NULL), parent(NULL)
                {
                }

                /** \brief Constructor that allocates memory for the state */
                Motion(const base::SpaceInformationPtr &si) : state(si->allocState()), parent(NULL)
                {
                }

                ~Motion(void)
                {
                }

                /** \brief The state contained by the motion */
                base::State       *state;

                /** \brief The parent motion in the exploration tree */
                Motion            *parent;

            };

            /** \brief Free the memory allocated by this planner */
            void freeMemory(void);

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion* a, const Motion* b) const
            {
                return si_->distance(a->state, b->state);
            }

            /** \brief State sampler */
            base::StateSamplerPtr                          sampler_;

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            boost::shared_ptr< NearestNeighbors<Motion*> > nn_;

            /** \brief The random number generator */
            RNG                                            rng_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion                                         *lastGoalMotion_;

            /** \breif The database of motions to search through */
            ompl::tools::ExperienceDBPtr experience_db_;
        };

    }
}

#endif
