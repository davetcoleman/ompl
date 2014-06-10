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

#include "ompl/base/StateStorage.h"
#include "ompl/base/StateSpace.h" // for storing to file
#include "ompl/base/SpaceInformation.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/util/Time.h" 
//#include "ompl/base/Planner.h"
//#include "ompl/base/PlannerData.h"
//#include "ompl/base/ProblemDefinition.h"
//#include "ompl/base/ProblemDefinition.h"
//#include "ompl/geometric/PathSimplifier.h"
//#include "ompl/util/Exception.h"
//#include "ompl/tools/multiplan/ParallelPlan.h"
//#include "ompl/tools/config/SelfConfig.h"

namespace og = ompl::geometric;
namespace ob = ompl::base;

namespace ompl
{

    namespace tools
    {
        /**
           @anchor ExperienceDB
           @par Short description
           Database for storing and retrieving past plans
        */

        /// @cond IGNORE
        OMPL_CLASS_FORWARD(ExperienceDB);
        /// @endcond

        /** \class ompl::geometric::ExperienceDBPtr
            \brief A boost shared pointer wrapper for ompl::tools::ExperienceDB */

        /** \brief Built off of SimpleSetup but provides support for planning from experience */
        class ExperienceDB
        {
        public:

            /** \brief Constructor needs the state space used for planning. 
             *  \param space - state space
             *  \param fileNmae - database to read and write from with experiences
             */
            explicit
            ExperienceDB(const base::StateSpacePtr &space)
            {
                si_.reset(new base::SpaceInformation(space));

                OMPL_INFORM("LOADING Experience DATABASE");

                // Create a file storage object
                storageDB_.reset(new ob::GraphStateStorage(space)); // TODO: don't use Graph, just regular?
            }

            virtual ~ExperienceDB(void)
            {
            }

            void print(std::ostream &out) const
            {
                /*
                  if (si_)
                  {
                  si_->printProperties(out);
                  si_->printSettings(out);
                  }
                  if (planner_)
                  {
                  planner_->printProperties(out);
                  planner_->printSettings(out);
                  }
                  if (pdef_)
                  pdef_->print(out);
                */
            }

            void addPath(og::PathGeometric& solutionPath)
            {
                // Add the states to one nodes files
                for (std::size_t i = 0; i < solutionPath.getStates().size(); ++i)
                {
                    storageDB_->addState( solutionPath.getStates()[i] );
                }
                
                // Add the edges to a edges file
                

                storageDB_->store(fileName_.c_str());
            }

            void load(const std::string& fileName)
            {
                // Remember so that we can save later
                fileName_ = fileName;

                // Load database from file, track loading time
                time::point start = time::now();
                OMPL_INFORM("Loading database from file: %s", fileName_.c_str());

                // Note: the StateStorage class checks if the states match for us
                storageDB_->load(fileName_.c_str());

                // Check how many states are stored
                OMPL_INFORM("Loaded %d states", storageDB_->size());

                double loadTime = time::seconds(time::now() - start);
                OMPL_INFORM("Loaded database from file in %f sec", loadTime);
            }

            std::vector<const ompl::base::State*> getStates()
            {
                return storageDB_->getStates();
            }
            
        protected:

            /// The created space information
            base::SpaceInformationPtr     si_; // TODO: is this even necessary?

            /// The storage file
            ompl::base::GraphStateStoragePtr storageDB_;

            /// The storage file location
            std::string fileName_;

        }; // end of class ExperienceDB

    } // end of namespace

} // end of namespace
#endif
