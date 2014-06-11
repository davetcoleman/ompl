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

// OMPL
#include "ompl/base/PlannerDataStorage.h"
#include "ompl/base/StateSpace.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/util/Time.h" 

// Boost
#include <boost/filesystem.hpp>

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
                //storageDB_.reset(new ob::PlannerDataStorage()); // TODO: don't use Graph, just regular?
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
                // Create a new planner data instance
                ompl::base::PlannerDataPtr pd(new ompl::base::PlannerData(si_));
                               
                // Add the states to one nodes files
                for (std::size_t i = 0; i < solutionPath.getStates().size(); ++i)
                {
                    ompl::base::PlannerDataVertex vert( solutionPath.getStates()[i] ); // TODO tag?
                    pd->addVertex( vert );
                }
                
                // Add the edges to a edges file
                // TODO

                // Save to vector
                plannerDatas_.push_back(pd);

                // note: does not save to file
            }
            
            bool save(const std::string& fileName)
            {
                // Error checking
                if (fileName.empty())
                {
                    OMPL_ERROR("Empty filename passed to save function");
                    return false;
                }

                // Save database from file, track saving time
                time::point start = time::now();

                OMPL_INFORM("Saving database to file: %s", fileName.c_str());                

                // Open a binary output stream
                std::ofstream outStream(fileName.c_str(), std::ios::binary);

                // Write the number of paths we will be saving
                double numPaths = plannerDatas_.size();
                outStream << numPaths;

                // Start saving each planner data object
                for (std::size_t i = 0; i < numPaths; ++i)
                {
                    // Save a single planner data
                    storageDB_.store((*plannerDatas_[i].get()), outStream);
                }

                // Close file
                outStream.close();

                // Benchmark
                double loadTime = time::seconds(time::now() - start);
                OMPL_INFORM("Saved database to file in %f sec with %d paths", loadTime, plannerDatas_.size()); 

                return true;
            }

            bool load(const std::string& fileName)
            {
                // Error checking
                if (fileName.empty())
                {
                    OMPL_ERROR("Empty filename passed to save function");
                    return false;
                }
                if ( !boost::filesystem::exists( fileName ) )
                {
                    OMPL_WARN("Database file does not exist: %s", fileName.c_str());
                    return false;
                }

                // Load database from file, track loading time
                time::point start = time::now();

                OMPL_INFORM("Loading database from file: %s", fileName.c_str());

                // Open a binary input stream
                std::ifstream ifStream(fileName.c_str(), std::ios::binary);
                std::istream& iStream = ifStream;

                // Get the total number of paths saved
                double numPaths = 0;
                iStream >> numPaths;

                // Check that the number of paths makes sense
                if (numPaths < 0 || numPaths > std::numeric_limits<double>::max())
                {
                    OMPL_WARN("Number of paths to load %d is a bad value", numPaths);
                    return false;
                }

                // Start loading all the PlannerDatas
                for (std::size_t i = 0; i < numPaths; ++i)
                {                    
                    // Create a new planner data instance
                    ompl::base::PlannerDataPtr pd(new ompl::base::PlannerData(si_));
                
                    // Note: the StateStorage class checks if the states match for us
                    storageDB_.load(iStream, *pd.get());

                    OMPL_INFORM("Loaded plan with %d states (vertices)", pd->numVertices());

                    // Add planner data to vector
                    plannerDatas_.push_back(pd);
                }
                
                // Close file
                ifStream.close();

                double loadTime = time::seconds(time::now() - start);
                OMPL_INFORM("Loaded database from file in %f sec with %d paths", loadTime, plannerDatas_.size()); 
                return true;
            }

            // Deprecated
            std::vector<const ompl::base::State*> getStates()
            {
                std::vector<const ompl::base::State*> states;

                // TODO: make more than just 1
                if (plannerDatas_.empty())
                {
                    OMPL_INFORM("There are no planner datas loaded");
                }
                else
                {
                    ompl::base::PlannerDataPtr pd = plannerDatas_[0];

                    // Convert the planner data verticies into a vector of states
                    for (std::size_t i = 0; i < pd->numVertices(); ++i)
                    {
                        states.push_back(pd->getVertex(i).getState());
                    }

                }
                return states;
            }

            std::vector<ompl::geometric::PathGeometric> getAllPaths()
            {
                std::vector<ompl::geometric::PathGeometric> paths;

                // Loop through all PlannerDatas
                for (std::size_t i = 0; i < plannerDatas_.size(); ++i)
                {
                    ompl::base::PlannerDataPtr pd = plannerDatas_[i];
                    ompl::geometric::PathGeometric path(si_);

                    // Convert the planner data verticies into a vector of states
                    for (std::size_t i = 0; i < pd->numVertices(); ++i)
                    {
                        path.append(pd->getVertex(i).getState());
                    }
                    paths.push_back(path);
                }
                return paths;
            }
            
        protected:

            /// The created space information
            base::SpaceInformationPtr     si_; // TODO: is this even necessary?

            /// The storage file
            ompl::base::PlannerDataStorage storageDB_;

            // Create a new planner data instance to be loaded
            std::vector<ompl::base::PlannerDataPtr> plannerDatas_;

        }; // end of class ExperienceDB

    } // end of namespace

} // end of namespace
#endif
