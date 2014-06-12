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
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/util/Time.h" 
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/base/spaces/RealVectorStateSpace.h" //temp
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

        // Used for the nearest neighbour tree:
        //typedef std::pair<const base::State*,const base::State*> StatePair;

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
                : saveRequired_(false)
            {
                si_.reset(new base::SpaceInformation(space));

                // Set nearest neighbor type
                nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<ob::PlannerDataPtr>(si_->getStateSpace()));

                // Use our custom distance function for nearest neighbor tree
                nn_->setDistanceFunction(boost::bind(&ompl::tools::ExperienceDB::distanceFunction, this, _1, _2));                

                // Load the PlannerData instance to be used for searching 
                nnSearchKey_.reset(new ob::PlannerData(si_));
                // Add 2 vertexes - one for start and one for goal - so the future searching is faster
                ob::State *temp;
                ob::PlannerDataVertex vert( temp );
                nnSearchKey_->addVertex( vert );
                nnSearchKey_->addVertex( vert );
            }

            virtual ~ExperienceDB(void)
            {
                if (saveRequired_)
                    OMPL_WARN("The database is being unloaded with unsaved experiences");
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
                    ob::PlannerDataPtr plannerData(new ob::PlannerData(si_));
                
                    // Note: the StateStorage class checks if the states match for us
                    plannerDataStorage_.load(iStream, *plannerData.get());

                    OMPL_INFORM("Loaded plan with %d states (vertices)", plannerData->numVertices());

                    // Add to nearest neighbor tree
                    nn_->add(plannerData);
                }
                
                // Close file
                ifStream.close();

                double loadTime = time::seconds(time::now() - start);
                OMPL_INFORM("Loaded database from file in %f sec with %d paths", loadTime, nn_->size()); 
                return true;
            }

            /**
             * \brief Add a new solution path to our database. Des not actually save to file so 
             *        experience will be lost if save() is not called
             */
            void addPath(og::PathGeometric& solutionPath)
            {
                OMPL_INFORM("Adding path to Experience Database");

                // Create a new planner data instance
                ob::PlannerDataPtr plannerData(new ob::PlannerData(si_));
                               
                // Add the states to one nodes files
                for (std::size_t i = 0; i < solutionPath.getStates().size(); ++i)
                {
                    ob::PlannerDataVertex vert( solutionPath.getStates()[i] ); // TODO tag?

                    //OMPL_INFORM("Vertex %d:", i);
                    //debugVertex(vert);

                    plannerData->addVertex( vert );
                }
                
                // TODO: Add the edges to a edges file
                // This might not be necessary actually since we're just using direct paths

                // Deep copy the states in the vertices so that when the planner goes out of scope, all data remains intact
                plannerData->decoupleFromPlanner();

                // Add to nearest neighbor tree
                nn_->add(plannerData);

                saveRequired_ = true;
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

                // Convert the NN tree to a vector
                std::vector<ob::PlannerDataPtr> plannerDatas;
                nn_->list(plannerDatas);

                // Write the number of paths we will be saving
                double numPaths = plannerDatas.size();
                outStream << numPaths;

                // Start saving each planner data object
                for (std::size_t i = 0; i < numPaths; ++i)
                {
                    ob::PlannerData &pd = *plannerDatas[i].get();

                    OMPL_INFORM("Saving experience %d with %d verticies and %d edges", i, pd.numVertices(), pd.numEdges());

                    if (false) // debug code
                    {
                        for (std::size_t i = 0; i < pd.numVertices(); ++i)
                        {
                            OMPL_INFORM("Vertex %d:", i);
                            debugVertex(pd.getVertex(i));
                        }
                    }

                    // Save a single planner data
                    plannerDataStorage_.store(pd, outStream);
                }

                // Close file
                outStream.close();

                // Benchmark
                double loadTime = time::seconds(time::now() - start);
                OMPL_INFORM("Saved database to file in %f sec with %d paths", loadTime, plannerDatas.size()); 

                saveRequired_ = false;

                return true;
            }

            void getAllPaths(std::vector<og::PathGeometric> &paths)
            {
                OMPL_DEBUG("ExperienceDB: getAllPaths");

                // Convert the NN tree to a vector
                std::vector<ob::PlannerDataPtr> plannerDatas;
                nn_->list(plannerDatas);

                OMPL_DEBUG("Number of paths found: %d", plannerDatas.size());

                // Loop through all PlannerDatas
                for (std::size_t i = 0; i < plannerDatas.size(); ++i)
                {
                    ob::PlannerDataPtr plannerData = plannerDatas[i];
                    og::PathGeometric path(si_);

                    // Convert the planner data verticies into a vector of states
                    for (std::size_t i = 0; i < plannerData->numVertices(); ++i)
                    {
                        OMPL_INFORM("Creating path for plan %d", i);
                        debugVertex(plannerData->getVertex(i));

                        path.append(plannerData->getVertex(i).getState());
                    }
                    paths.push_back(path);
                }
            }

            std::vector<ob::PlannerDataPtr> findNearestStartGoal(int nearestK, const base::State* start, const base::State* goal)
            {
                // Fill in our pre-made PlannerData instance with the new start and goal states to be searched for
                nnSearchKey_->getVertex( 0 ) = ob::PlannerDataVertex(start);
                nnSearchKey_->getVertex( 1 ) = ob::PlannerDataVertex(goal);

                std::vector<ob::PlannerDataPtr> nearest;
                nn_->nearestK(nnSearchKey_, nearestK, nearest);

                return nearest;
            }

            /** \brief Add the distance between both path's starts and the distance between both path's ends together
             */
            double distanceFunction(const ob::PlannerDataPtr a, const ob::PlannerDataPtr b) const
            {
                return si_->distance( a->getVertex(0).getState(), b->getVertex(0).getState() ) + 
                    si_->distance( a->getVertex(a->numVertices()-1).getState(), b->getVertex(b->numVertices()-1).getState() );
            }

            void debugVertex(const ob::PlannerDataVertex& vertex)
            {
                const ob::State* state = vertex.getState();
                        
                // Convert to RealVectorStateSpace
                OMPL_INFORM("Converting vertex to real vector");
                const ob::RealVectorStateSpace::StateType *real_state = static_cast<const ob::RealVectorStateSpace::StateType*>(state);

                // Add to vector of results
                OMPL_INFORM("Get value: %f and %f", real_state->values[0], real_state->values[1]);                
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
            
        protected:

            /// The created space information
            base::SpaceInformationPtr     si_; // TODO: is this even necessary?

            /// Helper class for storing each plannerData instance
            ob::PlannerDataStorage plannerDataStorage_;

            // A nearest-neighbors datastructure containing the tree of start/goal states combined
            boost::shared_ptr< NearestNeighbors<ob::PlannerDataPtr> > nn_;

            // Reusable plannerData instance for filling in start and goal and performing searches on the tree
            ob::PlannerDataPtr nnSearchKey_;

            // Flag to determine if a save is required
            bool saveRequired_;

        }; // end of class ExperienceDB

    } // end of namespace

} // end of namespace
#endif
