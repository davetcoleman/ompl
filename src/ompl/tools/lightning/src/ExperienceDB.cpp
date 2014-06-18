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

/* Author: Dave Coleman */

// OMPL
#include "ompl/tools/lightning/ExperienceDB.h"
#include "ompl/base/ScopedState.h"
#include "ompl/util/Time.h" 
#include "ompl/tools/config/SelfConfig.h"

// Boost
#include <boost/filesystem.hpp>

ompl::tools::ExperienceDB::ExperienceDB(const base::StateSpacePtr &space)
    : saveRequired_(false)
{
    si_.reset(new base::SpaceInformation(space));

    // Set nearest neighbor type
    //nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<ob::PlannerDataPtr>(si_->getStateSpace()));
    nn_.reset(new ompl::NearestNeighborsSqrtApprox<ob::PlannerDataPtr>());

    // Use our custom distance function for nearest neighbor tree
    nn_->setDistanceFunction(boost::bind(&ompl::tools::ExperienceDB::distanceFunction, this, _1, _2));

    // Load the PlannerData instance to be used for searching
    nnSearchKey_.reset(new ob::PlannerData(si_));
}

ompl::tools::ExperienceDB::~ExperienceDB(void)
{
    if (saveRequired_)
        OMPL_WARN("The database is being unloaded with unsaved experiences");
}

bool ompl::tools::ExperienceDB::load(const std::string& fileName)
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
    std::ifstream iStream(fileName.c_str(), std::ios::binary);

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

        //OMPL_INFORM("Loaded plan with %d states (vertices)", plannerData->numVertices());

        // Add to nearest neighbor tree
        nn_->add(plannerData);
    }

    // Close file
    iStream.close();

    double loadTime = time::seconds(time::now() - start);
    OMPL_INFORM("Loaded database from file in %f sec with %d paths", loadTime, nn_->size());
    return true;
}

void ompl::tools::ExperienceDB::addPath(og::PathGeometric& solutionPath)
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

bool ompl::tools::ExperienceDB::saveIfChanged(const std::string& fileName)
{
    if (saveRequired_)
        return save(fileName);
    return true;
}

bool ompl::tools::ExperienceDB::save(const std::string& fileName)
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

        //OMPL_INFORM("Saving experience %d with %d verticies and %d edges", i, pd.numVertices(), pd.numEdges());

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

void ompl::tools::ExperienceDB::getAllPaths(std::vector<ob::PlannerDataPtr> &plannerDatas)
{
    OMPL_DEBUG("ExperienceDB: getAllPaths");

    // Convert the NN tree to a vector
    nn_->list(plannerDatas);

    OMPL_DEBUG("Number of paths found: %d", plannerDatas.size());
}

std::vector<ob::PlannerDataPtr> ompl::tools::ExperienceDB::findNearestStartGoal(int nearestK, const base::State* start, const base::State* goal)
{
    // Fill in our pre-made PlannerData instance with the new start and goal states to be searched for
    if (nnSearchKey_->numVertices() == 2)
    {
        nnSearchKey_->getVertex( 0 ) = ob::PlannerDataVertex(start);
        nnSearchKey_->getVertex( 1 ) = ob::PlannerDataVertex(goal);   
    }
    else
    {
        nnSearchKey_->addVertex( ob::PlannerDataVertex(start) );
        nnSearchKey_->addVertex( ob::PlannerDataVertex(goal) );
    }
    assert( nnSearchKey_->numVertices() == 2);

    std::vector<ob::PlannerDataPtr> nearest;
    nn_->nearestK(nnSearchKey_, nearestK, nearest);

    return nearest;
}

double ompl::tools::ExperienceDB::distanceFunction(const ob::PlannerDataPtr a, const ob::PlannerDataPtr b) const
{
    // Basic implementation
    /*
    return si_->distance( a->getVertex(0).getState(), b->getVertex(0).getState() ) +
        si_->distance( a->getVertex(a->numVertices()-1).getState(), b->getVertex(b->numVertices()-1).getState() );
    */

    // Bi-directional implementation - check path b from [start, goal] and [goal, start] 
    return std::min(
        // [ a.start, b.start] + [a.goal + b.goal]
        si_->distance( a->getVertex(0).getState(), b->getVertex(0).getState() ) + 
        si_->distance( a->getVertex(a->numVertices()-1).getState(), b->getVertex(b->numVertices()-1).getState() ),
        // [ a.start, b.goal] + [a.goal + b.start]
        si_->distance( a->getVertex(0).getState(), b->getVertex(b->numVertices()-1).getState() ) + 
        si_->distance( a->getVertex(a->numVertices()-1).getState(), b->getVertex(0).getState() ) );

    /*
    // [ a.start, b.start] + [a.goal + b.goal]
    double n1 = si_->distance( a->getVertex(0).getState(), b->getVertex(0).getState() );
    double n2 = si_->distance( a->getVertex(a->numVertices()-1).getState(), b->getVertex(b->numVertices()-1).getState() );

    // [ a.start, b.goal] + [a.goal + b.start]
    double w1 = si_->distance( a->getVertex(0).getState(), b->getVertex(b->numVertices()-1).getState() );
    double w2 = si_->distance( a->getVertex(a->numVertices()-1).getState(), b->getVertex(0).getState() );

    double dist = std::min(n1 + n2, w1 + w2);

    std::cout << "a count " << a->numVertices() << " b count " << b->numVertices() << std::endl;

    if (n1 + n2 <= w1 + w2)
        std::cout << "Dist normal " << dist << " | n1: " << n1 << " | n2: " << n2 << std::endl;
    else
        std::cout << "Dist weird  " << dist << " | w1: " << w1 << " | w2: " << w2 << std::endl;

    return dist;
    */
}

void ompl::tools::ExperienceDB::debugVertex(const ob::PlannerDataVertex& vertex)
{
    debugState(vertex.getState());
}

void ompl::tools::ExperienceDB::debugState(const ob::State* state)
{
    si_->printState(state, std::cout);    
}

std::size_t ompl::tools::ExperienceDB::getExperiencesCount()
{
    return nn_->size();
}
