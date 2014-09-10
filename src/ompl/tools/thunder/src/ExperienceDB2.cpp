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
#include "ompl/tools/thunder/ExperienceDB2.h"
#include "ompl/base/ScopedState.h"
#include "ompl/util/Time.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/base/PlannerDataStorage.h"

// Boost
#include <boost/filesystem.hpp>

ompl::tools::ExperienceDB2::ExperienceDB2(const base::StateSpacePtr &space)
    : numUnsavedPaths_(0)
    , saving_enabled_(true)
{
    // Set space information
    si_.reset(new base::SpaceInformation(space));
}

ompl::tools::ExperienceDB2::~ExperienceDB2(void)
{
    if (numUnsavedPaths_)
        OMPL_WARN("The database is being unloaded with unsaved experiences");
}

bool ompl::tools::ExperienceDB2::load(const std::string& fileName)
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
    if (!spars_)
    {
        OMPL_ERROR("SPARStwo planner has not been passed into the ExperienceDB2 yet");
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

    if (numPaths > 1)
    {
        OMPL_WARN("Currently more than one planner data is disabled from loading");
        return false;
    }

    // Start loading all the PlannerDatas
    for (std::size_t i = 0; i < numPaths; ++i)
    {
        // Create a new planner data instance
        ompl::base::PlannerDataPtr plannerData(new ompl::base::PlannerData(si_));

        // Note: the StateStorage class checks if the states match for us
        plannerDataStorage_.load(iStream, *plannerData.get());

        OMPL_INFORM("ExperienceDB2: Loaded plan with \n  %d vertices\n  %d edges\n  %d start states\n  %d goal states", 
                    plannerData->numVertices(), plannerData->numEdges(), plannerData->numStartVertices(), plannerData->numGoalVertices());

        // Add to SPARStwo
        spars_->setPlannerData(*plannerData);
    }

    // Close file
    iStream.close();

    double loadTime = time::seconds(time::now() - start);
    OMPL_INFORM("Loaded database from file in %f sec ", loadTime);
    return true;
}

void ompl::tools::ExperienceDB2::addPath(ompl::geometric::PathGeometric& solutionPath)
{
    // Error check
    if (!spars_)
    {
        OMPL_ERROR("SPARStwo planner has not been passed into the ExperienceDB2 yet");
        return;
    }

    std::cout << std::endl;
    std::cout << "ExperienceDB2::addPath(): Before add SPARStwo has " << spars_->milestoneCount() << " states" << std::endl;

    double seconds = 10; // a large number, should never need to use this
    ompl::base::PlannerTerminationCondition ptc = ompl::base::timedPlannerTerminationCondition( seconds, 0.1 ); 
    spars_->addPathToRoadmap(ptc, solutionPath);

    std::cout << "SPARStwo now has " << spars_->milestoneCount() << " states" << std::endl;

    numUnsavedPaths_++;
}

bool ompl::tools::ExperienceDB2::saveIfChanged(const std::string& fileName)
{
    if (numUnsavedPaths_)
        return save(fileName);
    else
        OMPL_INFORM("Not saving because database has not changed");
    return true;
}

bool ompl::tools::ExperienceDB2::save(const std::string& fileName)
{
    // Disabled
    if (!saving_enabled_)
    {
        OMPL_WARN("Not saving because option disabled for ExperienceDB");
        return false;
    }

    // Error checking
    if (fileName.empty())
    {
        OMPL_ERROR("Empty filename passed to save function");
        return false;
    }
    if (!spars_)
    {
        OMPL_ERROR("SPARStwo planner has not been passed into the ExperienceDB2 yet");
        return false;
    }

    // Save database from file, track saving time
    time::point start = time::now();

    OMPL_INFORM("Saving database to file: %s", fileName.c_str());

    // Open a binary output stream
    std::ofstream outStream(fileName.c_str(), std::ios::binary);

    // Populate multiple planner Datas
    std::vector<ompl::base::PlannerDataPtr> plannerDatas;

    // TODO: make this more than 1 planner data perhaps
    base::PlannerDataPtr data(new base::PlannerData(si_));
    spars_->getPlannerData(*data);
    OMPL_INFORM("Get planner data from SPARS2 with \n  %d vertices\n  %d edges\n  %d start states\n  %d goal states", 
                data->numVertices(), data->numEdges(), data->numStartVertices(), data->numGoalVertices());

    plannerDatas.push_back(data);

    // Write the number of paths we will be saving
    double numPaths = plannerDatas.size();
    outStream << numPaths;

    // Start saving each planner data object
    for (std::size_t i = 0; i < numPaths; ++i)
    {
        ompl::base::PlannerData &pd = *plannerDatas[i].get();

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
    OMPL_INFORM("Saved database to file in %f sec with %d planner datas", loadTime, plannerDatas.size());

    numUnsavedPaths_ = 0;

    return true;
}

void ompl::tools::ExperienceDB2::setSPARStwo(ompl::tools::SPARStwoPtr &prm)
{
    // std::cout << "-------------------------------------------------------" << std::endl;
    // std::cout << "setSPARStwo " << std::endl;
    // std::cout << "-------------------------------------------------------" << std::endl;
    spars_ = prm;
}

ompl::tools::SPARStwoPtr& ompl::tools::ExperienceDB2::getSPARStwo()
{
    return spars_;
}

void ompl::tools::ExperienceDB2::getAllPlannerDatas(std::vector<ompl::base::PlannerDataPtr> &plannerDatas) const
{
    if (!spars_)
    {
        OMPL_ERROR("SPARStwo planner has not been passed into the ExperienceDB2 yet");
        return;
    }

    if (getExperiencesCount() == 0)
    {
        OMPL_INFORM("No paths found");
        return;
    }

    base::PlannerDataPtr data(new base::PlannerData(si_));
    spars_->getPlannerData(*data);
    plannerDatas.push_back(data);

    //OMPL_DEBUG("ExperienceDB2::getAllPlannerDatas: Number of planner databases found: %d", plannerDatas.size());
}

bool ompl::tools::ExperienceDB2::findNearestStartGoal(int nearestK, const base::State* start, const base::State* goal,
                                                      ompl::geometric::SPARStwo::CandidateSolution &candidateSolution,
                                                      const base::PlannerTerminationCondition& ptc)
{
    bool result;

    // Benchmark runtime
    time::point startTime = time::now();
    std::size_t problems = 1;
    
    std::cout << "Preparing to benchmark... " << std::endl;
    std::cout << std::endl;

    for (std::size_t i = 0; i < problems; ++i)
    {
        std::cout << std::endl;
        std::cout << "----------------------------------------------------------- " << std::endl;

        result = spars_->getSimilarPaths(nearestK, start, goal, candidateSolution, ptc);

        // Clear edge collision state for testing purposes
        if (problems > 1)
        {
            spars_->clearEdgeCollisionStates();
        }
    }

    // Benchmark runtime
    double milliseconds = time::seconds(time::now() - startTime);
    std::cout << "Total time: " << milliseconds << " seconds averaging " << milliseconds/problems
              << " seconds per recalled path for "<< problems << " runs" << std::endl;

    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << std::endl;

    if (!result)
    {
        OMPL_INFORM("spars::getSimilarPaths() returned false - does not have a solution");
        return false;
    }

    OMPL_INFORM("spars::getSimilarPaths() returned true - found a solution of size %d", 
                candidateSolution.getStateCount());
    return true;
}

void ompl::tools::ExperienceDB2::debugVertex(const ompl::base::PlannerDataVertex& vertex)
{
    debugState(vertex.getState());
}

void ompl::tools::ExperienceDB2::debugState(const ompl::base::State* state)
{
    si_->printState(state, std::cout);
}

std::size_t ompl::tools::ExperienceDB2::getExperiencesCount() const
{
    if (!spars_)
    {
        OMPL_ERROR("SPARStwo planner has not been passed into the ExperienceDB2 yet");
        return 0;
    }
    return spars_->milestoneCount(); // Get the number of milestones in the graph
}
