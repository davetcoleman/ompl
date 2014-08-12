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


#include "ompl/tools/thunder/Thunder.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/geometric/SimpleSetup.h" // use their implementation of getDefaultPlanner
#include "ompl/base/StateSpace.h" // for storing to file
#include "ompl/tools/thunder/ExperienceDB2.h"
#include "ompl/tools/multiplan/ParallelPlan.h"

// Boost
#include <boost/filesystem.hpp>

namespace og = ompl::geometric;
namespace ob = ompl::base;
namespace ot = ompl::tools;

ompl::tools::Thunder::Thunder(const base::SpaceInformationPtr &si) :
    ompl::geometric::SimpleSetup(si)
{
    initialize();
}

ompl::tools::Thunder::Thunder(const base::StateSpacePtr &space) :
    ompl::geometric::SimpleSetup(space)
{
    initialize();
}

void ompl::tools::Thunder::initialize()
{
    std::cout << OMPL_CONSOLE_COLOR_CYAN << std::endl;
    std::cout << std::endl;
    std::cout << "Initializing Thunder Framework ///////////////////////////////////////" << std::endl;
    std::cout << OMPL_CONSOLE_COLOR_RESET;

    recallEnabled_ = true;
    scratchEnabled_ = true;
    filePath_ = "unloaded";

    // Load dynamic time warp
    dtw_.reset(new ot::DynamicTimeWarp(si_));

    // Load the experience database
    experienceDB_.reset(new ompl::tools::ExperienceDB2(si_->getStateSpace()));

    // Load the Retrieve repair database. We do it here so that setRepairPlanner() works
    rrPlanner_ = ob::PlannerPtr(new og::RetrieveRepairPRM(si_, experienceDB_));

    OMPL_INFORM("Thunder Framework initialized.");
    std::cout << std::endl;
    std::cout << std::endl;
}

bool ompl::tools::Thunder::load(const std::string &databaseName, const std::string &databaseDirectory)
{
    setup();  // ensure the PRM db has been loaded to the Experience DB

    getFilePath(databaseName, databaseDirectory);

    return experienceDB_->load(filePath_); // load from file
}

void ompl::tools::Thunder::setup(void)
{
    if (!configured_ || !si_->isSetup() || !planner_->isSetup() || !rrPlanner_->isSetup() )
    {
        // Setup Space Information if we haven't already done so
        if (!si_->isSetup())
            si_->setup();

        // Setup planning from scratch planner
        if (!planner_)
        {
            if (pa_)
                planner_ = pa_(si_);
            if (!planner_)
            {
                planner_ = ompl::geometric::getDefaultPlanner(pdef_->getGoal()); // we could use the repairProblemDef_ here but that isn't setup yet

                OMPL_INFORM("No planner specified. Using default: %s", planner_->getName().c_str() );
            }
        }
        planner_->setProblemDefinition(pdef_);
        if (!planner_->isSetup())
            planner_->setup();

        // Setup planning from experience planner
        rrPlanner_->setProblemDefinition(pdef_);

        if (!rrPlanner_->isSetup())
            rrPlanner_->setup();

        // Setup parameters
        params_.clear();
        params_.include(si_->params());
        params_.include(planner_->params());
        params_.include(rrPlanner_->params());

        // Create the parallel component for splitting into two threads
        pp_ = ot::ParallelPlanPtr(new ot::ParallelPlan(pdef_) );
        if (!scratchEnabled_ && !recallEnabled_)
        {
            throw Exception("Both planning from scratch and experience have been disabled, unable to plan");
        }
        if (scratchEnabled_)
            pp_->addPlanner(planner_);   // Add the planning from scratch planner if desired
        if (recallEnabled_)
            pp_->addPlanner(rrPlanner_);  // Add the planning from experience planner if desired


        // Setup the PRM for the experience DB 
        // TODO: make this the same one as the repair planner?
        if (!experienceDB_->getSPARStwo())
        {
            boost::shared_ptr<ompl::geometric::SPARStwo> spars;
            spars.reset(new ompl::geometric::SPARStwo(si_));
            spars->setProblemDefinition(pdef_);

            if (!spars->isSetup())
            {
                std::cout << "Calling setup " << std::endl;
                spars->setup();
            }

            spars->printDebug();
            spars->setStretchFactor(1.2);
            spars->setSparseDeltaFraction(0.1); // vertex visibility range  = maximum_extent * this_fraction

            //spars->setDenseDeltaFraction(0.001);

            experienceDB_->setSPARStwo(spars);
        }

        // Set the configured flag
        configured_ = true;
    }
}

bool ompl::tools::Thunder::getFilePath(const std::string &databaseName, const std::string &databaseDirectory)
{
    namespace fs = boost::filesystem;

    // Check that the directory exists, if not, create it
    fs::path rootPath;
    if (!std::string(getenv("HOME")).empty())
        rootPath = fs::path(getenv("HOME")); // Support Linux/Mac
    else if (!std::string(getenv("HOMEPATH")).empty())
        rootPath = fs::path(getenv("HOMEPATH")); // Support Windows
    else
    {
        OMPL_WARN("Unable to find a home path for this computer");
        rootPath = fs::path("");
    }

    rootPath = rootPath / fs::path(databaseDirectory);

    boost::system::error_code returnedError;
    fs::create_directories( rootPath, returnedError );

    if ( returnedError )
    {
        //did not successfully create directories
        OMPL_ERROR("Unable to create directory %s", databaseDirectory.c_str());
        return false;
    }

    //directories successfully created, append the group name as the file name
    rootPath = rootPath / fs::path(databaseName + ".ompl");
    filePath_ = rootPath.string();
    OMPL_INFORM("Loading database from %s", filePath_.c_str());

    return true;
}

void ompl::tools::Thunder::clear(void)
{
    if (planner_)
        planner_->clear();
    if (rrPlanner_)
        rrPlanner_->clear();
    if (pdef_)
        pdef_->clearSolutionPaths();
    if (pp_)
    {
        pp_->clearHybridizationPaths();
        pp_->clearPlanners();
    }
}

// we provide a duplicate implementation here to allow the planner to choose how the time is turned into a planner termination condition
ompl::base::PlannerStatus ompl::tools::Thunder::solve(const base::PlannerTerminationCondition &ptc)
{
    OMPL_INFORM("Thunder Framework: Starting solve()");

    // Setup again in case it has not been done yet
    setup(); 

    lastStatus_ = base::PlannerStatus::UNKNOWN;
    time::point start = time::now();

    // Start both threads
    bool hybridize = false;
    lastStatus_ = pp_->solve(ptc, hybridize);

    // Planning time
    planTime_ = time::seconds(time::now() - start);
    logs_.totalPlanningTime_ += planTime_; // used for averaging
    logs_.numProblems_++; // used for averaging
    logs_.csvDataLogStream_ << planTime_ << ",";

    // Skip further processing if absolutely no path is available
    if (!lastStatus_)
    {
        OMPL_ERROR("Thunder Solve: No solution found after %f seconds", planTime_);
        logs_.numSolutionsFailed_++;

        logs_.csvDataLogStream_ << "0,neither_planner,failed,not_saved,0,";
    }
    else
    {
        OMPL_INFORM("Thunder Solve: Possible solution found in %f seconds", planTime_);


        // Smooth the result
        //OMPL_INFORM("Simplifying solution (smoothing)...");
        //simplifySolution(ptc);

        // Get information about the exploration data structure the motion planner used. Used later in visualizing
        og::PathGeometric solutionPath = getSolutionPath(); // copied so that it is non-const
        OMPL_INFORM("Solution path has %d states and was generated from planner %s", solutionPath.getStateCount(), getSolutionPlannerName().c_str());
        logs_.csvDataLogStream_ << solutionPath.getStateCount() << "," << getSolutionPlannerName() << ",";

        //solutionPath.print(std::cout);

        std::cout << OMPL_CONSOLE_COLOR_CYAN << "THUNDER RESULTS:" << OMPL_CONSOLE_COLOR_RESET << std::endl;

        // Do not save if approximate
        if (!haveExactSolutionPath())
        {
            logs_.csvDataLogStream_ << "not_exact_solution_path,not_saved,0,";
            logs_.numSolutionsApproximate_++;

            // TODO not sure what to do here, use case not tested
            OMPL_WARN("NOT saving to database because the solution is APPROXIMATE");
        }
        // Use dynamic time warping to see if the repaired path is too similar to the original
        else if (getSolutionPlannerName() == rrPlanner_->getName())
        {
            OMPL_INFORM("Solution from Recall");

            // Logging
            logs_.numSolutionsFromRecall_++;

            // Make sure solution has at least 2 states
            if (solutionPath.getStateCount() < 2)
            {
                OMPL_INFORM("NOT saving to database because solution is less than 2 states long");
                logs_.numSolutionsTooShort_++;
                logs_.csvDataLogStream_ << "from_recall,less_2_states,0,";
            }
            else
            {
                // Convert the original recalled path to PathGeometric
                ob::PlannerDataPtr chosenRecallPathData = getRetrieveRepairPlanner().getChosenRecallPath();
                og::PathGeometric chosenRecallPath(si_);
                convertPlannerData(chosenRecallPathData, chosenRecallPath);

                // Reverse path2 if necessary so that it matches path1 better
                reversePathIfNecessary(solutionPath, chosenRecallPath);

                double score = dtw_->getPathsScoreHalfConst( solutionPath, chosenRecallPath );

                //  TODO: From Ioan: It would be nice if we switch to percentages of path length for score, and maybe define this var in the magic namespace.
                //  From Dave: Actually I think I already made the getPathsScore function length-invariant by dividing the score by the path
                if (score < 4) //10)
                {
                    OMPL_ERROR("NOT saving to database because best solution was from database and is too similar (score %f)", score);
                    logs_.csvDataLogStream_ << "from_recall,score_too_similar," << score << ",";
                }
                else
                {
                    OMPL_INFORM("Adding path to database because repaired path is different enough from original recalled path (score %f)", score);
                    logs_.csvDataLogStream_ << "from_recall,score_different_enough," << score << ",";

                    // Logging
                    logs_.numSolutionsFromRecallSaved_++;

                    // Save to database
                    experienceDB_->addPath(solutionPath);
                }
            }
        }
        else
        {
            OMPL_INFORM("Solution from Scratch");

            // Logging
            logs_.numSolutionsFromScratch_++;

            // Make sure solution has at least 2 states
            if (solutionPath.getStateCount() < 2)
            {
                OMPL_INFORM("NOT saving to database because solution is less than 2 states long");
                logs_.csvDataLogStream_ << "from_scratch,less_2_states,0,";
                logs_.numSolutionsTooShort_++;
            }
            else
            {
                OMPL_INFORM("Adding path to database because best solution was not from database");
                logs_.csvDataLogStream_ << "from_scratch,saving,0,";

                // TODO: maybe test this path for validity also?

                // Save to database
                

                /*
                OMPL_WARN("Temp not adding path but instead whole planner data");
                base::PlannerData pd( si_ );
                getPlannerData(pd);
                experienceDB_->addPlannerData(pd);
                */
                experienceDB_->addPath(solutionPath);
            }
        }
    }

    // End this log entry with remaining stats
    logs_.csvDataLogStream_
        << logs_.numSolutionsFromScratch_ << ","
        << logs_.numSolutionsFromRecall_ << ","
        << logs_.numSolutionsFromRecallSaved_ << ","
        << logs_.numSolutionsFromRecall_ - logs_.numSolutionsFromRecallSaved_ << ","
        << logs_.numSolutionsFailed_ << ","
        << logs_.numSolutionsApproximate_ << ","
        << logs_.numSolutionsTooShort_ << ","
        << experienceDB_->getExperiencesCount() << ","
        << logs_.getAveragePlanningTime() << std::endl;

    std::cout << "Leaving Thunder::solve() fn " << std::endl;
    return lastStatus_;
}

ompl::base::PlannerStatus ompl::tools::Thunder::solve(double time)
{
    ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition( time );
    return solve(ptc);
}

bool ompl::tools::Thunder::save()
{
    setup(); // ensure the PRM db has been loaded to the Experience DB
    return experienceDB_->save(filePath_);
}

bool ompl::tools::Thunder::saveIfChanged()
{
    setup(); // ensure the PRM db has been loaded to the Experience DB
    return experienceDB_->saveIfChanged(filePath_);
}

void ompl::tools::Thunder::printResultsInfo(std::ostream &out) const
{
    for (std::size_t i = 0; i < pdef_->getSolutionCount(); ++i)
    {
        out << OMPL_CONSOLE_COLOR_GREEN << "Solution " << i
            << " | Length: " << pdef_->getSolutions()[i].length_
            << " | Approximate: " << (pdef_->getSolutions()[i].approximate_ ? "true" : "false")
            << " | Planner: " << pdef_->getSolutions()[i].plannerName_
            << OMPL_CONSOLE_COLOR_RESET << std::endl;
    }
}

void ompl::tools::Thunder::print(std::ostream &out) const
{
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
    if (rrPlanner_)
    {
        rrPlanner_->printProperties(out);
        rrPlanner_->printSettings(out);
    }
    if (pdef_)
        pdef_->print(out);
}

void ompl::tools::Thunder::printLogs(std::ostream &out) const
{
    out << "Thunder Framework Logging Results" << std::endl;
    out << "  Solutions Attempted:                 " << logs_.numProblems_ << std::endl;
    out << "     Solved from scratch:              " << logs_.numSolutionsFromScratch_ << std::endl;
    out << "     Solved from recall:               " << logs_.numSolutionsFromRecall_ << std::endl;
    out << "        That were saved:               " << logs_.numSolutionsFromRecallSaved_ << std::endl;
    out << "        That were discarded:           " << logs_.numSolutionsFromRecall_ - logs_.numSolutionsFromRecallSaved_ << std::endl;
    out << "        Less than 2 states:            " << logs_.numSolutionsTooShort_ << std::endl;
    out << "     Failed:                           " << logs_.numSolutionsFailed_ << std::endl;
    out << "     Approximate:                      " << logs_.numSolutionsApproximate_ << std::endl;
    out << "  Total nodes in database:         " << experienceDB_->getExperiencesCount() << std::endl;
    out << "     Unsaved solutions:                " << experienceDB_->getNumUnsavedPaths() << std::endl;
    out << "  Average planning time:               " << logs_.getAveragePlanningTime() << std::endl;
}

void ompl::tools::Thunder::saveDataLog(std::ostream &out)
{
    // Export to file and clear the stream
    out << logs_.csvDataLogStream_.str();
    logs_.csvDataLogStream_.str("");
}

void ompl::tools::Thunder::enablePlanningFromRecall(bool enable)
{
    // Remember state
    recallEnabled_ = enable;

    // Flag the planners as possibly misconfigured
    configured_ = false;
}


void ompl::tools::Thunder::enablePlanningFromScratch(bool enable)
{
    // Remember state
    scratchEnabled_ = enable;

    // Flag the planners as possibly misconfigured
    configured_ = false;
}

std::size_t ompl::tools::Thunder::getExperiencesCount() const
{
    return experienceDB_->getExperiencesCount();
}

void ompl::tools::Thunder::getAllPlannerDatas(std::vector<ob::PlannerDataPtr> &plannerDatas) const
{
    experienceDB_->getAllPlannerDatas(plannerDatas);
}

void ompl::tools::Thunder::convertPlannerData(const ob::PlannerDataPtr plannerData, og::PathGeometric &path)
{
    // Convert the planner data verticies into a vector of states
    for (std::size_t i = 0; i < plannerData->numVertices(); ++i)
        path.append(plannerData->getVertex(i).getState());
}

bool ompl::tools::Thunder::reversePathIfNecessary(og::PathGeometric &path1, og::PathGeometric &path2)
{
    // Reverse path2 if it matches better
    const ob::State* s1 = path1.getState(0);
    const ob::State* s2 = path2.getState(0);
    const ob::State* g1 = path1.getState(path1.getStateCount()-1);
    const ob::State* g2 = path2.getState(path2.getStateCount()-1);

    double regularDistance  = si_->distance(s1,s2) + si_->distance(g1,g2);
    double reversedDistance = si_->distance(s1,g2) + si_->distance(s2,g1);

    // Check if path is reversed from normal [start->goal] direction
    if ( regularDistance > reversedDistance )
    {
        // needs to be reversed
        path2.reverse();
        return true;
    }

    return false;
}

ompl::tools::ExperienceDB2Ptr ompl::tools::Thunder::getExperienceDB()
{
    return experienceDB_;
}
