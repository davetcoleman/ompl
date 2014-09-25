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


#include "ompl/tools/lightning/Lightning.h"
//#include "ompl/base/goals/GoalSampleableRegion.h"
//#include "ompl/geometric/PathGeometric.h"
//#include "ompl/base/StateSpace.h" // for storing to file
#include "ompl/tools/lightning/ExperienceDB.h"

namespace og = ompl::geometric;
namespace ob = ompl::base;
namespace ot = ompl::tools;

ompl::tools::Lightning::Lightning(const base::SpaceInformationPtr &si) :
    ompl::tools::ExperienceSetup(si)
{
    initialize();
}

ompl::tools::Lightning::Lightning(const base::StateSpacePtr &space) :
    ompl::tools::ExperienceSetup(space)
{
    initialize();
}

void ompl::tools::Lightning::initialize()
{
    std::cout << OMPL_CONSOLE_COLOR_CYAN << std::endl;
    std::cout << std::endl;
    std::cout << "Initializing Lightning Framework ///////////////////////////////////////" << std::endl;
    std::cout << OMPL_CONSOLE_COLOR_RESET;

    recallEnabled_ = true;
    scratchEnabled_ = true;
    filePath_ = "unloaded";

    // Load dynamic time warp
    dtw_.reset(new ot::DynamicTimeWarp(si_));

    // Load the experience database
    experienceDB_.reset(new ompl::tools::ExperienceDB(si_->getStateSpace()));

    // Load the Retrieve repair database. We do it here so that setRepairPlanner() works
    rrPlanner_ = ob::PlannerPtr(new og::RetrieveRepair(si_, experienceDB_));

    OMPL_INFORM("Lightning Framework initialized.");
    std::cout << std::endl;
    std::cout << std::endl;
}

bool ompl::tools::Lightning::setFile(const std::string &databaseName, const std::string &databaseDirectory)
{
    std::string fileName = "lightning_" + databaseName;
    return ompl::tools::ExperienceSetup::getFilePath(fileName, databaseDirectory);
}

void ompl::tools::Lightning::setup(void)
{
    if (!configured_ || !si_->isSetup() || !planner_->isSetup() || !rrPlanner_->isSetup() )
    {
        OMPL_ERROR("Setting up Lightning Framework");

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


        // Check if experience database is already loaded
        if (experienceDB_->isEmpty())
          experienceDB_->load(filePath_); // load from file
        else
          OMPL_ERROR("Attempting to load experience database when it is not empty");

        // Set the configured flag
        configured_ = true;
    }
}

void ompl::tools::Lightning::clear(void)
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
ompl::base::PlannerStatus ompl::tools::Lightning::solve(const base::PlannerTerminationCondition &ptc)
{
    OMPL_INFORM("Lightning Framework: Starting solve()");

    // Setup again in case it has not been done yet
    setup();

    lastStatus_ = base::PlannerStatus::UNKNOWN;
    time::point start = time::now();

    // Insertion time
    double insertionTime;

    // Start both threads
    bool hybridize = false;
    lastStatus_ = pp_->solve(ptc, hybridize);

    // Planning time
    planTime_ = time::seconds(time::now() - start);
    logs_.totalPlanningTime_ += planTime_; // used for averaging
    logs_.numProblems_++; // used for averaging
    logs_.csvDataLogStream_ << planTime_ << ",";

    if (lastStatus_ == ompl::base::PlannerStatus::TIMEOUT)
    {
        // Skip further processing if absolutely no path is available        
        OMPL_ERROR("Lightning Solve: No solution found after %f seconds", planTime_);
        logs_.numSolutionsTimedout_++;
        logs_.csvDataLogStream_ << planTime_ << ",neither_planner,timedout,not_saved,0";
    }
    else if (!lastStatus_)
    {
        // Skip further processing if absolutely no path is available      
        OMPL_ERROR("Lightning Solve: Unknown failure");
        std::cout << "UNKNOWN--------------------------------------------- " << std::endl;
        std::cout << "UNKNOWN--------------------------------------------- " << std::endl;
        std::cout << "UNKNOWN--------------------------------------------- " << std::endl;
        std::cout << "UNKNOWN--------------------------------------------- " << std::endl;
        std::cout << "UNKNOWN--------------------------------------------- " << std::endl;
        std::cout << "UNKNOWN--------------------------------------------- " << std::endl;
        std::cout << "UNKNOWN--------------------------------------------- " << std::endl;
        std::cout << "UNKNOWN--------------------------------------------- " << std::endl;
        std::cout << "UNKNOWN--------------------------------------------- " << std::endl;
        std::cout << "Planner Status: " << lastStatus_ << std::endl;
        logs_.numSolutionsFailed_++;
        logs_.csvDataLogStream_ << "0,neither_planner,failed,not_saved,0";
    }
    else
    {
        OMPL_INFORM("Lightning Solve: Possible solution found in %f seconds", planTime_);


        // Smooth the result
        //OMPL_INFORM("Simplifying solution (smoothing)...");
        //simplifySolution(ptc);

        std::cout << "before get solution path " << std::endl;
        // Get information about the exploration data structure the motion planner used. Used later in visualizing
        og::PathGeometric solutionPath = getSolutionPath(); // copied so that it is non-const
        std::cout << "after get solution path " << std::endl;
        OMPL_INFORM("Solution path has %d states and was generated from planner %s", solutionPath.getStateCount(), getSolutionPlannerName().c_str());
        logs_.csvDataLogStream_ << solutionPath.getStateCount() << "," << getSolutionPlannerName() << ",";

        //solutionPath.print(std::cout);

        std::cout << OMPL_CONSOLE_COLOR_CYAN << "LIGHTNING RESULTS:" << OMPL_CONSOLE_COLOR_RESET << std::endl;

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
                // Benchmark runtime
                time::point startTime = time::now();

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
                    double dummyInsertionTime; // unused because does not include scoring function
                    experienceDB_->addPath(solutionPath, dummyInsertionTime);
                }
                insertionTime += time::seconds(time::now() - startTime);
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
                experienceDB_->addPath(solutionPath, insertionTime);
            }
        }
    }

    logs_.totalInsertionTime_ += insertionTime; // used for averaging

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
        << logs_.getAveragePlanningTime()  << ","
        << logs_.getAverageInsertionTime()
        << std::endl;

    std::cout << "Leaving Lightning::solve() fn " << std::endl;
    return lastStatus_;
}

ompl::base::PlannerStatus ompl::tools::Lightning::solve(double time)
{
    ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition( time );
    return solve(ptc);
}

bool ompl::tools::Lightning::save()
{
    return experienceDB_->save(filePath_);
}

bool ompl::tools::Lightning::saveIfChanged()
{
    return experienceDB_->saveIfChanged(filePath_);
}

void ompl::tools::Lightning::printResultsInfo(std::ostream &out) const
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

void ompl::tools::Lightning::print(std::ostream &out) const
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

void ompl::tools::Lightning::printLogs(std::ostream &out) const
{
    out << "Lightning Framework Logging Results" << std::endl;
    out << "  Solutions Attempted:           " << logs_.numProblems_ << std::endl;
    out << "     Solved from scratch:        " << logs_.numSolutionsFromScratch_ << " (" << logs_.numSolutionsFromScratch_/logs_.numProblems_*100 << "%)" << std::endl;
    out << "     Solved from recall:         " << logs_.numSolutionsFromRecall_  << " (" << logs_.numSolutionsFromRecall_/logs_.numProblems_*100 << "%)" << std::endl;
    out << "        That were saved:         " << logs_.numSolutionsFromRecallSaved_ << std::endl;
    out << "        That were discarded:     " << logs_.numSolutionsFromRecall_ - logs_.numSolutionsFromRecallSaved_ << std::endl;
    out << "        Less than 2 states:      " << logs_.numSolutionsTooShort_ << std::endl;
    out << "     Failed:                     " << logs_.numSolutionsFailed_ << std::endl;
    out << "     Timedout:                   " << logs_.numSolutionsTimedout_ << std::endl;
    out << "     Approximate:                " << logs_.numSolutionsApproximate_ << std::endl;
    out << "  LightningDb                    " << std::endl;
    out << "     Total paths:                " << experienceDB_->getExperiencesCount() << std::endl;
    out << "     Vertices (states):          " << experienceDB_->getExperiencesCount() << std::endl;
    out << "     Unsaved solutions:          " << experienceDB_->getNumUnsavedPaths() << std::endl;
    out << "  Average planning time:         " << logs_.getAveragePlanningTime() << std::endl;
    out << "  Average insertion time:        " << logs_.getAverageInsertionTime() << std::endl;
}

void ompl::tools::Lightning::saveDataLog(std::ostream &out)
{
    // Export to file and clear the stream
    out << logs_.csvDataLogStream_.str();
    logs_.csvDataLogStream_.str("");
}

std::size_t ompl::tools::Lightning::getExperiencesCount() const
{
    return experienceDB_->getExperiencesCount();
}

void ompl::tools::Lightning::getAllPlannerDatas(std::vector<ob::PlannerDataPtr> &plannerDatas) const
{
    experienceDB_->getAllPlannerDatas(plannerDatas);
}

void ompl::tools::Lightning::convertPlannerData(const ob::PlannerDataPtr plannerData, og::PathGeometric &path)
{
    // Convert the planner data verticies into a vector of states
    for (std::size_t i = 0; i < plannerData->numVertices(); ++i)
        path.append(plannerData->getVertex(i).getState());
}

bool ompl::tools::Lightning::reversePathIfNecessary(og::PathGeometric &path1, og::PathGeometric &path2)
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
