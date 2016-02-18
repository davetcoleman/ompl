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

#include <ompl/tools/bolt/Bolt.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/PlannerStatus.h>
#include <ompl/util/Console.h>

namespace og = ompl::geometric;
namespace ob = ompl::base;
namespace ot = ompl::tools;

namespace ompl
{
namespace tools
{

Bolt::Bolt(const base::SpaceInformationPtr &si)
    : ExperienceSetup(si)
{
    initialize();
}

Bolt::Bolt(const base::StateSpacePtr &space)
    : ExperienceSetup(space)
{
    initialize();
}

void Bolt::initialize()
{
    OMPL_INFORM("Initializing Bolt Framework");

    recallEnabled_ = true;
    scratchEnabled_ = true;
    filePath_ = "unloaded";

    // Load the experience database
    boltDB_.reset(new geometric::BoltDB(si_));

    // Load the Retrieve repair database. We do it here so that setRepairPlanner() works
    boltPlanner_ = ob::PlannerPtr(new og::BoltRetrieveRepair(si_, boltDB_));

    OMPL_INFORM("Bolt Framework initialized.");
}

void Bolt::setup(void)
{
    if (!configured_ || !si_->isSetup() || !boltPlanner_->isSetup() )
    {
        // Setup Space Information if we haven't already done so
        if (!si_->isSetup())
            si_->setup();

        // Setup planning from experience planner
        boltPlanner_->setProblemDefinition(pdef_);

        if (!boltPlanner_->isSetup())
            boltPlanner_->setup();

        // Load from file or generate new grid
        if (!boltDB_->getNumVertices())
        {
            if (!boltDB_->load(filePath_)) // load from file
            {
                OMPL_INFORM("No database loaded from file - generating new grid");
                boltDB_->generateGrid();
            }
        }

        // Set the configured flag
        configured_ = true;
    }
}

void Bolt::clear(void)
{
    if (boltPlanner_)
        boltPlanner_->clear();
    if (pdef_)
        pdef_->clearSolutionPaths();
}

void Bolt::setPlannerAllocator(const base::PlannerAllocator &pa)
{
    pa_ = pa;
    // note: the boltPlanner_ never uses the allocator so does not need to be reset
    configured_ = false;
}

base::PlannerStatus Bolt::solve(const base::PlannerTerminationCondition &ptc)
{
    // we provide a duplicate implementation here to allow the planner to choose how the time is turned into a planner termination condition

    OMPL_INFORM("Bolt::solve()");

    // Setup again in case it has not been done yet
    setup();

    lastStatus_ = base::PlannerStatus::UNKNOWN;
    time::point start = time::now();

    // Warn if there are queued paths that have not been added to the experience database
    if (!queuedSolutionPaths_.empty())
    {
        OMPL_WARN("Previous solved paths are currently uninserted into the experience database and are in the post-proccessing queue");
    }

    // SOLVE
    lastStatus_ = boltPlanner_->solve(ptc);

    // Planning time
    planTime_ = time::seconds(time::now() - start);

    // Create log
    ExperienceLog log;
    log.planningTime = planTime_;

    // Record stats
    stats_.totalPlanningTime_ += planTime_; // used for averaging
    stats_.numProblems_++; // used for averaging

    if (lastStatus_ == base::PlannerStatus::TIMEOUT)
    {
        // Skip further processing if absolutely no path is available
        OMPL_ERROR("Bolt Solve: No solution found after %f seconds", planTime_);

        stats_.numSolutionsTimedout_++;

        // Logging
        log.planner = "neither_planner";
        log.result = "timedout";
        log.isSaved = "not_saved";
    }
    else if (!lastStatus_)
    {
        // Skip further processing if absolutely no path is available
        OMPL_ERROR("Bolt Solve: Unknown failure");
        stats_.numSolutionsFailed_++;

        // Logging
        log.planner = "neither_planner";
        log.result = "failed";
        log.isSaved = "not_saved";
    }
    else
    {
        OMPL_INFORM("Bolt Solve: Possible solution found in %f seconds", planTime_);

        // Smooth the result
        simplifySolution(ptc);

        og::PathGeometric solutionPath = og::SimpleSetup::getSolutionPath(); // copied so that it is non-const
        OMPL_INFORM("Solution path has %d states and was generated from planner %s", solutionPath.getStateCount(), getSolutionPlannerName().c_str());

        // Do not save if approximate
        if (!haveExactSolutionPath())
        {
            OMPL_ERROR("BOLT RESULTS: Approximate - how did this happen??");
            exit(-1);
        }

        // Stats
        stats_.numSolutionsFromRecall_++;

        // Make sure solution has at least 2 states
        if (solutionPath.getStateCount() < 2)
        {
            OMPL_INFORM("NOT saving to database because solution is less than 2 states long");
            stats_.numSolutionsTooShort_++;

            // Logging
            log.isSaved = "less_2_states";
            log.tooShort = true;
        }
        else
        {
            OMPL_INFORM("Adding path frequency to database");

            // Queue the solution path for future insertion into experience database (post-processing)
            queuedSolutionPaths_.push_back(solutionPath);
        }
    }

    // Final log data
    //log.insertion_time = insertionTime; TODO fix this
    log.numVertices = boltDB_->getNumVertices();
    log.numEdges = boltDB_->getNumEdges();
    log.numConnectedComponents = 0;

    // Flush the log to buffer
    convertLogToString(log);

    return lastStatus_;
}

base::PlannerStatus Bolt::solve(double time)
{
    ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition( time );
    return solve(ptc);
}

bool Bolt::save()
{
    setup(); // ensure the db has been loaded to the Experience DB
    return boltDB_->save(filePath_);
}

bool Bolt::saveIfChanged()
{
    setup(); // ensure the db has been loaded to the Experience DB
    return boltDB_->saveIfChanged(filePath_);
}

void Bolt::printResultsInfo(std::ostream &out) const
{
    for (std::size_t i = 0; i < pdef_->getSolutionCount(); ++i)
    {
        out << "Solution " << i
            << "\t | Length: " << pdef_->getSolutions()[i].length_
            << "\t | Approximate: " << (pdef_->getSolutions()[i].approximate_ ? "true" : "false")
            << "\t | Planner: " << pdef_->getSolutions()[i].plannerName_ << std::endl;
    }
}

void Bolt::print(std::ostream &out) const
{
    if (si_)
    {
        si_->printProperties(out);
        si_->printSettings(out);
    }
    if (boltPlanner_)
    {
        boltPlanner_->printProperties(out);
        boltPlanner_->printSettings(out);
    }
    if (pdef_)
        pdef_->print(out);
}

void Bolt::printLogs(std::ostream &out) const
{
    if (!recallEnabled_)
        out << "Scratch Planning Logging Results (inside Bolt Framework)" << std::endl;
    else
        out << "Bolt Framework Logging Results" << std::endl;
    out << "  Solutions Attempted:           " << stats_.numProblems_ << std::endl;
    out << "    Solved from scratch:        " << stats_.numSolutionsFromScratch_ << " (" << stats_.numSolutionsFromScratch_/stats_.numProblems_*100 << "%)" << std::endl;
    out << "    Solved from recall:         " << stats_.numSolutionsFromRecall_  << " (" << stats_.numSolutionsFromRecall_/stats_.numProblems_*100 << "%)" << std::endl;
    out << "      That were saved:         " << stats_.numSolutionsFromRecallSaved_ << std::endl;
    out << "      That were discarded:     " << stats_.numSolutionsFromRecall_ - stats_.numSolutionsFromRecallSaved_ << std::endl;
    out << "      Less than 2 states:      " << stats_.numSolutionsTooShort_ << std::endl;
    out << "    Failed:                     " << stats_.numSolutionsFailed_ << std::endl;
    out << "    Timedout:                    " << stats_.numSolutionsTimedout_ << std::endl;
    out << "    Approximate:                 " << stats_.numSolutionsApproximate_ << std::endl;
    out << "  BoltDB                        " << std::endl;
    out << "    Vertices:                    " << boltDB_->getNumVertices() << std::endl;
    out << "    Edges:                       " << boltDB_->getNumEdges() << std::endl;
    //out << "    Consecutive state failures:  " << boltDB_->getNumConsecutiveFailures() << std::endl;
    //out << "    Connected path failures:     " << boltDB_->getNumPathInsertionFailed() << std::endl;
    //out << "    Sparse Delta Fraction:       " << boltDB_->getSparseDeltaFraction() << std::endl;
    out << "  Average planning time:         " << stats_.getAveragePlanningTime() << std::endl;
    out << "  Average insertion time:        " << stats_.getAverageInsertionTime() << std::endl;
}

std::size_t Bolt::getExperiencesCount() const
{
    return boltDB_->getNumVertices();
}

void Bolt::getAllPlannerDatas(std::vector<ob::PlannerDataPtr> &plannerDatas) const
{
    boltDB_->getAllPlannerDatas(plannerDatas);
}

void Bolt::convertPlannerData(const ob::PlannerDataPtr plannerData, og::PathGeometric &path)
{
    // Convert the planner data verticies into a vector of states
    for (std::size_t i = 0; i < plannerData->numVertices(); ++i)
        path.append(plannerData->getVertex(i).getState());
}

bool Bolt::reversePathIfNecessary(og::PathGeometric &path1, og::PathGeometric &path2)
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

geometric::BoltDBPtr Bolt::getExperienceDB()
{
    return boltDB_;
}

bool Bolt::doPostProcessing()
{
    OMPL_WARN("Post processing does not work yet, perhaps unsolvable in current form");
    queuedSolutionPaths_.clear();
    return true;

    OMPL_INFORM("Performing post-processing for %i queued solution paths", queuedSolutionPaths_.size());

    for (std::size_t i = 0; i < queuedSolutionPaths_.size(); ++i)
    {
        // Time to add a path to experience database
        double insertionTime;

        if (!boltDB_->postProcessesPath(queuedSolutionPaths_[i], insertionTime))
        {
            OMPL_ERROR("Unable to save path");
        }

        OMPL_INFORM("Finished inserting experience path in %f seconds", insertionTime);
        stats_.totalInsertionTime_ += insertionTime; // used for averaging
    }

    // Remove all inserted paths from the queue
    queuedSolutionPaths_.clear();

    // Ensure graph doesn't get too popular
    boltDB_->normalizeGraphEdgeWeights();

    return true;
}

} // namespace tools
} // namespace ompl
