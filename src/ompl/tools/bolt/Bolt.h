/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, University of Colorado, Boulder
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

/* Author: Dave Coleman
*/

#ifndef OMPL_TOOLS_BOLT_BOLT_
#define OMPL_TOOLS_BOLT_BOLT_

#include <ompl/tools/experience/ExperienceSetup.h>  // the parent class

#include <ompl/tools/bolt/DenseDB.h>
#include <ompl/tools/bolt/Visualizer.h>
#include <ompl/tools/bolt/Discretizer.h>
#include <ompl/geometric/planners/experience/BoltRetrieveRepair.h>

#include <ompl/base/Planner.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/StateSpace.h>  // for storing to file

#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>

#include <ompl/util/Console.h>
#include <ompl/util/Exception.h>

namespace ompl
{
namespace tools
{
namespace bolt
{
/**
   @anchor Bolt
   @par Short description
   Bolt is an experience-based planning framework that learns to reduce computation time
   required to solve high-dimensional planning problems in varying environments.
*/

/// @cond IGNORE
OMPL_CLASS_FORWARD(Bolt);
/// @endcond

/** \class BoltPtr
    \brief A boost shared pointer wrapper for Bolt */

/** \brief Built off of SimpleSetup but provides support for planning from experience */
class Bolt : public tools::ExperienceSetup
{
  public:
    /** \brief Constructor needs the state space used for planning. */
    explicit Bolt(const base::SpaceInformationPtr &si);

    /** \brief Constructor needs the state space used for planning.
     *  \param space - the state space to plan in
     */
    explicit Bolt(const base::StateSpacePtr &space);

  private:
    /** \brief Shared constructor functions */
    void initialize();

  public:
    /** \brief Display debug data about potential available solutions */
    void printResultsInfo(std::ostream &out = std::cout) const;

    /** \brief Display debug data about overall results from Bolt since being loaded */
    void printLogs(std::ostream &out = std::cout) const;

    /** \brief Load database from file or, if unavailable, create new discretization */
    bool loadOrGenerate();

    /** \brief Get the current planner */
    base::PlannerPtr &getPlanner()
    {
        return planner_;
    }

    /** \brief Get a pointer to the retrieve repair planner */
    BoltRetrieveRepair &getRetrieveRepairPlanner() const
    {
        return static_cast<BoltRetrieveRepair &>(*boltPlanner_);
    }

    /** \brief Set the planner to use for repairing experience paths
        inside the BoltRetrieveRepair planner. If the planner is not
        set, a default planner is set. */
    void setRepairPlanner(const base::PlannerPtr &planner)
    {
        // This is required by the parent class but we no longer use this feature
        // static_cast<BoltRetrieveRepair&>(*boltPlanner_).setRepairPlanner(planner);
    }

    /** \brief Set the planner allocator to use. This is only
        used if no planner has been set. This is optional -- a default
        planner will be used if no planner is otherwise specified. */
    void setPlannerAllocator(const base::PlannerAllocator &pa);

    /** \brief Run the planner for up to a specified amount of time (default is 1 second) */
    virtual base::PlannerStatus solve(double time = 1.0);

    /** \brief Helper function for logging data to file */
    void logResults();

    /** \brief Run the planner until \e ptc becomes true (at most) */
    virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

    /** \brief Sanity check for solution paths */
    bool checkRepeatedStates(const geometric::PathGeometric& path);

    /** \brief Save the experience database to file */
    bool save();

    /** \brief Save the experience database to file if there has been a change */
    bool saveIfChanged();

    /** \brief Clear all planning data. This only includes
        data generated by motion plan computation. Planner
        settings, start & goal states are not affected. */
    virtual void clear(void);

    /** \brief Print information about the current setup */
    virtual void print(std::ostream &out = std::cout) const;

    /** \brief This method will create the necessary classes
        for planning. The solve() method will call this function automatically. */
    virtual void setup(void);

    /** \brief Get a vector of all the planning data in the database */
    void getAllPlannerDatas(std::vector<base::PlannerDataPtr> &plannerDatas) const;

    /** \brief Get the total number of paths stored in the database */
    std::size_t getExperiencesCount() const;

    /** \brief Convert PlannerData to PathGeometric. Assume ordering of verticies is order of path */
    void convertPlannerData(const base::PlannerDataPtr plannerData, geometric::PathGeometric &path);

    /** \brief Hook for getting access to dense db */
    DenseDBPtr getDenseDB();

    /** \brief Hook for getting access to discretizer */
    DiscretizerPtr getDiscretizer()
    {
        return discretizer_;
    }

    /** \brief Allow accumlated experiences to be processed */
    bool doPostProcessing();

    /** \brief Get class for managing various visualization features */
    base::VisualizerPtr getVisual()
    {
        return visual_;
    }

  protected:
    /** \brief Class for managing various visualization features */
    base::VisualizerPtr visual_;

    /**  The maintained experience planner instance */
    base::PlannerPtr boltPlanner_;

    /** \brief A shared object between all the planners for saving and loading previous experience */
    DenseDBPtr denseDB_;

    /** \brief Tool for gridding state space */
    DiscretizerPtr discretizer_;

    /** \brief Accumulated experiences to be later added to experience database */
    std::vector<geometric::PathGeometric> queuedSolutionPaths_;

};  // end of class Bolt

}  // namespace bolt
}  // namespace tools
}  // namespace ompl
#endif
