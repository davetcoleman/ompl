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

/* Author: Dave Coleman <dave@dav.ee>
   Desc:   Use an external program to visualize search
*/

#ifndef OMPL_TOOLS_DEBUG_VISUALIZER_
#define OMPL_TOOLS_DEBUG_VISUALIZER_

#include <ompl/base/Planner.h>
#include <ompl/geometric/PathGeometric.h>
#include <functional>

namespace ompl
{
namespace tools
{
/**
   @anchor Visualizer
   @par Short description
   Use an external program to visualize search
*/

/// @cond IGNORE
OMPL_CLASS_FORWARD(Visualizer);
/// @endcond

/** \class ompl::tools::VisualizerPtr
    \brief A boost shared pointer wrapper for ompl::tools::Visualizer */

// Sizes
enum sizes
{
    DELETE_ALL_MARKERS = 0,
    SMALL = 1,
    MEDIUM = 2,
    LARGE = 3,
    VARIABLE_SIZE = 4,
    SMALL_TRANSLUCENT = 5,
    SCALE = 7,
    ROBOT = 10
};

// Colors that correspond with rviz_visual_tools
enum colors
{
    BLACK = 0,
    BROWN = 1,
    BLUE = 2,
    CYAN = 3,
    GREY = 4,
    DARK_GREY = 5,
    GREEN = 6,
    LIME_GREEN = 7,
    MAGENTA = 8,
    ORANGE = 9,
    PURPLE = 10,
    RED = 11,
    PINK = 12,
    WHITE = 13,
    YELLOW = 14,
    TRANSLUCENT = 15,
    TRANSLUCENT_LIGHT = 16,
    TRANSLUCENT_DARK = 17,
    RAND = 18,
    CLEAR = 19,
    DEFAULT = 20  // i.e. 'do not change default color'
};

enum edgeColors
{
    eGREEN = 0,
    eUGLY_YELLOW = 25,
    eYELLOW = 50,
    eORANGE = 75,
    eRED = 100
};

/**
 * \brief Visualization callback hook for external debugging of states
 * \param v - state to visualize
 * \param type - different modes/colors/sizes for visualizing the state
 * \param extraData - special extra data for showing a range around a state
 */
typedef std::function<void(const ompl::base::State* v, sizes size, colors color, double extraData)> VizState;
/**
 * \brief Visualization callback hook for external debugging of edges
 * \param v1 - from state that marks beginning of edge
 * \param v2 - to state that marks end of edge
 * \param cost - signifies the weight of the edge using color
 */
typedef std::function<void(const ompl::base::State* v1, const ompl::base::State* v2, double cost)> VizEdge;

/**
 * \brief Visualization callback hook for external debugging of paths
 * \param path
 * \param type - style of line
 */
typedef std::function<void(ompl::geometric::PathGeometric* path, std::size_t type, colors color)> VizPath;

/**
 * \brief Visualization callback hook for external debugging that triggers the visualizer to render/publish
 */
typedef std::function<void(void)> VizTrigger;

/**
 * \brief Visualization callback hook for telling program to wait for user input
 */
typedef std::function<void(const std::string&)> VizWaitFeedback;

/**
 * \brief Visualization callback hook for external debugging that tells OMPL when to end early by SIG INT
 */
// typedef std::function<bool(void)> VizCheckSigInt;

/** \brief Use an external program to visualize search */
class Visualizer
{
  public:
    /** \brief Visualize planner's data during runtime, externally, using a function callback */
    void viz1State(const ompl::base::State* state, sizes type, colors color, double extraData)
    {
        if (viz1State_)
            viz1State_(state, type, color, extraData);
    }

    /** \brief Visualize planner's data during runtime, externally, using a function callback */
    void viz1Edge(const ompl::base::State* stateA, const ompl::base::State* stateB, double cost)
    {
        if (viz1Edge_)
            viz1Edge_(stateA, stateB, cost);
    }

    /** \brief Visualize planner's data during runtime, externally, using a function callback */
    void viz1Path(ompl::geometric::PathGeometric* path, std::size_t type, colors color)
    {
        if (viz1Path_)
            viz1Path_(path, type, color);
    }

    /** \brief Trigger visualizer to publish graphics */
    void viz1Trigger()
    {
        if (viz1Trigger_)
            viz1Trigger_();
    }

    /** \brief Trigger visualizer to publish graphics */
    void viz1DeleteAllMarkers()
    {
        if (viz1State_)
            viz1State_(nullptr, tools::DELETE_ALL_MARKERS, tools::DEFAULT, 0);  // black is dummy data
        // TODO: have an actual function to call instead of using vizState
    }

    /** \brief Set the callback to visualize/publish a planner's progress */
    void setViz1Callbacks(ompl::tools::VizState vizState, ompl::tools::VizEdge vizEdge, ompl::tools::VizPath vizPath,
                          ompl::tools::VizTrigger vizTrigger)
    {
        viz1State_ = vizState;
        viz1Edge_ = vizEdge;
        viz1Path_ = vizPath;
        viz1Trigger_ = vizTrigger;
    }

    // ------------------------------------------------------------------------------------

    /** \brief Visualize planner's data during runtime, externally, using a function callback */
    void viz2State(const ompl::base::State* state, sizes type, colors color, double extraData)
    {
        if (viz2State_)
            viz2State_(state, type, color, extraData);
    }

    /** \brief Visualize planner's data during runtime, externally, using a function callback */
    void viz2Edge(const ompl::base::State* stateA, const ompl::base::State* stateB, double cost)
    {
        if (viz2Edge_)
            viz2Edge_(stateA, stateB, cost);
    }

    /** \brief Visualize planner's data during runtime, externally, using a function callback */
    void viz2Path(ompl::geometric::PathGeometric* path, std::size_t type, colors color)
    {
        if (viz2Path_)
            viz2Path_(path, type, color);
    }

    /** \brief Trigger visualizer to publish graphics */
    void viz2Trigger()
    {
        if (viz2Trigger_)
            viz2Trigger_();
    }

    /** \brief Trigger visualizer to publish graphics */
    void viz2DeleteAllMarkers()
    {
        if (viz2State_)
            viz2State_(nullptr, tools::DELETE_ALL_MARKERS, tools::BLACK, 0);  // black is dummy data
        // TODO: have an actual function to call instead of using vizState
    }

    /** \brief Set the callback to visualize/publish a planner's progress */
    void setViz2Callbacks(ompl::tools::VizState vizState, ompl::tools::VizEdge vizEdge, ompl::tools::VizPath vizPath,
                          ompl::tools::VizTrigger vizTrigger)
    {
        viz2State_ = vizState;
        viz2Edge_ = vizEdge;
        viz2Path_ = vizPath;
        viz2Trigger_ = vizTrigger;
    }

    // ------------------------------------------------------------------------------------

    /** \brief Visualize planner's data during runtime, externally, using a function callback */
    void viz3State(const ompl::base::State* state, sizes type, colors color, double extraData)
    {
        if (viz3State_)
            viz3State_(state, type, color, extraData);
    }

    /** \brief Visualize planner's data during runtime, externally, using a function callback */
    void viz3Edge(const ompl::base::State* stateA, const ompl::base::State* stateB, double cost)
    {
        if (viz3Edge_)
            viz3Edge_(stateA, stateB, cost);
    }

    /** \brief Visualize planner's data during runtime, externally, using a function callback */
    void viz3Path(ompl::geometric::PathGeometric* path, std::size_t type, colors color)
    {
        if (viz3Path_)
            viz3Path_(path, type, color);
    }

    /** \brief Trigger visualizer to publish graphics */
    void viz3Trigger()
    {
        if (viz3Trigger_)
            viz3Trigger_();
    }

    /** \brief Trigger visualizer to publish graphics */
    void viz3DeleteAllMarkers()
    {
        if (viz3State_)
            viz3State_(nullptr, tools::DELETE_ALL_MARKERS, tools::BLACK, 0);  // black is dummy data
        // TODO: have an actual function to call instead of using vizState
    }

    /** \brief Set the callback to visualize/publish a planner's progress */
    void setViz3Callbacks(ompl::tools::VizState vizState, ompl::tools::VizEdge vizEdge, ompl::tools::VizPath vizPath,
                          ompl::tools::VizTrigger vizTrigger)
    {
        viz3State_ = vizState;
        viz3Edge_ = vizEdge;
        viz3Path_ = vizPath;
        viz3Trigger_ = vizTrigger;
    }

    // ------------------------------------------------------------------------------------

    /** \brief Visualize planner's data during runtime, externally, using a function callback */
    void viz4State(const ompl::base::State* state, sizes type, colors color, double extraData)
    {
        if (viz4State_)
            viz4State_(state, type, color, extraData);
    }

    /** \brief Visualize planner's data during runtime, externally, using a function callback */
    void viz4Edge(const ompl::base::State* stateA, const ompl::base::State* stateB, double cost)
    {
        if (viz4Edge_)
            viz4Edge_(stateA, stateB, cost);
    }

    /** \brief Visualize planner's data during runtime, externally, using a function callback */
    void viz4Path(ompl::geometric::PathGeometric* path, std::size_t type, colors color)
    {
        if (viz4Path_)
            viz4Path_(path, type, color);
    }

    /** \brief Trigger visualizer to publish graphics */
    void viz4Trigger()
    {
        if (viz4Trigger_)
            viz4Trigger_();
    }

    /** \brief Trigger visualizer to publish graphics */
    void viz4DeleteAllMarkers()
    {
        if (viz4State_)
            viz4State_(nullptr, tools::DELETE_ALL_MARKERS, tools::BLACK, 0);  // black is dummy data
        // TODO: have an actual function to call instead of using vizState
    }

    /** \brief Set the callback to visualize/publish a planner's progress */
    void setViz4Callbacks(ompl::tools::VizState vizState, ompl::tools::VizEdge vizEdge, ompl::tools::VizPath vizPath,
                          ompl::tools::VizTrigger vizTrigger)
    {
        viz4State_ = vizState;
        viz4Edge_ = vizEdge;
        viz4Path_ = vizPath;
        viz4Trigger_ = vizTrigger;
    }

    // ------------------------------------------------------------------------------------

    /** \brief Visualize planner's data during runtime, externally, using a function callback */
    void viz5State(const ompl::base::State* state, sizes type, colors color, double extraData)
    {
        if (viz5State_)
            viz5State_(state, type, color, extraData);
    }

    /** \brief Visualize planner's data during runtime, externally, using a function callback */
    void viz5Path(ompl::geometric::PathGeometric* path, std::size_t type, colors color)
    {
        if (viz5Path_)
            viz5Path_(path, type, color);
    }

    /** \brief Visualize planner's data during runtime, externally, using a function callback */
    void viz5Edge(const ompl::base::State* stateA, const ompl::base::State* stateB, double cost)
    {
        if (viz5Edge_)
            viz5Edge_(stateA, stateB, cost);
    }

    /** \brief Trigger visualizer to publish graphics */
    void viz5Trigger()
    {
        if (viz5Trigger_)
            viz5Trigger_();
    }

    /** \brief Trigger visualizer to publish graphics */
    void viz5DeleteAllMarkers()
    {
        if (viz5State_)
            viz5State_(nullptr, tools::DELETE_ALL_MARKERS, tools::BLACK, 0);  // black is dummy data
        // TODO: have an actual function to call instead of using vizState
    }

    /** \brief Set the callback to visualize/publish a planner's progress */
    void setViz5Callbacks(ompl::tools::VizState vizState, ompl::tools::VizEdge vizEdge, ompl::tools::VizPath vizPath,
                          ompl::tools::VizTrigger vizTrigger)
    {
        viz5State_ = vizState;
        viz5Edge_ = vizEdge;
        viz5Path_ = vizPath;
        viz5Trigger_ = vizTrigger;
    }

    // ------------------------------------------------------------------------------------

    /** \brief Visualize planner's data during runtime, externally, using a function callback */
    void viz6State(const ompl::base::State* state, sizes type, colors color, double extraData)
    {
        if (viz6State_)
            viz6State_(state, type, color, extraData);
    }

    /** \brief Visualize planner's data during runtime, externally, using a function callback */
    void viz6Path(ompl::geometric::PathGeometric* path, std::size_t type, colors color)
    {
        if (viz6Path_)
            viz6Path_(path, type, color);
    }

    /** \brief Visualize planner's data during runtime, externally, using a function callback */
    void viz6Edge(const ompl::base::State* stateA, const ompl::base::State* stateB, double cost)
    {
        if (viz6Edge_)
            viz6Edge_(stateA, stateB, cost);
    }

    /** \brief Trigger visualizer to publish graphics */
    void viz6Trigger()
    {
        if (viz6Trigger_)
            viz6Trigger_();
    }

    /** \brief Trigger visualizer to publish graphics */
    void viz6DeleteAllMarkers()
    {
        if (viz6State_)
            viz6State_(nullptr, tools::DELETE_ALL_MARKERS, tools::BLACK, 0);  // black is dummy data
        // TODO: have an actual function to call instead of using vizState
    }

    /** \brief Set the callback to visualize/publish a planner's progress */
    void setViz6Callbacks(ompl::tools::VizState vizState, ompl::tools::VizEdge vizEdge, ompl::tools::VizPath vizPath,
                          ompl::tools::VizTrigger vizTrigger)
    {
        viz6State_ = vizState;
        viz6Edge_ = vizEdge;
        viz6Path_ = vizPath;
        viz6Trigger_ = vizTrigger;
    }

    // ------------------------------------------------------------------------------------

    /** \brief Wait for user feedback i.e. through a button or joystick */
    void waitForUserFeedback(const std::string& msg)
    {
        if (waitForUserFeedback_)
            waitForUserFeedback_(msg);
    }

    /** \brief Set the callback to wait for user feedback */
    void setWaitForUserFeedback(ompl::tools::VizWaitFeedback waitForUserFeedback)
    {
        waitForUserFeedback_ = waitForUserFeedback;
    }

    // ------------------------------------------------------------------------------------

    /** \brief Callback to display the node covereage in 2D */
    void vizVoronoiDiagram()
    {
        if (vizVoronoiDiagram_)
            vizVoronoiDiagram_();
    }

    /** \brief Set the callback to wait for user feedback */
    void setVizVoronoiDiagram(ompl::tools::VizTrigger vizVoronoiDiagram)
    {
        vizVoronoiDiagram_ = vizVoronoiDiagram;
    }

    // ------------------------------------------------------------------------------------

    /** \brief State callback by vizID */
    void vizState(const std::size_t vizID, const ompl::base::State* state, sizes type, colors color, double extraData)
    {
        switch (vizID)
        {
            case 1:
                viz1State(state, type, color, extraData);
                break;
            case 2:
                viz2State(state, type, color, extraData);
                break;
            case 3:
                viz3State(state, type, color, extraData);
                break;
            case 4:
                viz4State(state, type, color, extraData);
                break;
            case 5:
                viz5State(state, type, color, extraData);
                break;
            case 6:
                viz6State(state, type, color, extraData);
                break;
            default:
                OMPL_ERROR("Incorrect visualizer ID specified: %u", vizID);
        }
    }

    /** \brief Trigger visualizer to publish graphics by vizID */
    void vizTrigger(const std::size_t vizID)
    {
        switch (vizID)
        {
            case 1:
                viz1Trigger();
                break;
            case 2:
                viz2Trigger();
                break;
            case 3:
                viz3Trigger();
                break;
            case 4:
                viz4Trigger();
                break;
            case 5:
                viz5Trigger();
                break;
            case 6:
                viz6Trigger();
                break;
            default:
                OMPL_ERROR("Incorrect visualizer ID specified: %u", vizID);
        }
    }

  private:
    /** \brief Optional st callbacks for data */
    VizState viz1State_;
    VizEdge viz1Edge_;
    VizPath viz1Path_;
    VizTrigger viz1Trigger_;

    /** \brief Optional 2nd callbacks for data */
    VizState viz2State_;
    VizEdge viz2Edge_;
    VizPath viz2Path_;
    VizTrigger viz2Trigger_;

    /** \brief Optional 3rd callbacks for data */
    VizState viz3State_;
    VizEdge viz3Edge_;
    VizPath viz3Path_;
    VizTrigger viz3Trigger_;

    /** \brief Optional 4th callbacks for data */
    VizState viz4State_;
    VizEdge viz4Edge_;
    VizPath viz4Path_;
    VizTrigger viz4Trigger_;

    /** \brief Optional 5th callbacks for data */
    VizState viz5State_;
    VizEdge viz5Edge_;
    VizPath viz5Path_;
    VizTrigger viz5Trigger_;

    /** \brief Optional 6th callbacks for data */
    VizState viz6State_;
    VizEdge viz6Edge_;
    VizPath viz6Path_;
    VizTrigger viz6Trigger_;

    /** \brief Callback to wait for user input before proceeding */
    VizWaitFeedback waitForUserFeedback_;

    /** \brief Callback to display the node covereage in 2D */
    VizTrigger vizVoronoiDiagram_;

};  // end of class Visualizer
}  // namespace tools
}  // namespace ompl

#endif  // OMPL_TOOLS_DEBUG_VISUALIZER_
