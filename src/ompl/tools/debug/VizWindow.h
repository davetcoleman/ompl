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
   Desc:   Visualizations tools for a single screen/window in an external visualizer
*/

#ifndef OMPL_TOOLS_DEBUG_VIZ_WINDOW_
#define OMPL_TOOLS_DEBUG_VIZ_WINDOW_

#include <ompl/util/ClassForward.h>
#include <ompl/base/State.h>
#include <ompl/util/Deprecation.h>

// C++
#include <vector>

namespace ompl
{
    namespace geometric
    {
        OMPL_CLASS_FORWARD(PathGeometric);
    }
}

namespace ompl
{
    namespace base
    {
        OMPL_CLASS_FORWARD(SpaceInformation);
    }
}

namespace ompl
{
    namespace tools
    {
        // Sizes
        enum VizSizes
        {
            XXXXSMALL = 1,
            XXXSMALL = 2,
            XXSMALL = 3,
            XSMALL = 4,
            SMALL = 5,
            MEDIUM = 6,
            LARGE = 7,
            XLARGE = 8,
            XXLARGE = 9,
            XXXLARGE = 10,
            XXXXLARGE = 11,
            VARIABLE_SIZE = 11,
            SMALL_TRANSLUCENT = 12,
            SCALE = 13,
            ROBOT = 14
        };

        // Colors that correspond with rviz_visual_tools
        enum VizColors
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

        /**
           @anchor VizWindow
           @par Short description
           Visualizations tools for a single screen/window in an external visualizer
        */

        /// @cond IGNORE
        OMPL_CLASS_FORWARD(VizWindow);
        /// @endcond

        /** \class ompl::tools::VizWindowPtr
            \brief A boost shared pointer wrapper for ompl::tools::VizWindow */

        class VizWindow
        {
        public:
            VizWindow(ompl::base::SpaceInformationPtr si)
                : si_(si)
            {
            }

            /** \brief Visualize a state during runtime, externally */
            virtual void state(const ompl::base::State *state, VizSizes type, VizColors color, double extraData,
                               ompl::base::SpaceInformationPtr si) = 0;

            /** \brief Visualize a state during runtime, externally */
            void state(const ompl::base::State *state, VizSizes size, VizColors color, double extraData = 0)
            {
                this->state(state, size, color, extraData, si_);
            }

            /** \brief Visualize multiple states during runtime, externally */
            virtual void states(std::vector<const ompl::base::State *> states,
                                std::vector<ompl::tools::VizColors> colors, ompl::tools::VizSizes size)
            {
            }

            /** \brief Visualize edge during runtime, externally */
            OMPL_DEPRECATED
            virtual void edge(const ompl::base::State *stateA, const ompl::base::State *stateB, double cost) = 0;

            /** \brief Visualize edge with a level during runtime, externally */
            virtual void edge(const ompl::base::State *stateA, const ompl::base::State *stateB,
                              ompl::tools::VizSizes size, ompl::tools::VizColors color) = 0;

            /** \brief Visualize multiple edges during runtime, externally */
            virtual void edges(const std::vector<const ompl::base::State *> stateAs,
                               const std::vector<const ompl::base::State *> stateBs,
                               std::vector<ompl::tools::VizColors> colors, ompl::tools::VizSizes size)
            {
            }

            /** \brief Visualize path during runtime, externally */
            virtual void path(ompl::geometric::PathGeometric *path, VizSizes type, VizColors vertexColor,
                              VizColors edgeColor) = 0;

            /** \brief Trigger visualizer to refresh/repaint/display all graphics */
            virtual void trigger(std::size_t queueSize = 0) = 0;
            void triggerEvery(std::size_t queueSize)
            {
                trigger(queueSize);
            }

            /** \brief Trigger visualizer to clear all graphics */
            virtual void deleteAllMarkers() = 0;

            /** \brief Pause process and only accept incoming ROS messages */
            virtual void spin() = 0;

            /** \brief Check if SIGINT has been called for shutdown */
            virtual bool shutdownRequested() = 0;

            /** \brief Getter for SpaceInformation */
            const ompl::base::SpaceInformationPtr &getSpaceInformation() const
            {
                return si_;
            }

            /** \brief Setter for SpaceInformation */
            void setSpaceInformation(ompl::base::SpaceInformationPtr si)
            {
                si_ = si;
            }

            /** \brief Wait for user feedback i.e. through a button or joystick */
            virtual void prompt(const std::string &msg) = 0;

            ompl::base::SpaceInformationPtr si_;

        };  // end of class VizWindow
    }       // namespace tools
}  // namespace ompl

#endif  // OMPL_TOOLS_VIZ_WINDOW_
