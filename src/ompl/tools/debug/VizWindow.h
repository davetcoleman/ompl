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

#include <ompl/geometric/PathGeometric.h>

namespace ompl
{
    namespace tools
    {
        // Sizes
        enum VizSizes
        {
            DELETE_ALL_MARKERS = 0,  // DEPRECATED, DO NOT USE
            SMALL = 1,
            MEDIUM = 2,
            LARGE = 3,
            VARIABLE_SIZE = 4,
            SMALL_TRANSLUCENT = 5,
            SCALE = 7,
            ROBOT = 10
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
            VizWindow()
            {}

            /** \brief Visualize a state during runtime, externally */
            virtual void state(const ompl::base::State *state, VizSizes type, VizColors color, double extraData) = 0;

            /** \brief Visualize edge during runtime, externally */
            virtual void edge(const ompl::base::State *stateA, const ompl::base::State *stateB, double cost) = 0;

            /** \brief Visualize edge with a level during runtime, externally */
            virtual void edge(const ompl::base::State* stateA, const ompl::base::State* stateB,
                ompl::tools::VizSizes size, ompl::tools::VizColors color) = 0;

            /** \brief Visualize path during runtime, externally */
            virtual void path(ompl::geometric::PathGeometric *path, VizSizes type, VizColors color) = 0;

            /** \brief Trigger visualizer to refresh/repaint/display all graphics */
            virtual void trigger() = 0;

            /** \brief Trigger visualizer to clear all graphics */
            virtual void deleteAllMarkers() = 0;

            /** \brief Check if SIGINT has been called for shutdown */
            virtual bool shutdownRequested() = 0;

        };  // end of class VizWindow
    }       // namespace tools
}  // namespace ompl

#endif  // OMPL_TOOLS_VIZ_WINDOW_
