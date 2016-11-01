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

//#include <ompl/base/Planner.h>
#include <ompl/tools/debug/VizWindow.h>

// C++
#include <functional>
#include <cassert>

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

        /**
         * \brief Visualization callback hook for external debugging that triggers the visualizer to render/publish
         */
        typedef std::function<void(void)> VizTrigger;

        /**
         * \brief Visualization callback hook for telling program to wait for user input
         */
        typedef std::function<void(const std::string &)> VizWaitFeedback;

        /** \brief Use an external program to visualize search */
        class Visualizer
        {
        public:
            static const std::size_t NUM_VISUALIZERS = 7;

            Visualizer()
            {
                viz_.resize(NUM_VISUALIZERS);
            }

            // ------------------------------------------------------------------------------------

            /** \brief Wait for user feedback i.e. through a button or joystick */
            void waitForUserFeedback(const std::string &msg)
            {
                viz_.front()->prompt(msg);
            }
            void prompt(const std::string &msg)
            {
                viz_.front()->prompt(msg);
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

            void setVizWindow(std::size_t id, VizWindowPtr viz)
            {
                viz_[id - 1] = viz;
            }

            VizWindowPtr viz(std::size_t id)
            {
                assert(id > 0 && id <= NUM_VISUALIZERS);
                return viz_[id - 1];
            }

            VizWindowPtr viz1()
            {
                return viz_[1 - 1];
            }

            VizWindowPtr viz2()
            {
                return viz_[2 - 1];
            }

            VizWindowPtr viz3()
            {
                return viz_[3 - 1];
            }

            VizWindowPtr viz4()
            {
                return viz_[4 - 1];
            }

            VizWindowPtr viz5()
            {
                return viz_[5 - 1];
            }

            VizWindowPtr viz6()
            {
                return viz_[6 - 1];
            }

            VizWindowPtr viz7()
            {
                return viz_[7 - 1];
            }

        private:
            /** \brief Callback to display the node covereage in 2D */
            VizTrigger vizVoronoiDiagram_;

            // Pointers to visualization windows in another project
            std::vector<VizWindowPtr> viz_;

        };  // end of class Visualizer
    }       // namespace tools
}  // namespace ompl

#endif  // OMPL_TOOLS_DEBUG_VISUALIZER_
