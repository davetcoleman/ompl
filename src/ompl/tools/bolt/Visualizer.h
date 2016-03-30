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

#ifndef OMPL_TOOLS_BOLT_VISUALIZER_
#define OMPL_TOOLS_BOLT_VISUALIZER_

#include <ompl/base/Planner.h>

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

        /** \brief Use an external program to visualize search */
        class Visualizer
        {
        public:
            /** \brief Visualize a planner's data during runtime, externally, using a function callback
             *         This could be called whenever the graph changes */
            void viz1StateCallback(const ompl::base::State* state, std::size_t type, double neighborRadius)
            {
                if (viz1StateCallback_)
                    viz1StateCallback_(state, type, neighborRadius);
            }

            /** \brief Visualize a planner's data during runtime, externally, using a function callback
             *         This could be called whenever the graph changes */
            void viz1EdgeCallback(const ompl::base::State* stateA, const ompl::base::State* stateB, double cost)
            {
                if (viz1EdgeCallback_)
                    viz1EdgeCallback_(stateA, stateB, cost);
            }

            /** \brief Trigger visualizer to publish graphics */
            void viz1TriggerCallback()
            {
                if (viz1TriggerCallback_)
                    viz1TriggerCallback_();
            }

            /** \brief Set the callback to visualize/publish a planner's progress */
            void setViz1Callbacks(ompl::base::VizStateCallback vizStateCallback, ompl::base::VizEdgeCallback vizEdgeCallback,
                ompl::base::VizTriggerCallback vizTriggerCallback)
            {
                viz1StateCallback_ = vizStateCallback;
                viz1EdgeCallback_ = vizEdgeCallback;
                viz1TriggerCallback_ = vizTriggerCallback;
            }

            /** \brief Visualize a planner's data during runtime, externally, using a function callback
             *         This could be called whenever the graph changes */
            void viz2StateCallback(const ompl::base::State* state, std::size_t type, double neighborRadius)
            {
                if (viz2StateCallback_)
                    viz2StateCallback_(state, type, neighborRadius);
            }

            /** \brief Visualize a planner's data during runtime, externally, using a function callback
             *         This could be called whenever the graph changes */
            void viz2EdgeCallback(const ompl::base::State* stateA, const ompl::base::State* stateB, double cost)
            {
                if (viz2EdgeCallback_)
                    viz2EdgeCallback_(stateA, stateB, cost);
            }

            /** \brief Trigger visualizer to publish graphics */
            void viz2TriggerCallback()
            {
                if (viz2TriggerCallback_)
                    viz2TriggerCallback_();
            }

            /** \brief Set the callback to visualize/publish a planner's progress */
            void setViz2Callbacks(ompl::base::VizStateCallback vizStateCallback, ompl::base::VizEdgeCallback vizEdgeCallback,
                ompl::base::VizTriggerCallback vizTriggerCallback)
            {
                viz2StateCallback_ = vizStateCallback;
                viz2EdgeCallback_ = vizEdgeCallback;
                viz2TriggerCallback_ = vizTriggerCallback;
            }

            /** \brief Visualize a planner's data during runtime, externally, using a function callback
             *         This could be called whenever the graph changes */
            void vizDBStateCallback(const ompl::base::State* state, std::size_t type, double neighborRadius)
            {
                if (vizDBStateCallback_)
                    vizDBStateCallback_(state, type, neighborRadius);
            }

            /** \brief Visualize a planner's data during runtime, externally, using a function callback
             *         This could be called whenever the graph changes */
            void vizDBEdgeCallback(const ompl::base::State* stateA, const ompl::base::State* stateB, double cost)
            {
                if (vizDBEdgeCallback_)
                    vizDBEdgeCallback_(stateA, stateB, cost);
            }

            /** \brief Trigger visualizer to publish graphics */
            void vizDBTriggerCallback()
            {
                if (vizDBTriggerCallback_)
                    vizDBTriggerCallback_();
            }

            /** \brief Set the callback to visualize/publish a planner's progress */
            void setVizDBCallbacks(ompl::base::VizStateCallback vizStateCallback, ompl::base::VizEdgeCallback vizEdgeCallback,
                ompl::base::VizTriggerCallback vizTriggerCallback)
            {
                vizDBStateCallback_ = vizStateCallback;
                vizDBEdgeCallback_ = vizEdgeCallback;
                vizDBTriggerCallback_ = vizTriggerCallback;
            }

    private:

            /** \brief Optional callbacks to allow easy introspection of a planner's search progress */
            ompl::base::VizStateCallback viz1StateCallback_;
            ompl::base::VizEdgeCallback viz1EdgeCallback_;
            ompl::base::VizTriggerCallback viz1TriggerCallback_;

            /** \brief Optional secondary callbacks to allow easy introspection of database */
            ompl::base::VizStateCallback vizDBStateCallback_;
            ompl::base::VizEdgeCallback vizDBEdgeCallback_;
            ompl::base::VizTriggerCallback vizDBTriggerCallback_;

            /** \brief Optional third callbacks for other data */
            ompl::base::VizStateCallback viz2StateCallback_;
            ompl::base::VizEdgeCallback viz2EdgeCallback_;
            ompl::base::VizTriggerCallback viz2TriggerCallback_;

        };  // end of class Visualizer

    }  // namespace tools
}  // namespace ompl

#endif // OMPL_TOOLS_BOLT_VISUALIZER_
