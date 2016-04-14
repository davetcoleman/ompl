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
   Desc:   Utility to discretize a space into a uniform grid
*/

// OMPL
#include <ompl/tools/bolt/Discretizer.h>
#include <ompl/tools/bolt/DenseDB.h>
//#include <ompl/base/ScopedState.h>
//#include <ompl/util/Time.h>
//#include <ompl/util/Console.h>
//#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>
//#include <ompl/base/spaces/RealVectorStateSpace.h>  // TODO: remove, this is not space agnostic

// Boost
// #include <boost/filesystem.hpp>
// #include <boost/lambda/bind.hpp>
// #include <boost/graph/incremental_components.hpp>
// #include <boost/property_map/vector_property_map.hpp>
#include <boost/foreach.hpp>
#include <boost/thread.hpp>

// C++
//#include <limits>
//#include <queue>

#define foreach BOOST_FOREACH

namespace og = ompl::geometric;
namespace ot = ompl::tools;
namespace ob = ompl::base;

namespace ompl
{
namespace tools
{
namespace bolt
{

Discretizer::Discretizer(base::SpaceInformationPtr si, DenseDBPtr denseDB, base::VisualizerPtr visual)
  : si_(si)
  , denseDB_(denseDB)
  , visual_(visual)
  , visualizeGridGeneration_(false)
  , discretization_(2.0)
{

}

Discretizer::~Discretizer(void)
{

}

void Discretizer::generateGrid()
{
    OMPL_INFORM("Generating grid");

    if (!si_->isSetup())
    {
        OMPL_WARN("Space information setup was not yet called. Calling now.");
        si_->setup();
    }

    // Prepare for recursion
    // TODO(davetcoleman): currently the last joint is not being discretized, so we should set its
    // default value smartly and not just '0'
    std::vector<double> values(si_->getStateSpace()->getDimension(), 0);

    // Choose first state to discretize
    nextDiscretizedState_ =
        si_->getStateSpace()->allocState();  // Note: it is currently possible the last state is never freed

    std::size_t desired_depth = si_->getStateSpace()->getDimension();

    // TODO: This is a custom dimensionality reduction hack, that maybe should not be in this location
    if (desired_depth > 5)
    {
        // OMPL_INFORM("Truncated discretization depth to 5");
        desired_depth = 6;
    }
    else if (desired_depth == 3)
    {
        // This is for the 2D case where the third dimension is task
        OMPL_INFORM("Truncated discretization depth to 2");
        desired_depth = 2;

        // Set all generated states to a level 0 task space
        values[2] = 0;
    }

    // Create vertices
    const std::size_t starting_joint_id = 0;
    recursiveDiscretization(values, starting_joint_id, desired_depth);
    OMPL_INFORM("Generated %i vertices.", denseDB_->getNumVertices());

    // Error check
    if (denseDB_->getNumVertices() < 2)
    {
        OMPL_ERROR("No vertices generated, failing");
        exit(-1);
    }

    // Remove vertices in collision using multithreading
    // TODO enable
    // std::vector<DenseVertex> unvalidatedVertices;
    // checkVerticesThreaded(unvalidatedVertices);

    {
        // Benchmark runtime
        time::point start_time = time::now();

        // Create edges ----------------------------------------
        generateEdges();

        // Benchmark runtime
        double duration = time::seconds(time::now() - start_time);
        OMPL_INFORM("Generate edges total time: %f seconds (%f hz)", duration, 1.0 / duration);
    }

    // Get the average vertex degree (number of connected edges)
    std::size_t average_degree = (denseDB_->getNumEdges() * 2) / denseDB_->getNumVertices();
    OMPL_INFORM("Average degree: %i", average_degree);

    // Display
    if (visualizeGridGeneration_)
        visual_->viz1Trigger();

}

std::size_t Discretizer::getEdgesPerVertex(base::SpaceInformationPtr si)
{
    // in 2D this creates the regular square with diagonals of 8 edges
    if (si->getStateSpace()->getDimension() == 3)
    {
        return 8;
    }

    // full robot
    return si->getStateSpace()->getDimension() * 2;
}

void Discretizer::generateEdges()
{
    OMPL_INFORM("Generating edges");
    bool verbose = false;

    // Benchmark runtime
    time::point startTime = time::now();
    std::size_t count = 0;

    // Nearest Neighbor search
    std::vector<DenseVertex> graphNeighborhood;
    std::vector<DenseEdge> unvalidatedEdges;
    std::size_t feedbackFrequency = std::max(static_cast<int>(denseDB_->getNumVertices() / 10), 10);

    // Loop through each vertex
    for (std::size_t v1 = 1; v1 < denseDB_->getNumVertices(); ++v1)  // 1 because 0 is the search vertex?
    {
        if (v1 % feedbackFrequency == 0)
        {
            // Benchmark runtime
            double duration = time::seconds(time::now() - startTime);
            std::cout << "Edge generation progress: " << double(v1) / denseDB_->getNumVertices() * 100.0 << " % "
                      << "Total edges: " << count << " Total time: " << duration << std::endl;
            startTime = time::now();
        }

        // Add edges
        graphNeighborhood.clear();

        // How many edges should each vertex connect with?
        std::size_t findNearestKNeighbors = getEdgesPerVertex(si_);
        const std::size_t numSameVerticiesFound = 1;  // add 1 to the end because the NN tree always returns itself

        // Search
        denseDB_->stateProperty_[denseDB_->queryVertex_] = denseDB_->stateProperty_[v1];
        denseDB_->nn_->nearestK(denseDB_->queryVertex_, findNearestKNeighbors + numSameVerticiesFound, graphNeighborhood);
        denseDB_->stateProperty_[denseDB_->queryVertex_] = NULL;  // Set search vertex to NULL to prevent segfault on class unload of memory

        if (verbose)
            OMPL_INFORM("Found %u neighbors", graphNeighborhood.size());

        // For each nearby vertex, add an edge
        std::size_t errorCheckNumSameVerticies = 0;  // sanity check
        for (std::size_t i = 0; i < graphNeighborhood.size(); ++i)
        {
            if (verbose)
                OMPL_INFORM("Edge %u", i);

            DenseVertex &v2 = graphNeighborhood[i];

            // Check if these vertices are the same
            if (v1 == v2)
            {
                errorCheckNumSameVerticies++;  // sanity check
                continue;
            }

            // Check if these vertices already share an edge
            if (boost::edge(v1, v2, denseDB_->g_).second)
                continue;

            if (visualizeGridGeneration_)  // Debug: display edge
                visual_->viz1Edge(denseDB_->stateProperty_[v1], denseDB_->stateProperty_[v2], 1);

            // Create edge - maybe removed later
            DenseEdge e = denseDB_->addEdge(v1, v2, denseDB_->desiredAverageCost_);
            unvalidatedEdges.push_back(e);

            count++;
        }  // for each v2

        // Make sure one and only one vertex is returned from the NN search that is the same as parent vertex
        assert(errorCheckNumSameVerticies == 1);

        // Visualize
        if (visualizeGridGeneration_)
        {
            visual_->viz1Trigger();
            usleep(0.001 * 1000000);
        }

    }  // for each v1

    OMPL_INFORM("Generated %i edges. Finished generating grid.", denseDB_->getNumEdges());

    // Benchmark runtime
    time::point startTime3 = time::now();
    std::size_t numEdgesBeforeCheck = denseDB_->getNumEdges();

    // Collision check all edges using threading
    checkEdgesThreaded(unvalidatedEdges);

    // Calculate statistics on collision state of robot
    std::size_t numEdgesAfterCheck = denseDB_->getNumEdges();
    std::size_t numRemovedEdges = numEdgesBeforeCheck - numEdgesAfterCheck;
    double duration3 = time::seconds(time::now() - startTime3);

    // User feedback
    OMPL_INFORM("Collision State of Robot -----------------");
    OMPL_INFORM("   Removed Edges:        %u", numRemovedEdges);
    OMPL_INFORM("   Remaining Edges:      %u", numEdgesAfterCheck);
    OMPL_INFORM("   Percent Removed:      %f %%", numRemovedEdges / double(numEdgesBeforeCheck) * 100.0);
    OMPL_INFORM("   Total time:           %f sec (%f hz)", duration3, 1.0 / duration3);

    // Calculate cost for each edge
    std::size_t errorCheckCounter = 0;
    foreach (const DenseEdge e, boost::edges(denseDB_->g_))
    {
        const DenseVertex &v1 = boost::source(e, denseDB_->g_);
        const DenseVertex &v2 = boost::target(e, denseDB_->g_);

        // Determine cost for edge depending on mode
        double cost;
        if (denseDB_->popularityBiasEnabled_)
        {
            cost = MAX_POPULARITY_WEIGHT;
        }
        else
        {
            cost = denseDB_->distanceFunction(v1, v2);
        }
        denseDB_->edgeWeightProperty_[e] = cost;
        errorCheckCounter++;

        if (visualizeGridGeneration_)  // Debug in Rviz
        {
            visual_->viz1Edge(denseDB_->stateProperty_[v1], denseDB_->stateProperty_[v2], denseDB_->edgeWeightProperty_[e]);
            if (errorCheckCounter % 100 == 0)
            {
                visual_->viz1Trigger();
                usleep(0.001 * 1000000);
            }
        }
    }
    assert(errorCheckCounter == numEdgesAfterCheck);
}

void Discretizer::checkEdges()  // TODO: deprecated, remove this func
{
    OMPL_WARN("Collision checking generated edges without threads");
    OMPL_ERROR("Has not been updated for remove_edge");

    foreach (const DenseEdge e, boost::edges(denseDB_->g_))
    {
        const DenseVertex &v1 = boost::source(e, denseDB_->g_);
        const DenseVertex &v2 = boost::target(e, denseDB_->g_);

        // Remove any edges that are in collision
        if (!si_->checkMotion(denseDB_->stateProperty_[v1], denseDB_->stateProperty_[v2]))
        {
            denseDB_->edgeCollisionStateProperty_[e] = IN_COLLISION;
        }
        else
        {
            denseDB_->edgeCollisionStateProperty_[e] = FREE;
        }
    }
}

void Discretizer::checkEdgesThreaded(const std::vector<DenseEdge> &unvalidatedEdges)
{
    const bool verbose = false;

    // Error check
    assert(unvalidatedEdges.size() == denseDB_->getNumEdges());

    // Setup threading
    static const std::size_t numThreads = boost::thread::hardware_concurrency();
    OMPL_INFORM("Collision checking %u generated edges using %u threads", unvalidatedEdges.size(), numThreads);

    std::vector<boost::thread *> threads(numThreads);
    std::size_t numEdges = denseDB_->getNumEdges();  // we copy this number, because it might start shrinking when threads spin up
    std::size_t edgesPerThread = numEdges / numThreads;  // rounds down
    std::size_t startEdge = 0;
    std::size_t endEdge;
    std::size_t errorCheckTotalEdges = 0;

    // For each thread
    for (std::size_t i = 0; i < threads.size(); ++i)
    {
        endEdge = startEdge + edgesPerThread - 1;

        // Check if this is the last thread
        if (i == threads.size() - 1)
        {
            // have it do remaining edges to check
            endEdge = numEdges - 1;
        }
        errorCheckTotalEdges += (endEdge - startEdge);

        if (verbose)
            std::cout << "Thread " << i << " has edges from " << startEdge << " to " << endEdge << std::endl;

        base::SpaceInformationPtr si(new base::SpaceInformation(si_->getStateSpace()));
        si->setStateValidityChecker(si_->getStateValidityChecker());
        si->setMotionValidator(si_->getMotionValidator());

        threads[i] = new boost::thread(
            boost::bind(&Discretizer::checkEdgesThread, this, startEdge, endEdge, si, unvalidatedEdges));
        startEdge += edgesPerThread;
    }

    // Join threads
    for (std::size_t i = 0; i < threads.size(); ++i)
    {
        threads[i]->join();
        delete threads[i];
    }

    // Error check
    if (errorCheckTotalEdges == denseDB_->getNumEdges())
    {
        OMPL_ERROR("Incorrect number of edges were processed");
        exit(-1);
    }

    // Sanity check: make sure all remaining edges were validated
    foreach (const DenseEdge e, boost::edges(denseDB_->g_))
    {
        if (denseDB_->edgeCollisionStateProperty_[e] != FREE)
        {
            OMPL_ERROR("Remaining edge %u has not been marked free", e);
        }
    }
}

void Discretizer::checkEdgesThread(std::size_t startEdge, std::size_t endEdge, base::SpaceInformationPtr si,
                                    const std::vector<DenseEdge> &unvalidatedEdges)
{
    // Process [startEdge, endEdge] inclusive
    for (std::size_t edgeID = startEdge; edgeID <= endEdge; ++edgeID)
    {
        const DenseEdge &e = unvalidatedEdges[edgeID];

        const DenseVertex &v1 = boost::source(e, denseDB_->g_);
        const DenseVertex &v2 = boost::target(e, denseDB_->g_);

        // Remove any edges that are in collision
        if (!si->checkMotion(denseDB_->stateProperty_[v1], denseDB_->stateProperty_[v2]))
        {
            boost::remove_edge(v1, v2, denseDB_->g_);
        }
        else
        {
            denseDB_->edgeCollisionStateProperty_[e] = FREE;
        }
    }
}

void Discretizer::recursiveDiscretization(std::vector<double> &values, std::size_t joint_id, std::size_t desiredDepth)
{
    ob::RealVectorBounds bounds = si_->getStateSpace()->getBounds();

    // Error check
    assert(bounds.high.size() == bounds.low.size());
    assert(bounds.high.size() == si_->getStateSpace()->getDimension());
    assert(joint_id < values.size());

    // Loop through current joint
    for (double value = bounds.low[joint_id]; value <= bounds.high[joint_id]; value += discretization_)
    {
        values[joint_id] = value;

        // User feedback
        if (joint_id == 0)
        {
            const double percent =
                (value - bounds.low[joint_id]) / (bounds.high[joint_id] - bounds.low[joint_id]) * 100.0;
            std::cout << "Vertex generation progress: " << percent << " % Total vertices: " << denseDB_->getNumVertices()
                      << std::endl;
        }

        // Check if we are at the end of the recursion
        if (joint_id < desiredDepth - 1)
        {
            // Keep recursing
            recursiveDiscretization(values, joint_id + 1, desiredDepth);
        }
        else  // this is the end of recursion, create a new state
        {
            // TODO(davetcoleman): way to not allocState if in collision?
            nextDiscretizedState_ = si_->getStateSpace()->allocState();

            // Fill the state with current values
            si_->getStateSpace()->populateState(nextDiscretizedState_, values);

            // Collision check
            if (!si_->isValid(nextDiscretizedState_))
            {
                // OMPL_ERROR("Found a state that is not valid! ");
                continue;
            }

            // Add vertex to graph
            GuardType type = START;  // TODO(davetcoleman): type START is dummy
            denseDB_->addVertex(nextDiscretizedState_, type);

            // Visualize
            if (visualizeGridGeneration_)
            {
                // Candidate node has already (just) been added
                visual_->viz1State(nextDiscretizedState_, /*mode=*/5, 1);
                visual_->viz1Trigger();
                usleep(0.001 * 1000000);
            }

            // Prepare for next new state by allocating now
            // nextDiscretizedState_ = si_->getStateSpace()->allocState();
        }
    }
}

void Discretizer::checkVerticesThreaded(const std::vector<DenseVertex> &unvalidatedVertices)
{
    // TODO(davetcoleman): have not tested this functionality
    bool verbose = true;

    // Setup threading
    static const std::size_t numThreads = boost::thread::hardware_concurrency();
    OMPL_INFORM("Collision checking %u generated vertices using %u threads", unvalidatedVertices.size(), numThreads);

    std::vector<boost::thread *> threads(numThreads);
    std::size_t verticesPerThread = denseDB_->getNumVertices() / numThreads;  // rounds down
    std::size_t startVertex = 0;
    std::size_t endVertex;
    std::size_t errorCheckTotalVertices = 0;

    // For each thread
    for (std::size_t i = 0; i < threads.size(); ++i)
    {
        endVertex = startVertex + verticesPerThread - 1;

        // Check if this is the last thread
        if (i == threads.size() - 1)
        {
            // have it do remaining vertices to check
            endVertex = denseDB_->getNumVertices() - 1;
        }
        errorCheckTotalVertices += (endVertex - startVertex);

        if (verbose)
            std::cout << "Thread " << i << " has vertices from " << startVertex << " to " << endVertex << std::endl;

        base::SpaceInformationPtr si(new base::SpaceInformation(si_->getStateSpace()));
        si->setStateValidityChecker(si_->getStateValidityChecker());
        // si->setMotionValidator(si_->getMotionValidator());

        threads[i] = new boost::thread(
            boost::bind(&Discretizer::checkVerticesThread, this, startVertex, endVertex, si, unvalidatedVertices));
        startVertex += verticesPerThread;
    }

    // Join threads
    for (std::size_t i = 0; i < threads.size(); ++i)
    {
        threads[i]->join();
        delete threads[i];
    }

    // Error check
    if (errorCheckTotalVertices == denseDB_->getNumVertices())
    {
        OMPL_ERROR("Incorrect number of vertices were processed");
        exit(-1);
    }
}

void Discretizer::checkVerticesThread(std::size_t startVertex, std::size_t endVertex, base::SpaceInformationPtr si,
                                       const std::vector<DenseVertex> &unvalidatedVertices)
{
    // Process [startVertex, endVertex] inclusive
    for (std::size_t vertexID = startVertex; vertexID <= endVertex; ++vertexID)
    {
        const DenseVertex &v = unvalidatedVertices[vertexID];

        // Remove any vertices that are in collision
        if (!si_->isValid(denseDB_->stateProperty_[v]))
        {
            boost::remove_vertex(v, denseDB_->g_);
            std::cout << "found vertex in collision " << std::endl;
        }
    }
}

} // namespace
} // namespace
} // namespace
