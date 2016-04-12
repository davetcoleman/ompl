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
   Desc:   Save and load from file
*/

// OMPL
#include <ompl/tools/bolt/BoltStorage.h>
#include <ompl/tools/bolt/DenseDB.h>
#include <ompl/tools/bolt/BoltGraph.h>

// Boost
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

namespace ompl
{
namespace tools
{
namespace bolt
{
BoltStorage::BoltStorage(const base::SpaceInformationPtr &si, DenseDB *denseDB) : si_(si), denseDB_(denseDB)
{
    std::cout << "BoltStorage constructor " << std::endl;
}

void BoltStorage::store(std::ostream &out)
{
    if (!out.good())
    {
        OMPL_ERROR("Failed to store PlannerData: output stream is invalid");
        return;
    }

    try
    {
        boost::archive::binary_oarchive oa(out);

        // Writing the header
        Header h;
        h.marker = OMPL_PLANNER_DATA_ARCHIVE_MARKER;
        h.vertex_count = denseDB_->getNumVertices();
        h.edge_count = denseDB_->getNumEdges();
        OMPL_INFORM("Computing state space signuture");
        si_->getStateSpace()->computeSignature(h.signature);
        OMPL_INFORM("Writing header");
        oa << h;

        OMPL_INFORM("Storing vertices");
        storeVertices(oa);
        OMPL_INFORM("Storing edges");
        storeEdges(oa);
    }
    catch (boost::archive::archive_exception &ae)
    {
        OMPL_ERROR("Failed to store PlannerData: %s", ae.what());
    }
}

void BoltStorage::store(const char *filename)
{
    std::ofstream out(filename, std::ios::binary);

    // Write the number of paths we will be saving
    OMPL_WARN("TODO: remove numPaths");
    double numPaths = 1;
    out << numPaths;

    store(out);
    out.close();
}

void BoltStorage::storeVertices(boost::archive::binary_oarchive &oa)
{
    /*
    const base::StateSpacePtr &space = si_->getStateSpace();

    std::vector<unsigned char> state(space->getSerializationLength());
    std::size_t feedbackFrequency = denseDB_->getNumVertices() / 10;

    std::cout << "Saving vertices: " << std::flush;
    std::size_t count = 0;
    foreach (const DenseVertex v, boost::vertices(denseDB_->g_))
    {
        // Skip the query vertex that is NULL
        if (v == denseDB_->queryVertex_)
            continue;

        PlannerDataVertexData vertexData;

        // Serializing all data in the vertex (except the state)
        PlannerDataVertex vData(denseDB_->stateProperty_[v], denseDB_->typeProperty_[v]);
        vertexData.v_ = &vData;

        // Record the type of the vertex (i.e. start vertex).
        vertexData.type_ = PlannerDataVertexData::STANDARD;

        // Serializing the state contained in this vertex
        space->serialize(&state[0], vData.getState());
        vertexData.state_ = state;

        oa << vertexData;

        // Feedback
        if ((++count) % feedbackFrequency == 0)
            std::cout << std::setprecision(0) << (count / double(denseDB_->getNumVertices())) * 100.0 << "% " << std::flush;
    }
    */
    std::cout << std::endl;
}

void BoltStorage::storeEdges(boost::archive::binary_oarchive &oa)
{
    /*
    std::size_t feedbackFrequency = denseDB_->getNumVertices() / 10;

    std::cout << "Saving edges: " << std::flush;
    std::size_t count = 0;
    foreach (const DenseEdge e, boost::edges(denseDB_->g_))
    {
        const DenseVertex v1 = boost::source(e, denseDB_->g_);
        const DenseVertex v2 = boost::target(e, denseDB_->g_);

        // Convert to new structure
        PlannerDataEdgeData edgeData;
        //PlannerDataEdge edge;
        //edgeData.e_ = &edge;
        edgeData.endpoints_.first = v1;
        edgeData.endpoints_.second = v2;
        edgeData.weight_ = denseDB_->edgeWeightProperty_[e];
        oa << edgeData;

        // Feedback
        if ((++count + 1) % feedbackFrequency == 0)
            std::cout << std::setprecision(0) << (count / double(denseDB_->getNumEdges())) * 100.0 << "% " << std::flush;

    }  // for each edge
    std::cout << std::endl;
    */
}

void BoltStorage::load(const char *filename)
{
    std::ifstream in(filename, std::ios::binary);

    // Get the total number of paths saved
    OMPL_WARN("TODO: remove numPaths");
    double numPaths = 0;
    in >> numPaths;

    load(in);
    in.close();
}

void BoltStorage::load(std::istream &in)
{
    if (!in.good())
    {
        OMPL_ERROR("Failed to load PlannerData: input stream is invalid");
        return;
    }

    // Loading the planner data:
    try
    {
        boost::archive::binary_iarchive ia(in);

        // Read the header
        Header h;
        ia >> h;

        // Checking the archive marker
        if (h.marker != OMPL_PLANNER_DATA_ARCHIVE_MARKER)
        {
            OMPL_ERROR("Failed to load PlannerData: PlannerData archive marker not found");
            return;
        }

        // Verify that the state space is the same
        std::vector<int> sig;
        si_->getStateSpace()->computeSignature(sig);
        if (h.signature != sig)
        {
            OMPL_ERROR("Failed to load PlannerData: StateSpace signature mismatch");
            return;
        }

        // File seems ok... loading vertices and edges
        loadVertices(h.vertex_count, ia);
        loadEdges(h.edge_count, ia);
    }
    catch (boost::archive::archive_exception &ae)
    {
        OMPL_ERROR("Failed to load PlannerData: %s", ae.what());
    }
}

void BoltStorage::loadVertices(unsigned int numVertices, boost::archive::binary_iarchive &ia)
{
    OMPL_INFORM("Loading %u verticies from file", numVertices);

    const base::StateSpacePtr &space = si_->getStateSpace();
    std::size_t feedbackFrequency = numVertices / 10;

    std::cout << "Vertices loaded: ";
    for (unsigned int i = 0; i < numVertices; ++i)
    {
        PlannerDataVertexData vertexData;
        ia >> vertexData;

        // Deserializing all data in the vertex (except the state)
        PlannerDataVertex *v = vertexData.v_;

        // Allocating a new state and deserializing it from the buffer
        base::State *state = space->allocState();
        space->deserialize(state, &vertexData.state_[0]);

        // const_cast<PlannerDataVertex *>(v)->state_ = state;
        v->state_ = state;

        // Record the type of the vertex (i.e. start vertex).
        denseDB_->addVertexFromFile(v);

        // We deserialized the vertex object pointer, and we own it.
        delete vertexData.v_;

        // Feedback
        if ((i + 1) % feedbackFrequency == 0)
            std::cout << std::setprecision(0) << (i / double(numVertices)) * 100.0 << "% " << std::flush;
    }
    std::cout << std::endl;
}

void BoltStorage::loadEdges(unsigned int numEdges, boost::archive::binary_iarchive &ia)
{
    OMPL_INFORM("Loading %u edges from file", numEdges);
    std::size_t feedbackFrequency = numEdges / 10;

    std::cout << "Edges loaded: ";
    for (unsigned int i = 0; i < numEdges; ++i)
    {
        PlannerDataEdgeData edgeData;
        ia >> edgeData;
        // pd.addEdge(edgeData.endpoints_.first, edgeData.endpoints_.second, *edgeData.e_, Cost(edgeData.weight_));
        denseDB_->addEdgeFromFile(edgeData);

        // We deserialized the edge object pointer, and we own it.
        // Since addEdge copies the object, it is safe to free here.
        delete edgeData.e_;

        // Feedback
        if ((i + 1) % feedbackFrequency == 0)
            std::cout << std::setprecision(0) << (i / double(numEdges)) * 100.0 << "% " << std::flush;
    }
    std::cout << std::endl;
}

}  // namespace bolt
}  // namespace tools
}  // namespace ompl
