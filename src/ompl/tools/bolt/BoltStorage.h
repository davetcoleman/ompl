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

#ifndef OMPL_TOOLS_BOLT_BOLT_STORAGE_H_
#define OMPL_TOOLS_BOLT_BOLT_STORAGE_H_

#include <iostream>
#include <fstream>
#include <vector>
#include <ompl/base/State.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/util/ClassForward.h>

// Boost
#include <boost/noncopyable.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/access.hpp>
#include <boost/archive/archive_exception.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

namespace ompl
{
namespace tools
{
namespace bolt
{
/**
   @anchor BoltStorage
   @par Short description
*/

/// @cond IGNORE
OMPL_CLASS_FORWARD(BoltStorage);
OMPL_CLASS_FORWARD(DenseDB);
/// @endcond

/** \class ompl::tools::bolt::BoltStoragePtr
    \brief A boost shared pointer wrapper for ompl::tools::bolt::BoltStorage */

static const boost::uint32_t OMPL_PLANNER_DATA_ARCHIVE_MARKER = 0x5044414D;  // this spells PDAM

class BoltStorage
{
  public:
    /// \brief Base class for a vertex in the PlannerData structure.  All
    /// derived classes must implement the clone and equivalence operators.
    /// It is assumed that each vertex in the PlannerData structure is
    /// unique (i.e. no duplicates allowed).
    class PlannerDataVertex
    {
      public:
        /// \brief Constructor.  Takes a state pointer and an optional integer tag.
        PlannerDataVertex(base::State *st, int tag = 0) : state_(st), tag_(tag)
        {
        }
        /// \brief Copy constructor.
        PlannerDataVertex(const PlannerDataVertex &rhs) : state_(rhs.state_), tag_(rhs.tag_)
        {
        }
        virtual ~PlannerDataVertex()
        {
        }

        /// \brief Returns the integer tag associated with this vertex.
        virtual int getTag() const
        {
            return tag_;
        }
        /// \brief Set the integer tag associated with this vertex.
        virtual void setTag(int tag)
        {
            tag_ = tag;
        }
        /// \brief Retrieve the state associated with this vertex.
        virtual base::State *getState()
        {
            return state_;
        }

        // /// \brief Return a clone of this object, allocated from the heap.
        // virtual PlannerDataVertex *clone() const
        // {
        //     return new PlannerDataVertex(*this);
        // }

        // /// \brief Equivalence operator.  Return true if the state pointers are equal.
        // virtual bool operator==(const PlannerDataVertex &rhs) const
        // {
        //     // States should be unique
        //     return state_ == rhs.state_;
        // }

        // /// \brief Returns true if this vertex is not equal to the argument.
        // /// This is the complement of the == operator.
        // bool operator!=(const PlannerDataVertex &rhs) const
        // {
        //     return !(*this == rhs);
        // }

      protected:
        PlannerDataVertex()
        {
        }

        friend class boost::serialization::access;
        template <class Archive>
        void serialize(Archive &ar, const unsigned int /*version*/)
        {
            ar &tag_;
            // Serialization of the state pointer is handled by PlannerDataStorage
        }

        /// \brief The state represented by this vertex
        base::State *state_;
        /// \brief A generic integer tag for this state.  Not used for equivalence checking.
        int tag_;

        // friend class PlannerData;
        // friend class PlannerDataStorage;
        friend class BoltStorage;
        friend class DenseDB;
    };

    /// \brief Base class for a PlannerData edge.
    class PlannerDataEdge
    {
      public:
        PlannerDataEdge()
        {
        }
        virtual ~PlannerDataEdge()
        {
        }
        // /// \brief Return a clone of this object, allocated from the heap.
        // virtual PlannerDataEdge *clone() const
        // {
        //     return new PlannerDataEdge();
        // }

        // /// \brief Returns true if the edges point to the same memory
        // virtual bool operator==(const PlannerDataEdge &rhs) const
        // {
        //     return this == &rhs;
        // }

        // /// \brief Returns true if the edges do not point to the same memory.
        // /// This is the complement of the == operator.
        // bool operator!=(const PlannerDataEdge &rhs) const
        // {
        //     return !(*this == rhs);
        // }

      protected:
        friend class boost::serialization::access;
        template <class Archive>
        void serialize(Archive & /*ar*/, const unsigned int /*version*/)
        {
        }
    };

    /// \brief Information stored at the beginning of the PlannerData archive
    struct Header
    {
        /// \brief OMPL PlannerData specific marker (fixed value)
        boost::uint32_t marker;

        /// \brief Number of vertices stored in the archive
        std::size_t vertex_count;

        /// \brief Number of edges stored in the archive
        std::size_t edge_count;

        /// \brief Signature of state space that allocated the saved states in the vertices (see
        /// ompl::base::StateSpace::computeSignature()) */
        std::vector<int> signature;

        /// \brief boost::serialization routine
        template <typename Archive>
        void serialize(Archive &ar, const unsigned int /*version*/)
        {
            ar &marker;
            ar &vertex_count;
            ar &edge_count;
            ar &signature;
        }
    };

    /// \brief The object containing all vertex data that will be stored
    struct PlannerDataVertexData
    {
        enum VertexType
        {
            STANDARD = 0,
            START,
            GOAL
        };

        template <typename Archive>
        void serialize(Archive &ar, const unsigned int /*version*/)
        {
            ar &v_;
            ar &state_;
            ar &type_;
        }

        PlannerDataVertex *v_;
        std::vector<unsigned char> state_;
        VertexType type_;
    };

    /// \brief The object containing all edge data that will be stored
    struct PlannerDataEdgeData
    {
        template <typename Archive>
        void serialize(Archive &ar, const unsigned int /*version*/)
        {
            ar &e_;
            ar &endpoints_;
            ar &weight_;
        }

        const PlannerDataEdge *e_;
        std::pair<unsigned int, unsigned int> endpoints_;
        double weight_;
    };

    /** \brief Constructor */
    BoltStorage(const base::SpaceInformationPtr &si, DenseDB *denseDB);

    void store(const char *filename);

    void store(std::ostream &out);

    /// \brief Serialize and store all vertices in \e pd to the binary archive.
    void storeVertices(boost::archive::binary_oarchive &oa);

    /// \brief Serialize and store all edges in \e pd to the binary archive.
    void storeEdges(boost::archive::binary_oarchive &oa);

    void load(const char *filename);

    void load(std::istream &in);

    /// \brief Read \e numVertices from the binary input \e ia and store them as PlannerData.
    void loadVertices(unsigned int numVertices, boost::archive::binary_iarchive &ia);

    /// \brief Read \e numEdges from the binary input \e ia and store them as PlannerData.
    void loadEdges(unsigned int numEdges, boost::archive::binary_iarchive &ia);

    /// \brief The space information instance for this data.
    base::SpaceInformationPtr si_;

    DenseDB *denseDB_;

};  // end of class BoltStorage

}  // namespace bolt
}  // namespace tools
}  // namespace ompl

#endif
