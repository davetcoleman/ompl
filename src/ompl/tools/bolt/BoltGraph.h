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
   Desc:   Basic graph components for Bolt
*/

#ifndef OMPL_TOOLS_BOLT_BOLTGRAPH_
#define OMPL_TOOLS_BOLT_BOLTGRAPH_

// OMPL
#include <ompl/base/State.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/PlannerTerminationCondition.h>

// Boost
#include <boost/range/adaptor/map.hpp>
#include <boost/unordered_map.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/pending/disjoint_sets.hpp>

namespace ompl
{
namespace tools
{
namespace bolt
{

// TODO(davetcoleman): maybe make all popularity use ints instead of doubles for memory efficiency?
static const double MAX_POPULARITY_WEIGHT = 100.0;  // 100 means the edge is very unpopular
// Everytime an edge is used, it is reduced by this amount (becomes more popular)
static const double POPULARITY_WEIGHT_REDUCTION = 5;

////////////////////////////////////////////////////////////////////////////////////////
// VERTEX PROPERTIES
////////////////////////////////////////////////////////////////////////////////////////

/** \brief Enumeration which specifies the reason a guard is added to the spanner. */
enum GuardType
{
    START,
    GOAL,
    COVERAGE,
    CONNECTIVITY,
    INTERFACE,
    QUALITY,
    CARTESIAN
};

/** \brief The type used internally for representing vertex IDs */
typedef unsigned long int VertexIndexType;  // TODO(davetcoleman): just use size_t?

/** \brief Boost vertex properties */
struct vertex_state_t
{
    typedef boost::vertex_property_tag kind;
};

struct vertex_dense_pointer_t
{
    typedef boost::vertex_property_tag kind;
};

// struct vertex_list_t
// {
//     typedef boost::vertex_property_tag kind;
// };

// struct vertex_interface_list_t
// {
//     typedef boost::vertex_property_tag kind;
//};

struct vertex_sparse_rep_t
{
    typedef boost::vertex_property_tag kind;
};

struct vertex_type_t
{
    typedef boost::vertex_property_tag kind;
};

struct vertex_popularity_t
{
    typedef boost::vertex_property_tag kind;
};

////////////////////////////////////////////////////////////////////////////////////////
// Edge properties

struct edge_collision_state_t
{
    typedef boost::edge_property_tag kind;
};

/** \brief Possible collision states of an edge */
enum EdgeCollisionState
{
    NOT_CHECKED,
    IN_COLLISION,
    FREE
};

////////////////////////////////////////////////////////////////////////////////////////
// INTERFACE PROPERTIES
////////////////////////////////////////////////////////////////////////////////////////

/** \brief Hash for storing interface information. */
typedef boost::unordered_map<VertexIndexType, std::set<VertexIndexType>, boost::hash<VertexIndexType> > InterfaceHash;

// The InterfaceHash structure is wrapped inside of this struct due to a compilation error on
// GCC 4.6 with Boost 1.48.  An implicit assignment operator overload does not compile with these
// components, so an explicit overload is given here.
// Remove this struct when the minimum Boost requirement is > v1.48.
struct InterfaceHashStruct
{
    InterfaceHashStruct &operator=(const InterfaceHashStruct &rhs)
    {
        interfaceHash = rhs.interfaceHash;
        return *this;
    }
    InterfaceHash interfaceHash;
};

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
// BOOST GRAPH - SPARSE
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////
/**
   \brief The underlying roadmap graph.

   \par Any BGL graph representation could be used here. Because we
   expect the roadmap to be sparse (m<n^2), an adjacency_list is more
   appropriate than an adjacency_matrix. Edges are undirected.

   *Properties of vertices*
   - vertex_dense_pointer_t: a reference back to the dense graph where the original state is stored
   - vertex_predecessor_t: Requred by incremental connected components algorithm (disjoint sets)
   - vertex_rank_t: Requred by incremental connected components algorithm (disjoint sets)
   - vertex_type_t: The type of guard this node is
   - vertex_list_t: non interface list property?

   Note: If boost::vecS is not used for vertex storage, then there must also
   be a boost:vertex_index_t property manually added.

   *Properties of edges*
   - edge_weight_t - cost/distance between two vertices
   - edge_collision_state_t - used for lazy collision checking, determines if an edge has been checked
   already for collision. 0 = not checked/unknown, 1 = in collision, 2 = free
*/

/** Wrapper for the vertex's multiple as its property. */
// clang-format off
typedef boost::property<vertex_dense_pointer_t, VertexIndexType,
        boost::property<boost::vertex_predecessor_t, VertexIndexType,
        boost::property<boost::vertex_rank_t, VertexIndexType,
        boost::property<vertex_type_t, GuardType,
        boost::property<vertex_popularity_t, double
        //boost::property<vertex_list_t, std::set<VertexIndexType>,
        //boost::property<vertex_interface_list_t, InterfaceHashStruct > >
        > > > > > SparseVertexProperties;
// clang-format on

/** Wrapper for the double assigned to an edge as its weight property. */
// clang-format off
typedef boost::property<boost::edge_weight_t, double,
        boost::property<edge_collision_state_t, int> > SparseEdgeProperties;
// clang-format on

/** The underlying boost graph type (undirected weighted-edge adjacency list with above properties). */
typedef boost::adjacency_list<boost::vecS,  // store in std::vector
                              boost::vecS,  // store in std::vector
                              boost::undirectedS, SparseVertexProperties, SparseEdgeProperties> SparseGraph;

/** \brief Vertex in Graph */
typedef boost::graph_traits<SparseGraph>::vertex_descriptor SparseVertex;

/** \brief Edge in Graph */
typedef boost::graph_traits<SparseGraph>::edge_descriptor SparseEdge;

////////////////////////////////////////////////////////////////////////////////////////
// Typedefs for property maps

/** \brief Access map that stores the lazy collision checking status of each edge */
typedef boost::property_map<SparseGraph, edge_collision_state_t>::type SparseEdgeCollisionStateMap;

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
// BOOST GRAPH - DENSE
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////
/**
   \brief The underlying roadmap graph.

   \par Any BGL graph representation could be used here. Because we
   expect the roadmap to be sparse (m<n^2), an adjacency_list is more
   appropriate than an adjacency_matrix. Edges are undirected.

   *Properties of vertices*
   - vertex_state_t: an ompl::base::State* is required for OMPL
   - vertex_predecessor_t: The incremental connected components algorithm requires it
   - vertex_rank_t: The incremental connected components algorithm requires it
   - vertex_type_t - Type of vertex/guard in SPARS

   Note: If boost::vecS is not used for vertex storage, then there must also
   be a boost:vertex_index_t property manually added.

   *Properties of edges*
   - edge_weight_t - cost/distance between two vertices
   - edge_collision_state_t - used for lazy collision checking, determines if an edge has been checked
   already for collision. 0 = not checked/unknown, 1 = in collision, 2 = free
*/

/** Wrapper for the vertex's multiple as its property. */

// clang-format off
typedef boost::property<vertex_state_t, base::State *,
        boost::property<boost::vertex_predecessor_t, VertexIndexType,
        boost::property<boost::vertex_rank_t, VertexIndexType,
        boost::property<vertex_sparse_rep_t, SparseVertex, // Currently unused
        boost::property<boost::vertex_rank_t, VertexIndexType,
        boost::property<vertex_type_t, GuardType> > > > > > DenseVertexProperties;
// clang-format on

/** Wrapper for the double assigned to an edge as its weight property. */
typedef boost::property<boost::edge_weight_t, double, boost::property<edge_collision_state_t, int> >
    DenseEdgeProperties;

/** The underlying boost graph type (undirected weighted-edge adjacency list with above properties). */
typedef boost::adjacency_list<boost::vecS,  // store in std::vector
                              boost::vecS,  // store in std::vector
                              boost::undirectedS, DenseVertexProperties, DenseEdgeProperties> DenseGraph;

/** \brief Vertex in Graph */
typedef boost::graph_traits<DenseGraph>::vertex_descriptor DenseVertex;

/** \brief Edge in Graph */
typedef boost::graph_traits<DenseGraph>::edge_descriptor DenseEdge;

/** \brief Internal representation of a dense path */
typedef std::deque<base::State *> DensePath;

////////////////////////////////////////////////////////////////////////////////////////
// Typedefs for property maps

/** \brief Access map that stores the lazy collision checking status of each edge */
typedef boost::property_map<DenseGraph, edge_collision_state_t>::type DenseEdgeCollisionStateMap;

////////////////////////////////////////////////////////////////////////////////////////
/** \brief Custom storage class */
class WeightedVertex
{
  public:
    WeightedVertex(DenseVertex v, double weight) : v_(v), weight_(weight)
    {
    }

    DenseVertex v_;
    double weight_;
};

    /** \brief Custom comparator class */
    class CompareWeightedVertex
    {
      public:
        bool operator()(WeightedVertex a, WeightedVertex b)
        {
            return a.weight_ < b.weight_;  // TODO(davetcoleman): which direction should the sign go?
        }
    };



}  // namespace bolt
}  // namespace tools
}  // namespace ompl

#endif  // OMPL_TOOLS_BOLT_BOLTGRAPH_
