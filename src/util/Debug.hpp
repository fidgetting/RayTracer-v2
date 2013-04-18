/*
 * Debug.hpp
 *
 *  Created on: Jan 24, 2013
 *      Author: norton
 */

#pragma once

#ifdef DEBUG

/* std includes */
#include <iostream>
#include <stack>
#include <map>
#include <set>
#include <cstdio>
#include <stdint.h>
#include <memory>

/* boost includes */
#include <boost/preprocessor.hpp>

namespace ray {
  namespace debug {

#define DEBUG_PRINT(args) \
  do { if(ray::debug::section::print()) std::cout << args << std::flush; } while(0)

#define DEBUG_SECTION(name, condition) \
  ray::debug::section name(condition)

    class section {
      public:

        section(bool b);
        ~section();

        static bool print();

    };

#define DEBUG_WRAPPER(name, string) \
  ray::debug::wrapper name(wrapper)

    class wrapper {
      public:

        wrapper(std::string str) { DEBUG_PRINT(str << "{"); }
        ~wrapper() { DEBUG_PRINT("}" << std::endl); }
    };

#define GRAPH_DEF(name, ostr) do {                                 \
  if(ray::debug::section::print())                                 \
    ray::debug::dotGraph::graphs[BOOST_PP_STRINGIZE(name)] =       \
      ray::debug::dotGraph(ostr, BOOST_PP_STRINGIZE(name));        \
  } while(0)

#define GRAPH_NODE(name, key, label) do {                          \
  if(ray::debug::section::print() &&                               \
      !ray::debug::dotGraph::graphs[BOOST_PP_STRINGIZE(name)].     \
          contains(key))                                           \
        ray::debug::dotGraph::graphs[BOOST_PP_STRINGIZE(name)].    \
          nodeDef(key, label);                                     \
  } while(0)

#define GRAPH_EDGE(name, from, to) do {                            \
  if(ray::debug::section::print() &&                               \
    !ray::debug::dotGraph::graphs[BOOST_PP_STRINGIZE(name)].       \
        contains(from, to))                                        \
      ray::debug::dotGraph::graphs[BOOST_PP_STRINGIZE(name)].      \
        edgeDef(from, to);                                         \
  } while(0)

#define GRAPH_DONE(name) do {                                      \
  if(ray::debug::section::print())                                 \
    ray::debug::dotGraph::graphs.erase(BOOST_PP_STRINGIZE(name));  \
  } while(0)

    class dotGraph {
      public:

        typedef std::map<int32_t, uint32_t>             node_mapping;
        typedef std::map<uint32_t, std::set<uint32_t> > edge_mapping;
        typedef std::shared_ptr<std::ostream>           ostr_t;

        dotGraph() :
            nodeOut(NULL), nodeMap(), nodeGen(0) { }
        dotGraph(std::ostream& ostr, std::string name) :
            nodeOut(&ostr, deleter()), nodeMap(), nodeGen(1) {
          (*nodeOut) << "digraph " << name << " {" << std::endl;
        }

        inline void nodeDef(int32_t key, std::string label) {
          (*nodeOut) << "\t" << nodeGen << " [label=\""
                     << label << ": " << key << "\"]" << std::endl;
          nodeMap[key] = (nodeGen++);
        }

        inline void edgeDef(int32_t from, int32_t to) {
          (*nodeOut) << "\t" << nodeMap[from] << " -> "
                     << nodeMap[to] << std::endl;
          edgeMap[from].insert(to);
        }

        inline bool contains(int32_t key) {
          return nodeMap.find(key) != nodeMap.end();
        }

        inline bool contains(int32_t from, int32_t to) {
          return edgeMap[from].find(to) != edgeMap[from].end();
        }

        static std::map<std::string, dotGraph> graphs;

      private:

        struct deleter {
          void operator()(std::ostream* o) {
            if(o) (*o) << "}" << std::endl;
          }
        };

        ostr_t        nodeOut;
        node_mapping  nodeMap;
        edge_mapping  edgeMap;
        uint32_t      nodeGen;
    };

#else

namespace ray {
  namespace debug {

#define DEBUG_PRINT(args) \
  do { } while(0)

#define DEBUG_SECTION(name, contidtion) \
  do { } while(0)

#define DEBUG_WRAPPER(name, string) \
  do { } while(0)

#define GRAPH_DEF(name, ostr) \
  do { } while(0)

#define GRAPH_NODE(name, key, label) \
  do { } while(0)

#define GRAPH_EDGE(name, from, to) \
  do { } while(0)

#define GRAPH_DONE(name) \
  do { } while(0)

#endif

  }
}

