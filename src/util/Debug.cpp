/*
 * Debug.cpp
 *
 *  Created on: Jan 24, 2013
 *      Author: norton
 */

/* local includes */
#include <Debug.hpp>

#ifdef DEBUG

namespace ray {
  namespace debug {

    static std::stack<bool> _stack;

    section::section(bool b) {
      _stack.push(b);
    }

    section::~section() {
      _stack.pop();
    }

    bool section::print() {
      return _stack.top();
    }

    std::map<std::string, dotGraph> dotGraph::graphs;

  }

}

#endif
