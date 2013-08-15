/*
 * RefVector.hpp
 *
 *  Created on: Aug 15, 2013
 *      Author: norton
 */

#pragma once

namespace detail {

  template<typename T1, typename T2>
  inline uint32_t _max_index(uint32_t lc, uint32_t rc, T1 l, T2 r) {
    return l > r ? lc : rc;
  }

  template<typename T1, typename T2, typename... Args>
  inline uint32_t _max_index(uint32_t lc, uint32_t rc, T1 l, T2 r, Args... rest) {
    return l > r ?
        _max_index(lc, rc + 1, l, rest...) :
        _max_index(rc, rc + 1, r, rest...);
  }

  template<typename T1, typename T2>
  inline uint32_t _min_index(uint32_t lc, uint32_t rc, T1 l, T2 r) {
    return l < r ? lc : rc;
  }

  template<typename T1, typename T2, typename... Args>
  inline uint32_t _min_index(uint32_t lc, uint32_t rc, T1 l, T2 r, Args... rest) {
    return l < r ?
        _min_index(lc, rc + 1, l, rest...) :
        _min_index(rc, rc + 1, r, rest...);
  }

}

template<typename T1, typename T2>
inline T1 max(T1 l, T2 r) { return l > r ? l : r; }

template<typename T1, typename T2, typename... Args>
inline T1 max(T1 l, T2 r, Args... rest) {
  return l > r ? max(l, rest...) : max(r, rest...);
}

template<typename T1, typename T2>
inline T1 min(T1 l, T2 r) { return l < r ? l : r; }

template<typename T1, typename T2, typename... Args>
inline T1 min(T1 l, T2 r, Args... rest) {
  return l < r ? min(l, rest...) : min(r, rest...);
}

template<typename... Args>
inline uint32_t max_index(Args... args) {
  return detail::_max_index(0, 1, args...);
}

template<typename... Args>
inline uint32_t min_index(Args... args) {
  return detail::_min_index(0, 1, args...);
}

