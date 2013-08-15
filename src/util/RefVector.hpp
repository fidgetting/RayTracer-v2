/*
 * MatrixReference.tpp
 *
 *  Created on: Jan 24, 2013
 *      Author: norton
 */

#pragma once

/* local includes */
#include <Matrix.tpp>
#include <Vector.hpp>

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

template<typename T1, typename P1, typename T2, typename P2>
inline P1 max_pair(T1 lt, P1 lp, T2 rt, P2 rp) { return lt > rt ? lp : rp; }

template<typename T1, typename P1, typename T2, typename P2, typename... Args>
inline P1 max_pair(T1 lt, P1 lp, T2 rt, P2 rp, Args... rest) {
  return lt > rt ? max_pair(lt, lp, rest...) : max_pair(rt, rp, rest...);
}

template<typename T1, typename P1, typename T2, typename P2>
inline P1 min_pair(T1 lt, P1 lp, T2 rt, P2 rp) { return lt < rt ? lp : rp; }

template<typename T1, typename P1, typename T2, typename P2, typename... Args>
inline P1 min_pair(T1 lt, P1 lp, T2 rt, P2 rp, Args... rest) {
  return lt < rt ? min_pair(lt, lp, rest...) : min_pair(rt, rp, rest...);
}

namespace ray {

  class RefVector {
    public:

      RefVector(const double* data);
      RefVector(const Matrix<double>& mat, uint16_t idx);

      /* getters */
      inline double x() const { return data[0]; }
      inline double y() const { return data[1]; }
      inline double z() const { return data[2]; }
      inline double w() const { return data[3]; }

      inline double operator[](int i) const { return data[i]; }

      /* operations */
      Vector negate()    const;
      Vector normalize() const;

      double distance(const Vector& rhs) const;
      double length  ()                  const;

    private:

      const double* data;
  };

  Vector operator *(const Matrix<double>& lhs, const Vector& rhs);

}





