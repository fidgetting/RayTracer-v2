/*
 * Vector.h
 *
 *  Created on: Dec 24, 2012
 *      Author: norton
 */

#pragma once

#define VECTOR_SIZE 4

#define EPSILON 1.0e-10
#define BRANCHING_FACTOR  3

/* std includes */
#include <iostream>

#ifdef __CUDACC__
#define CUDA_CALL __host__ __device__
#else
#define CUDA_CALL
#define __host__
#define __device__
#define __global__

/* optional local includes */
#include <Debug.hpp>

/* optional std includes */
#include <cstdlib>
#include <cstring>
#include <cmath>

#define cudaFree(a) ::free(a);
#define cudaMemcpy(a, b, c, d) ::memcpy(a, b, c);

inline void cudaMalloc(void** out, size_t size) {
  (*out) = (void*)malloc(size);
}

#endif

namespace ray {

  class RefVector;

  class Vector {
    public:

      CUDA_CALL Vector();
      CUDA_CALL Vector(double d);
      CUDA_CALL Vector(double x, double y, double z, double w = 1.0);
      __host__  Vector(const RefVector& vec);

      /* getters */
      CUDA_CALL inline double x() const { return data[0]; }
      CUDA_CALL inline double y() const { return data[1]; }
      CUDA_CALL inline double z() const { return data[2]; }
      CUDA_CALL inline double w() const { return data[3]; }

      CUDA_CALL inline double operator [](int idx) const { return data[idx]; }

      /* operations */
      CUDA_CALL Vector negate    () const;
      CUDA_CALL Vector normalize () const;

      CUDA_CALL Vector getPerpendicular() const;

      CUDA_CALL double distance (const Vector& rhs) const;
      CUDA_CALL double length   ()                  const;

    private:

      /** the double cooridnates of the Vector */
      double data[VECTOR_SIZE];
  };

  CUDA_CALL Vector operator +(const Vector& lhs, const Vector& rhs);
  CUDA_CALL Vector operator +(const Vector& lhs, const double   rhs);
  CUDA_CALL Vector operator -(const Vector& lhs, const Vector& rhs);
  CUDA_CALL Vector operator -(const Vector& lhs, const double   rhs);
  CUDA_CALL Vector operator *(const Vector& lhs, const Vector& rhs);
  CUDA_CALL Vector operator *(const Vector& lhs, const double   rhs);
  CUDA_CALL Vector operator /(const Vector& lhs, const Vector& rhs);
  CUDA_CALL Vector operator /(const Vector& lhs, const double   rhs);

  CUDA_CALL Vector max(const Vector& lhs, const Vector& rhs);
  CUDA_CALL Vector min(const Vector& lhs, const Vector& rhs);
  CUDA_CALL Vector max(const Vector& lhs, const double&  rhs);
  CUDA_CALL Vector min(const Vector& lhs, const double&  rhs);

  __host__ std::ostream& operator<<(std::ostream& ostr, const Vector& vec);

  /* ************************************************************************ */
  /* ************************************************************************ */
  /* ************************************************************************ */

  /**
   * Calculates the cross product of 2 Vectors. This is templated so that it can
   * be used on either Vectors or RefVectors
   *
   * @param lhs  left hand side of the cross product
   * @param rhs  right hand side of the cross product
   * @return     the cross product of the two Vectors
   */
  template<typename T1, typename T2>
  CUDA_CALL Vector cross(const T1& lhs, const T2& rhs) {
    return Vector(
        lhs.y() * rhs.z() - lhs.z() * rhs.y(),
        lhs.z() * rhs.x() - lhs.x() * rhs.z(),
        lhs.x() * rhs.y() - lhs.y() * rhs.x());
  }

  /**
   * Calculates the dot product of 2 Vectors. This is templated so that it can
   * be used on either Vectors or RefVectors
   *
   * @param lhs  left hand side of the dot product
   * @param rhs  right hand side of the dot product
   * @return     the dot product of the two Vectors
   */
  template<typename T1, typename T2>
  CUDA_CALL double dot(const T1& lhs, const T2& rhs) {
    return
        lhs.x() * rhs.x() +
        lhs.y() * rhs.y() +
        lhs.z() * rhs.z();
  }

  inline CUDA_CALL double max(const double& lhs, const double& rhs) {
    return lhs > rhs ? lhs : rhs;
  }

  inline CUDA_CALL double min(const double& lhs, const double& rhs) {
    return lhs < rhs ? lhs : rhs;
  }
}
