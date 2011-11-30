/*
 * Surface.cpp
 *
 *  Created on: Oct 25, 2010
 *      Author: norton
 */

#include <surface.h>
#include <camera.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
using std::get;

int ray::surface::id_gen = 0;

ray::vector ray::sphere::normal(const ray::vector& v) const {
  return v - _center;
}

/**
 * Calculate the intersection between a ray and a sphere.
 *
 * @param U
 * @param L
 * @param skip
 * @return
 */
std::tuple<ray::vector, double, const ray::surface*>
ray::sphere::intersection(const vector& U, const vector& L, const surface* skip) const {
  std::tuple<ray::vector, double, const surface*> i, tmp;
  double s, t_sq, r_sq, m_sq, q;
  ray::vector T = center() - L;

  get<1>(i) = std::numeric_limits<double>::infinity();
  get<2>(i) = NULL;

  if(skip == this && U.dot(normal(L)) > 0) {
    return i;
  }

  /* perform first check to see if we hit the circle */
  s = T.dot(U);
  t_sq = T.dot(T);
  r_sq = radius() * radius();
  if(s < 0 && t_sq > r_sq) {
    return i;
  }

  /* second easy rejection check */
  m_sq = t_sq - s*s;
  if(m_sq > r_sq) {
    return i;
  }

  /* we now know that the ray will intersect the sphere, fork for subsurfaces */
  if(_subsurfaces.size() == 0) {
    q = sqrt(r_sq - m_sq);
    if(t_sq > r_sq && (this != skip || U.dot(normal(L)) >= 0)) {
      s -= q;
    } else {
      s += q;
    }

    return std::tuple<ray::vector, double, const surface*>(L + (U * s), s, this);
  }

  /* this sphere does have subsurfaces, find the closest and return its intersection */
  for(auto iter = _subsurfaces.begin(); iter != _subsurfaces.end(); iter++) {
    if(*iter != skip) {
      tmp = (*iter)->intersection(U, L, skip);

      if(get<1>(tmp) >= 0 && get<1>(tmp) < get<1>(i)) {
        i = tmp;
      }
    }
  }

  return i;
}

void ray::polygon::set_normal() {
  if(_indeces.size() > 2) {
    ray::vector a = (*_owner)[_indeces[0]];
    ray::vector b = (*_owner)[_indeces[1]];
    ray::vector c = (*_owner)[_indeces[2]];
    _n = (a - b).cross(c - b);
    _n.normalize();
    _n.negate();
  }
}

/**
 * virtual function overloaded from the surface class. This looks a little odd
 * since it has the "v == v" in it, but that is to prevent compiler warnings.
 * This does not get called much so it should not created too much overhead.
 *
 * @param v not really used
 * @return the normal to the surface
 */
ray::vector ray::polygon::normal(const vector& v) const {
  return v == v ? _n : _n;
}

/**
 * Calculates the intersection of a ray and a polygon. This function uses
 * Gaussian elimination to determine if the ray intersected the polygon.
 *
 * TODO
 *
 * @param U
 * @param L
 * @param skip
 * @return
 */
std::tuple<ray::vector, double, const ray::surface*>
ray::polygon::intersection(const vector& U, const vector& L, const surface* skip) const {
  matrix<3, 4> m;
  vector A, B, C, u, v, n, w, I;
  double r, a, b, uu, uv, vv, wu, wv, D, gama, beta;

  A = (*this)[*begin()];

  if(skip != this) {
    auto s = this->end() - 1;
    for(auto e = this->begin(); e != this->end(); e++) {
      B = (*this)[*e];
      C = (*this)[*s];
      s = e;

      /*m[0][0] = A[0] - B[0]; m[0][1] = A[0] - C[0]; m[0][2] = U[0]; m[0][3] = A[0] - L[0];
      m[1][0] = A[1] - B[1]; m[1][1] = A[1] - C[1]; m[1][2] = U[1]; m[1][3] = A[1] - L[1];
      m[2][0] = A[2] - B[2]; m[2][1] = A[2] - C[2]; m[2][2] = U[2]; m[2][3] = A[2] - L[2];
      m.gaussian_elimination();

      if(m[0][3] >= 0 && m[1][3] >= 0 && m[2][3] >= 0 && m[0][3] + m[1][3] < 1) {
        return std::tuple<ray::vector, double, const surface*>
          (L + (U * m[2][3]), m[2][3], this);
      }*/

      u = B - A;
      v = C - A;
      n = u.cross(v);

      a = -n.dot(L - A);
      b =  n.dot(U);
      if(fabs(b) < 0.00000001)
        continue;

      if((r = a / b) < 0.0)
        continue;

      I = L + (U * r);
      uu = u.dot(u);
      uv = u.dot(v);
      vv = v.dot(v);
      w = I - A;
      wu = w.dot(u);
      wv = w.dot(v);
      D = uv * uv - uu * vv;

      gama = (uv * wv - vv * wu) / D;
      if(gama < 0.0 || gama > 1.0)
        continue;
      beta = (uv * wu - uu * wv) / D;
      if(beta < 0.0 || (gama + beta) > 1.0)
        continue;

      return std::tuple<ray::vector, double, const surface*>(I, r, this);
    }
  }

  return std::tuple<vector, double, const surface*>
    (vector(), -1, (const surface*)NULL);
}

/**
 * Finds the center of the polygon, this is used when the polygons are put
 * inside spheres since it will give close to the best center for the polygon
 *
 * @return a point that should be close to the center of the polygon
 */
ray::vector ray::polygon::center() const {
  ray::vector center;

  for(const_iterator iter = begin(); iter != end(); iter++) {
    center += this->get(iter);
  }

  center /= size();
  return center;
}

/**
 * Finds the radius of the polygon. This simply returns the distance from the
 * center of the polygon to the most distant point on the polygon.
 *
 * @return decimal radius of the polygon
 */
double ray::polygon::radius() const {
  double rad = 0, d;
  ray::vector c = center();

  for(const_iterator iter = begin(); iter != end(); iter++) {
    d = c.distance(*iter);
    rad = std::max(d, rad);
  }

  return rad;
}
