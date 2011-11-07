/*
 * Surface.cpp
 *
 *  Created on: Oct 25, 2010
 *      Author: norton
 */

#include <surface.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
using std::get;

int ray::surface::id_gen = 0;

/**
 * Check if two spheres are close enough together that we can justify placing them under the
 * same super sphere. A Sphere is close to this sphere if the distance between their centers
 * is smaller than their summed radius's.
 *
 * @param oth the sphere to compare this sphere to
 * @return true if they are close, false otherwise
 */
/*bool sphere::close(const sphere& oth) const {
  return (oth.center().distance(center()) < oth.radius() + radius());
}*/

/**
 * Calculate the intersection between a ray and a sphere.
 *
 * @param U
 * @param L
 * @param skip
 * @return
 */
/*tuple<point, double, const surface*> sphere::intersection(const Vector<3>& U, const point& L, const surface* skip) const {
  tuple<point, double, const surface*> i(point(0), numeric_limits<double>::infinity(), (const surface*)NULL), tmp;
  double s, t_sq, r_sq, m_sq, q;
  Vector<3> T = center() - L;

  if(skip == this && U.dot(normal(L)) > 0) {
    return i;
  }

   TODO recomment: perform first check to see if we hit the circle
  s = T.dot(U);
  t_sq = T.dot(T);
  r_sq = radius() * radius();
  if(s < 0 && t_sq > r_sq) {
    return i;
  }

   TODO recomment: second easy rejection check
  m_sq = t_sq - s*s;
  if(m_sq > r_sq) {
    return i;
  }

   TODO recomment: we now know that the ray will intersect the sphere, fork for subsurfaces
  if(_subsurfaces.size() == 0) {
    q = sqrt(r_sq - m_sq);
    if(t_sq > r_sq && (this != skip || U.dot(normal(L)) >= 0)) {
      s -= q;
    } else {
      s += q;
    }

    return tuple<point, double, const surface*>(L + s*U, s, this);
  }

   TODO recomment: this sphere does have subsurfaces, find the closest and return its intersection
  for(auto iter = _subsurfaces.begin(); iter != _subsurfaces.end(); iter++) {
    if(*iter != skip) {
      tmp = (*iter)->intersection(U, L, skip);

      if(get<1>(tmp) >= 0 && get<1>(tmp) < get<1>(i)) {
        i = tmp;
      }
    }
  }

  return i;
}*/

void ray::polygon::set_normal() {
  if(_indeces.size() > 2) {
    ray::vector a = (*_owner)[_indeces[0]];
    ray::vector b = (*_owner)[_indeces[1]];
    ray::vector c = (*_owner)[_indeces[2]];
    _n = (a - b).cross(c - b);
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
  vector A, B, C;

  A[0] = (*this)[0][0];
  A[1] = (*this)[0][1];
  A[2] = (*this)[0][2];

  if(skip != this) {
    for(int i = 1; i < size() - 1; i++) {
      B = (*this)[i];
      C = (*this)[i + 1];


      m[0][0] = A[0] - B[0]; m[0][1] = A[0] - C[0]; m[0][2] = U[0]; m[0][3] = A[0] - L[0];
      m[1][0] = A[1] - B[1]; m[1][1] = A[1] - C[1]; m[1][2] = U[1]; m[1][3] = A[1] - L[1];
      m[2][0] = A[2] - B[2]; m[2][1] = A[2] - C[2]; m[2][2] = U[2]; m[2][3] = A[2] - L[2];
      m.gaussian_elimination();

      vector tmp = U * L;
      if(m[0][3] >= 0 && m[1][3] >= 0 && m[2][3] >= 0 && m[0][3] + m[1][3] < 1) {
        return std::tuple<ray::vector, double, const surface*>
          (L + U, m[2][3], this);
      }
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
