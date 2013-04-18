/*
 * Ray.cpp
 *
 *  Created on: Dec 31, 2012
 *      Author: norton
 */

/* local includes */
#include <Ray.hpp>
#include <Surface.hpp>

namespace ray {

  Ray::Ray() : _source(nullptr) { }

  /**
   * Constructs a Ray out of the compontent Vectors.
   *
   * @param U  The direction that the ray is traveling
   * @param L  The origin of the Ray
   */
  Ray::Ray(const Vector& L, const Vector& U, const Surface* source):
      _L(L),
      _U(U),
      _iU(1.0 / U.x(), 1.0 / U.y(), 1.0 / U.z()),
      _iL(1.0 / L.x(), 1.0 / L.y(), 1.0 / L.z()),
      _positive({U.x()  > 0.0, U.y()  > 0.0, U.z()  > 0.0}),
      _nonzero ({U.x() != 0.0, U.y() != 0.0, U.z() != 0.0}),
      _source(source) { }

  Ray::operator ray::render::d_Ray() const {
    render::d_Ray ret;

    ret.L  = _L;
    ret.U  = _U;
    ret.iL = _iL;
    ret.iU = _iU;
    ret.positive[0] = _positive[0];
    ret.positive[1] = _positive[1];
    ret.positive[2] = _positive[2];
    ret.nonzero [0] = _nonzero [0];
    ret.nonzero [1] = _nonzero [1];
    ret.nonzero [2] = _nonzero [2];
    ret.src      = -1;

    return ret;
  }

  /**
   * Chooses the best Intersection. The best Intersection is the closest in this
   * case.
   *
   * @param l  an Intersection of a Surface and a Ray
   * @param r  an Intersection of a Surface and a Ray
   * @return   the best of the two Intersections
   */
  Intersection Intersection::best(const Intersection& l, const Intersection& r) {
    return l._distance < r._distance ? l : r;
  }

  std::ostream& operator<<(std::ostream& ostr, const Ray& ray) {
    if(ray.source() != nullptr) {
      ostr << "RAY[L:" << ray.L() << " U:" << ray.U() << " SRC:" << ray.source()->id << "]";
    } else {
      ostr << "RAY[L:" << ray.L() << " U:" << ray.U() << "]";
    }
    return ostr;
  }

  std::ostream& operator<<(std::ostream& ostr, const Intersection& inter) {
    ostr << "INTER[i:"
        << inter.i()
        << " n: " << inter.n() << " v: "
        << inter.v() << " id:"
        << (inter.source() == nullptr ? -1 : inter.source()->id) << "]";
    return ostr;
  }
}

