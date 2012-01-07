/*
 * ray.cpp
 *
 *  Created on: Sep 14, 2011
 *      Author: norton
 */

#include <camera.h>
#include <model.h>
#include <ray.h>
#include <surface.h>

#include <algorithm>
#include <limits>
#include <tuple>
using std::get;

bool ray::l_ray::operator()() {
  _pixel = _pixel + intersect();

  return false;
}

ray::vector ray::l_ray::intersect() {
  const ray::surface* best = NULL;
  const ray::surface* who  = NULL;
  ray::ray_info info(src(), dir());
  ray::vector ci, cn, inter, n, color;
  ray::vector Rp, v;
  ray::material mat;
  double d, small;

  small = DBL_MAX;

  for(auto iter = _m->begin(); iter != _m->end(); iter++) {
    who = iter->second->root()->intersection(info, _src, ci, d, cn);

    if(who && d > 0 && d < small) {
      small = d;
      inter = ci;
      best  = who;
      n     = cn;
    }
  }

  if(best != NULL) {
    v = _direction;
    v.negate();
    mat = _m->mat(best->material());

    color = _m->reflectance(inter, v, n, best) * _cont;

    /* recursively calculate new rays */
    Rp = n * (v.dot(n) * 2) - v;
    Rp.normalize();

    if(mat.kt() != 0) {
      ray::l_ray new_ray(_m, inter, _direction, _pixel);
      new_ray._src = best;
      new_ray._cont *= mat.kt();
      new_ray();
    }

    _direction = Rp;
    _src_point = inter;
    _src       = best;
    _cont      = mat.ks() * _cont;
    _depth++;

    if(_cont > 0.0039 && _depth < MAX_DEPTH &&
        color[X] < 255 && color[Y] < 255 && color[Z] < 255)
      color += intersect();

    /* clip the colors */
    color[0] = std::min(int(color[0]), 255);
    color[1] = std::min(int(color[1]), 255);
    color[2] = std::min(int(color[2]), 255);
  }

  return color;
}

cv::Vec<uc, 3> operator+(const cv::Vec<uc, 3>& rhs, const ray::vector& lhs) {
  cv::Vec<uc, 3> ret;

  ret[2] = std::min(255, int(rhs[ray::Z] + lhs[ray::X]));
  ret[1] = std::min(255, int(rhs[ray::Y] + lhs[ray::Y]));
  ret[0] = std::min(255, int(rhs[ray::X] + lhs[ray::Z]));

  return ret;
}
