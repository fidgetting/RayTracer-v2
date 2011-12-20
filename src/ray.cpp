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
  static int num_run = 0;

  _pixel = _pixel + color();

  if((num_run%1000) == 0)
    std::cout << "." << std::flush;
  num_run++;

  return false;
}

ray::vector ray::l_ray::color() {
  const ray::surface* best = NULL;
  const ray::surface* who  = NULL;
  ray::ray_info info(src(), dir());
  ray::vector ci, cn, inter, norm;
  double d, small;

  small = DBL_MAX;

  for(auto iter = _m->begin(); iter != _m->end(); iter++) {
    who = iter->second->root()->intersection(info, _src, ci, d, cn);

    if(who && d > 0 && d < small) {
      small = d;
      inter = ci;
      best  = who;
      norm  = cn;
    }
  }

  if(best != NULL) {
    return reflectance(
        inter,
        norm,
        best);
  }

  _cont = 0;
  return ray::vector();
}

ray::vector ray::l_ray::reflectance(vector p, vector n, const surface* s) {
  /* locals */
  ray::material mat = _m->mat(s->material());
  ray::vector Lp, Rp, Rl;
  ray::vector ret;
  ray::vector v = _direction;

  /* setup the necessary vectors */
  v.normalize();
  n.normalize();
  v.negate();
  if(v.dot(n) < 0)
    n.negate();

  /* add each light to the color */
  for(auto light = _m->l_begin(); light != _m->l_end(); light++) {
    /* direct of the light source */
    Lp = light->direction(p);
    Lp.normalize();

    /* check for shadowed */
    if(Lp.dot(n) < 0 || shadowed(p, light->direction(p), s)) {
      continue;
    }

    Rl = n * (Lp.dot(n) * 2) - Lp;
    Rl.normalize();

    ret += (mat.diffuse() * light->illumination() * Lp.dot(n)) +
           (light->illumination() * mat.ks() *
               std::pow(std::max(0.0, v.dot(Rl)), mat.alpha()));
  }

  ret = ret * _cont;

  /* recursively calculate new rays */
  Rp = n * (v.dot(n) * 2) - v;
  Rp.normalize();

  if(mat.kt() != 0) {
    ray::l_ray new_ray(_m, p, _direction, _pixel);
    new_ray._src = s;
    new_ray._cont *= mat.kt();
    new_ray();
  }

  _direction = Rp;
  _src_point = p;
  _src       = s;
  _cont      = mat.ks() * _cont;
  _depth++;

  if(_cont > 0.0039 && _depth < MAX_DEPTH &&
      ret[X] < 255 && ret[Y] < 255 && ret[Z] < 255)
    ret += color();

  /* clip the colors */
  ret[0] = std::min(int(ret[0]), 255);
  ret[1] = std::min(int(ret[1]), 255);
  ret[2] = std::min(int(ret[2]), 255);

  return ret;
}

bool ray::l_ray::shadowed(const ray::vector& pt, const ray::vector& U,
    const surface* s) const {
  ray::vector i, n;
  ray::vector tmp = U;
  ray::ray_info info(pt, tmp);
  double d;

  for(auto iter = _m->begin(); iter != _m->end(); iter++) {
    if(iter->second->root()->intersection(info, s, i, d, n)) {
      if(d > 0 && d < U.length()) {
        return true;
      }
    }
  }

  return false;
}

cv::Vec<uc, 3> operator+(const cv::Vec<uc, 3>& rhs, const ray::vector& lhs) {
  cv::Vec<uc, 3> ret;

  ret[2] = std::min(255, int(rhs[2] + lhs[0]));
  ret[1] = std::min(255, int(rhs[1] + lhs[1]));
  ret[0] = std::min(255, int(rhs[0] + lhs[2]));

  return ret;
}
