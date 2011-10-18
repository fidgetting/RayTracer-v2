/*
 * ray.cpp
 *
 *  Created on: Sep 14, 2011
 *      Author: norton
 */

#include <camera.h>
#include <model.h>
#include <ray.h>

#include <algorithm>
#include <limits>
#include <tuple>
using std::get;

bool ray::l_ray::operator()() {
#ifdef BLOCKING
  if(_depth == block_on) {
    std::unique_lock<std::mutex> lock(lock_on);
    numb_on++;
    wait_on.wait(lock);
    numb_on--;
  }
#endif
  _pixel = _pixel + color();
  _pixel[0] = std::min(int(_pixel[0]), 255);
  _pixel[1] = std::min(int(_pixel[1]), 255);
  _pixel[2] = std::min(int(_pixel[2]), 255);

  return !(_cont < 0.0039 || _depth > MAX_DEPTH ||
      (_pixel[0] == 255 && _pixel[1] == 255 && _pixel[2] == 255));
}

ray::vector ray::l_ray::color() {
  std::tuple<ray::vector, double, const surface*> i, t;
  get<1>(i) = std::numeric_limits<double>::infinity();

  for(auto iter = _m->begin(); iter != _m->end(); iter++) {
    for(auto oi = iter->second->begin(); oi != iter->second->end(); oi++) {
      t = oi->intersection(_direction, _src_point, _src);

      if(get<1>(t) > 0 && get<1>(t) < get<1>(i)) {
        i = t;
      }
    }
  }

  if(get<2>(i) != NULL) {
    return reflectance(
        get<0>(i),
        get<2>(i)->normal(get<0>(i)),
        get<2>(i)) * _cont;
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

  /* recursively calculate new rays */
  Rp = n * (v.dot(n) * 2) - v;
  Rp.normalize();
  _direction = Rp;
  _src_point = p;
  _src       = s;
  _cont      = mat.ks() * _cont;
  _depth++;

  /* clip the colors */
  ret[0] = std::min(int(ret[0]), 255);
  ret[1] = std::min(int(ret[1]), 255);
  ret[2] = std::min(int(ret[2]), 255);

  return ret;
}

bool ray::l_ray::shadowed(const ray::vector& pt, const ray::vector& U,
    const surface* s) const {
  std::tuple<ray::vector, double, const surface*> inter;
  ray::vector tmp = U;
  tmp.normalize();

  for(auto iter = _m->begin(); iter != _m->end(); iter++) {
    for(auto oi = iter->second->begin(); oi != iter->second->end(); oi++) {
      inter = oi->intersection(tmp, pt, s);

      if(get<1>(inter) > 0 && get<1>(inter) < U.length()) {
        return true;
      }
    }
  }

  return false;
}

cv::Vec<uc, 3> operator+(const cv::Vec<uc, 3>& rhs, const ray::vector& lhs) {
  cv::Vec<uc, 3> ret;

  ret[0] = rhs[0] + lhs[0];
  ret[1] = rhs[1] + lhs[1];
  ret[2] = rhs[2] + lhs[2];

  return ret;
}
