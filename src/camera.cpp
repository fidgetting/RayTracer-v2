/*
 * Camera.cpp
 *
 *  Created on: Sep 21, 2010
 *      Author: norton
 */

#include <camera.h>
#include <queue.tpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
using std::get;

#include <cv.h>
#include <cvaux.h>
#include <cxcore.h>
#include <highgui.h>

#define X_PRINT 0
#define Y_PRINT 0

#ifdef DEBUG
bool camera::print = false;
#else
int ray::camera::running = 0;

ray::concurrent_queue<ray::l_ray> rays;
std::condition_variable_any wait_on;
std::mutex                  lock_on;
unsigned int                numb_on;
int                         block_on;

void wrapper() {
  rays.worker();
  ray::camera::running--;
}

#endif

/**
 *
 * @param m
 * @return
 */
void ray::camera::click(model* m) {
  /* locals */
  cv::Mat raw_image(vmax() - vmin() + 1, umax() - umin() + 1, CV_8UC3);
  std::vector<std::thread*> threads;
  ray::vector U, L;

  numb_on = 1;
  block_on = 1;

  for(int x = umin(); x <= umax(); x++) {
    for(int y = vmin(); y <= vmax(); y++) {
      L = vrp() + u()*x + v()*y;
      U = L - fp(); U.normalize();

#ifdef DEBUG
      if(x == X_PRINT && y == Y_PRINT) {
        print = true;
      }
      l_ray(m, this, L, U,
          raw_image.at<Vector<3, uc> >(vmax() - y, x - umin()))();
      if(x == X_PRINT && y == Y_PRINT) {
        print = false;
      }
#else
      rays.push(
          new l_ray(m, this, L, U,
              raw_image.at<cv::Vec<uc, 3> >(vmax() - y, x - umin())));
#endif
    }
  }

#ifndef DEBUG
  std::random_shuffle(rays.begin(), rays.end());
  for(unsigned int i = 0; i < /*std::thread::hardware_concurrency()*/2 - 1; i++) {
    running++;
    //threads.push_back(new std::thread(wrapper));
  }

  while(running) {
    cv::imshow("win", raw_image);
    if(numb_on == std::thread::hardware_concurrency()) {
      cv::waitKey(0);
      block_on++;
      wait_on.notify_all();
      while(numb_on != 1)
        usleep(100);
    }

    cv::waitKey(30);
  }

  for(unsigned int i = 0; i < /*std::thread::hardware_concurrency()*/2 - 1; i++) {
    threads[i]->join();
  }
#endif

#ifdef DEBUG
  Vector<3, uc> fill(255);
  out_image.at<Vector<3, uc> >(X_PRINT - umin() - 1, Y_PRINT - vmin() - 1) = fill;
  out_image.at<Vector<3, uc> >(X_PRINT - umin() - 1, Y_PRINT - vmin() + 1) = fill;
  out_image.at<Vector<3, uc> >(X_PRINT - umin() - 1, Y_PRINT - vmin()    ) = fill;
  out_image.at<Vector<3, uc> >(X_PRINT - umin()    , Y_PRINT - vmin() - 1) = fill;
  out_image.at<Vector<3, uc> >(X_PRINT - umin()    , Y_PRINT - vmin() + 1) = fill;
  out_image.at<Vector<3, uc> >(X_PRINT - umin() + 1, Y_PRINT - vmin() - 1) = fill;
  out_image.at<Vector<3, uc> >(X_PRINT - umin() + 1, Y_PRINT - vmin() + 1) = fill;
  out_image.at<Vector<3, uc> >(X_PRINT - umin() + 1, Y_PRINT - vmin()    ) = fill;
#endif

  /* create the output image */
  cv::imwrite("output.png", raw_image);
  cv::imshow("win", raw_image);
  cv::waitKey(-1);
}

/**
 *
 * @param m
 * @param ray
 * @param p
 * @param cont
 * @return
 */
ray::vector ray::camera::color(ray::l_ray* r) const {
  std::tuple<ray::vector, double, const surface*> i, t;
  get<1>(i) = std::numeric_limits<double>::infinity();
  double cont = r->cont();

  for(auto iter = r->world()->begin(); iter != r->world()->end(); iter++) {
    for(auto oi = iter->second->begin(); oi != iter->second->end(); oi++) {
      t = oi->intersection(r->dir(), r->src(), r->surf());

      if(get<1>(t) > 0 && get<1>(t) < get<1>(i)) {
        i = t;
      }
    }
  }

  if(get<2>(i) != NULL) {
    return reflectance(
        r,
        get<0>(i),
        get<2>(i)->normal(get<0>(i)),
        r->world()->mat(get<2>(i)->material()),
        get<2>(i)) * cont;
  }

  r->cont() = 0;
  return ray::vector();
}

/**
 * v = r->dir()
 *
 * @param m
 * @param p
 * @param n
 * @param mat
 * @return
 */
ray::vector ray::camera::reflectance(ray::l_ray* r, ray::vector p,
    ray::vector n, const material& mat, const surface* s) const {
  ray::vector Lp, Rp(4), Rl(4);
  ray::vector ret;
  ray::vector v = r->dir();
  v.normalize();
  n.normalize();
  v.negate();

  if(v.dot(n) < 0) {
    n.negate();
  }

  /* add each light the red, green and blue values */
  for(auto light = r->world()->l_begin(); light != r->world()->l_end(); light++) {
    /* calculate the direction of the light source and angle of reflectance*/
    Lp = light->direction(p); Lp.normalize();
    /* calculate the actual reflectance values */
    if(Lp.dot(n) < 0 || shadowed(p, light->direction(p), r->world(), s)) {
      continue;
    }

    Rl = n * (Lp.dot(n) * 2) - Lp; Rl.normalize();
    ret += (mat.diffuse() * light->illumination() * Lp.dot(n)) +
           (light->illumination() * mat.ks() * std::pow(std::max(0.0, v.dot(Rl)), mat.alpha()));
  }

  /* recursively calculate new rays */
  Rp = n * (v.dot(n) * 2) - v; Rp.normalize();
  r->dir()  = Rp;
  r->src()  = p;
  r->surf() = s;
  r->cont() = mat.ks() * r->cont();
  r->depth()++;

  /* clip the colors */
  ret[0] = std::min(int(ret[0]), 255);
  ret[1] = std::min(int(ret[1]), 255);
  ret[2] = std::min(int(ret[2]), 255);

  return ret;
}

/**
 * TODO
 *
 * @param pt
 * @param l
 * @param m
 * @return
 */
bool ray::camera::shadowed(const ray::vector& pt, const ray::vector& U,
    const model* m, const surface* s) const {
  std::tuple<ray::vector, double, const surface*> inter;
  ray::vector tmp = U;
  tmp.normalize();

  for(auto iter = m->begin(); iter != m->end(); iter++) {
    for(auto oi = iter->second->begin(); oi != iter->second->end(); oi++) {
      inter = oi->intersection(tmp, pt, s);

      if(get<1>(inter) > 0 && get<1>(inter) < U.length()) {
        return true;
      }
    }
  }

  return false;
}
