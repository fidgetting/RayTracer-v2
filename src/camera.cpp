/*
 * Camera.cpp
 *
 *  Created on: Sep 21, 2010
 *      Author: norton
 */

#include <surface.h>
#include <camera.h>
#include <threadpool.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <future>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
using std::get;

#include <condition_variable>
#include <mutex>

#include <boost/timer.hpp>

#define WIND "render"
#define ZBUF "zbuffer"

int ray::camera::x_print = 0;
int ray::camera::y_print = 0;

const int ray::camera::width  = 1024;
const int ray::camera::height = 1024;

bool ray::camera::animation = false;

#ifdef DEBUG
bool ray::camera::print = false;
#endif

/**
 * TODO
 *
 * @param src
 */
ray::camera::camera(ray::model& m) {
  double zdiff = sqrt(pow(m.height(), 2) * pow(m.width(), 2)) + m.depth();

  ray::vector up(0, 1, 0);
  ray::vector at(0, 0, 1);

  /* calculate min and max values */
  _umin = -1.0;
  _umax =  1.0;
  _vmin = -1.0;
  _vmax =  1.0;

  _fp = m.center() + ray::vector(0, 0, 0.25 * zdiff);
  _fl = -1;

  _n = at;
  _u = up.cross(at);
  _v = _n.cross(_u);

  _n.normalize();
  _u.normalize();
  _v.normalize();

  _vrp = _fp + (_n * _fl);

  ray::light l;
  l.illumination() = ray::vector(255, 255, 255);
  l.position()     = _fp;
  m.push_light(l);
  m.push_light(l);
}

/**
 * TODO
 *
 * @param amount
 * @param which
 */
void ray::camera::translate(double amount, axis which) {
  switch(which) {
    case x_axis: _fp = _fp + (_u * amount); break;
    case y_axis: _fp = _fp + (_v * amount); break;
    case z_axis: _fp = _fp + (_n * amount); break;
  }
  _vrp = _fp + (_n * _fl);
}

/**
 * TODO
 *
 * @param amount
 * @param around
 * @param which
 */
void ray::camera::rotate(double amount, ray::vector around, axis which) {
  ray::matrix<4, 4> R, z;
  ray::matrix<4, 4> rotat;
  ray::vector vecs[3];

  /* create R */
  switch(which) {
    case x_axis: vecs[2] = _v; break;
    case y_axis: vecs[2] = _u; break;
    case z_axis: return;       break;
  }

  vecs[0] = vecs[2].normal();
  vecs[1] = vecs[0].cross(vecs[2]);

  vecs[0].normalize();
  vecs[1].normalize();
  vecs[2].normalize();

  R[3][3] = 1;
  for(int i = 0; i < 3; i++)
    for(int j = 0; j < 3; j++)
      R[i][j] = vecs[i][j];

  /* create z */
  z = ray::identity<4>();
  z[0][0] =  std::cos(amount);
  z[0][1] =  std::sin(amount);
  z[1][0] = -std::sin(amount);
  z[1][1] =  std::cos(amount);

  /* finish the rotation matrix */
  rotat = R.t() * z * R;

  /* move the focal point */
  _fp -= around;
  _fp = rotat * _fp;
  _fp += around;

  /* rotate the camera */
  _n   = rotat * _n;
  _u   = rotat * _u;
  _v   = rotat * _v;
  _vrp = _fp + (_n * _fl);
}

/**
 * TODO
 *
 * @param m
 * @param dst
 * @param zbuffer
 */
void ray::camera::draw_proj(model& m, cv::Mat& dst, cv::Mat& zbuffer) {
  std::vector<double> z_values;
  ray::matrix<4, 4> proj = projection();
  ray::matrix<4, 4> rota = rotation();

  double xfact = ((umax() - umin()) / camera::width );
  double yfact = ((vmax() - vmin()) / camera::height);

  for(auto iter = zbuffer.begin<double>(); iter != zbuffer.end<double>(); iter++)
    *iter = DBL_MAX;

  for(auto iter = m.begin(); iter != m.end(); iter++) {
    ray::object obj = *(iter->second);
    obj *= rota;
    z_values = point_z(obj);
    obj *= proj;

    for(unsigned int i = 0; i < obj.v_size(); i++) {
      obj.at(i)[0] = ( (obj.at(i)[0] / obj.at(i)[3]) - _umin) / xfact;
      obj.at(i)[1] = (-(obj.at(i)[1] / obj.at(i)[3]) - _vmin) / yfact;
      obj.at(i)[2] /= obj.at(i)[3];
      obj.at(i)[3] /= obj.at(i)[3];
    }

    iter->second->root()->fill(&m, this, obj, dst, zbuffer, z_values);
  }
}

/**
 * TODO
 *
 * @param obj
 * @return
 */
std::vector<double> ray::camera::point_z(const ray::object& obj) {
  std::vector<double> ret;

  for(unsigned int i = 0; i < obj.v_size(); i++) {
    ret.push_back(-obj[i][2]);
  }

  return ret;
}

/**
 * TODO
 *
 * @param m
 * @return
 */
cv::Mat ray::camera::click(model& m) {
  ray::vector U, L;
  cv::Mat dst = cv::Mat::zeros(height, width, CV_8UC3);

#ifndef DEBUG
  int nthread = std::max(int(std::thread::hardware_concurrency()- 1), 3);
  ray::threadpool workers(nthread, 1000);
#endif

  /* two different versions of this function can be compiled.
   *   1. A debugging version that runs purely in the main thread. This has
   *      the advantage that it can print all the information for a specific
   *      pixel. Simply set X_PRINT and Y_PRINT before compiling, and this
   *      if one checks the print variable, they can get any information
   *      about a particular pixel
   *
   *   2. The standard threaded version. This has evolved over time into
   *      the current version. Currently this will display an image to the
   *      screen and show the rendering process in real time. It will check
   *      the number of threads available on the system. One thread will be
   *      reserved for the display (the main thread). The other threads will
   *      be allocated to the rendering process.
   */

  double xv, yv;
  double xi = (umax() - umin()) / double(width);
  double yi = (vmax() - vmin()) / double(height);

  xv = umin();
  for(int x = 0; x <= width; x++, xv += xi) {
    yv = vmin();
    for(int y = 0; y <= height; y++, yv += yi) {
      L = vrp() + u()*xv - v()*yv;
      U = L - fp(); U.normalize();
#ifdef DEBUG
      if(x == x_print && y == y_print)
        print = true;
      ray::l_ray(&m, L, U,
          dst.at<cv::Vec<uc, 3> >(y, x))();
      if(x == x_print && y == y_print) {
        print = false;
      }
    }
  }
#else
      workers.put(ray::l_ray(&m, L, U,
          dst.at<cv::Vec<uc, 3> >(y, x)));
    }
  }

  workers.join();
#endif

  return dst;
}

ray::matrix<4, 4> ray::camera::projection() const {
  ray::matrix<4, 4> proj = ray::identity<4>();

  proj[3][2] = 1.0/_fl;
  proj[3][3] = 0;

  return proj;
}

ray::matrix<4, 4> ray::camera::rotation() const {
  ray::matrix<4, 4> scal = ray::identity<4>();
  ray::matrix<4, 4> rota = ray::identity<4>();
  ray::matrix<4, 4> tran = ray::identity<4>();

  scal[0][0] = scal[1][1] = scal[2][2] =
      sqrt(pow(width, 2) + pow(height, 2));

  rota[0][0] = _u[0]; rota[0][1] = _u[1]; rota[0][2] = _u[2];
  rota[1][0] = _v[0]; rota[1][1] = _v[1]; rota[1][2] = _v[2];
  rota[2][0] = _n[0]; rota[2][1] = _n[1]; rota[2][2] = _n[2];

  tran[0][3] = -_fp[0];
  tran[1][3] = -_fp[1];
  tran[2][3] = -_fp[2];

  return scal * rota * tran;
}

ray::display::display(ray::model& m, ray::camera& cam, bool wire) :
    _m(&m), _cam(&cam), _rot(m.center()), _wire(wire), last_x(-1), last_y(-1),
    _state(none),
    image  (camera::height, camera::width, CV_8UC3),
    zbuffer(camera::height, camera::width, CV_64FC1) {
  cv::namedWindow(WIND, CV_WINDOW_AUTOSIZE);
  cvSetMouseCallback(WIND, ray::display::mouse, this);

  gettimeofday(&last_t, NULL);
  dir = cam.fl() < 0 ? -1 : 1;
}

void ray::display::show() {

  for(auto iter = image.begin<cv::Vec<uc, 3> >();
      iter != image.end<cv::Vec<uc, 3> >(); iter++) {
    *iter = cv::Vec<uc, 3>();
  }

  _cam->draw_proj(*_m, image, zbuffer);
  cv::imshow(WIND, image);
}

void ray::display::exec() {
  boost::timer watch;
  cv::Mat ray_render;
  state_t tmp = ray;

  for(;;) {
    show();
    cv::waitKey(-1);

    std::swap(tmp, _state);

    watch.restart();
    std::cout << "DISPLAY: starting render " << std::flush;
    ray_render = _cam->click(*_m);
    std::cout << "finished [" << watch.elapsed() << "]" << std::endl;

    cv::imshow(WIND, ray_render);
    cv::waitKey(-1);

    std::swap(tmp, _state);
  }
}

void ray::display::mouse(int event, int x, int y, int flags, void* disp) {
  ray::display* data = (ray::display*)disp;

  switch(event) {
    case move:   data->move_e(x, y);    break;
    case l_down: data->left_e(true);    break;
    case r_down: data->right_e(true);   break;
    case m_down: data->middle_e(true);  break;
    case l_up:   data->left_e(false);   break;
    case r_up:   data->right_e(false);  break;
    case m_up:   data->middle_e(false); break;
  }
}

void ray::display::move_e(int x, int y) {
  struct timeval curr;
  long seconds, useconds;
  ray::vector diff;

  double xmove = _m->width()  / 500.0;
  double ymove = _m->height() / 500.0;
  double zmove = _m->depth()  / 20.0;

  /* this function can get overloaded      */
  /* so we calculate  time since last call */
  gettimeofday(&curr, NULL);
  seconds  = curr.tv_sec  - last_t.tv_sec;
  useconds = curr.tv_usec - last_t.tv_usec;
  if(((seconds * 1000 + useconds/1000.0) < 50) || _state == ray) {
    return;
  }

  switch(_state) {
    case none: /* do nothing */ break;
    case left:
      _cam->rotate((x - last_x) * 0.0174 * dir, _rot, ray::camera::x_axis);
      _cam->rotate((y - last_y) * 0.0174 * dir, _rot, ray::camera::y_axis);
      break;
    case right:
      _cam->translate( (x - last_x) * xmove * dir, ray::camera::x_axis);
      _cam->translate(-(y - last_y) * ymove * dir, ray::camera::y_axis);
      break;
    case middle:
      _cam->translate((y - last_y) * zmove, ray::camera::z_axis);
      break;
    case ray:
      break;
  }

  show();

  last_x = x;
  last_y = y;
  gettimeofday(&last_t, NULL);
}

void ray::display::left_e(bool down) {
  if(_state == none && down) {
    _state = left;
  } else if(_state == left && !down) {
    _state = none;
  }
}

void ray::display::right_e(bool down) {
  if(_state == none && down) {
    _state = right;
  } else if(_state == right && !down) {
    _state = none;
  }
}

void ray::display::middle_e(bool down) {
  if(_state == none && down) {
    _state = middle;
  } else if(_state == middle && !down) {
    _state = none;
  }
}
