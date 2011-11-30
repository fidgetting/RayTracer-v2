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

#include <condition_variable>
#include <mutex>

#define X_PRINT 0
#define Y_PRINT 0

#define WIND "render"
#define ZBUF "zbuffer"

#ifdef DEBUG
bool ray::camera::print = false;
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
 * TODO
 *
 * @param src
 */
ray::camera::camera(const obj::objstream::camera& src) :
    _fp(src.fp()), _n(src.vpn()), _u(src.vup().cross(src.vpn())), _fl(src.d()) {
  _v = _n.cross(_u);

  _n.normalize();
  _u.normalize();
  _v.normalize();

  _vrp = _fp + (_n * _fl);
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
  _vrp = _fp + (_n * (-_fl));
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
  ray::matrix<4, 4> trans;
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

  /* create translate to */
  trans = ray::identity<4>();
  trans[0][3] = around[0];
  trans[1][3] = around[1];
  trans[2][3] = around[2];

  R = rotat * trans;

  /* create translate back */
  trans[0][3] = -around[0];
  trans[1][3] = -around[1];
  trans[2][3] = -around[2];

  /* move the camera */
  _fp  =  (trans * R) * _fp;
  _n   = rotat * _n;
  _u   = rotat * _u;
  _v   = rotat * _v;
  _vrp = _fp + (_n * (-_fl));
}

/**
 * TODO
 *
 * @param m
 * @param dst
 */
void ray::camera::draw_wire(model* m, cv::Mat& dst) {
  ray::matrix<4, 4> proj = projection();
  ray::matrix<4, 4> rota = rotation();

  for(auto iter = m->begin(); iter != m->end(); iter++) {
    ray::object obj = *(iter->second);

    obj *= rota;
    obj *= proj;

    for(unsigned int i = 0; i < obj.size(); i++) {
      obj.at(i)[0] =  int(std::round(obj.at(i)[0] / obj.at(i)[3])) - _umin;
      obj.at(i)[1] = -int(std::round(obj.at(i)[1] / obj.at(i)[3])) - _vmin;
      obj.at(i)[2] /= obj.at(i)[3];
      obj.at(i)[3] /= obj.at(i)[3];
    }

    for(auto oi = obj.begin(); oi != obj.end(); oi++) {
      auto s = oi->end() - 1;
      for(auto e = oi->begin(); e != oi->end(); e++) {
        cv::line(dst,
            cv::Point(obj.at(*s)[0], obj.at(*s)[1]),
            cv::Point(obj.at(*e)[0], obj.at(*e)[1]),
            cv::Scalar(255, 0, 0));;
        s = e;
      }
    }
  }
}

/**
 * TODO
 *
 * @param m
 * @param dst
 * @param zbuffer
 */
void ray::camera::draw_proj(model* m, cv::Mat& dst, cv::Mat& zbuffer) {
  int x_limits[2], j, idx_s, idx_e, idx_f;
  double rl[2], gl[2], bl[2], zl[2], za, zb;
  unsigned int i;
  double ymin, ymax;

  ray::vector a, b, cca, ccb;
  std::vector<std::vector<double> > z_values;
  ray::matrix<4, 4> proj = projection();
  ray::matrix<4, 4> rota = rotation();

  for(auto iter = zbuffer.begin<double>(); iter != zbuffer.end<double>(); iter++)
    *iter = std::numeric_limits<double>::max();

  for(auto iter = m->begin(); iter != m->end(); iter++) {
    ray::object obj = *(iter->second);
    obj *= rota;
    z_values = point_z(obj);
    obj *= proj;

    for(i = 0; i < obj.size(); i++) {
      obj.at(i)[0] =  int(std::round(obj.at(i)[0] / obj.at(i)[3])) - _umin;
      obj.at(i)[1] = -int(std::round(obj.at(i)[1] / obj.at(i)[3])) - _vmin;
      obj.at(i)[2] /= obj.at(i)[3];
      obj.at(i)[3] /= obj.at(i)[3];
    }

    idx_f = 0;
    for(auto oi = obj.begin(); oi != obj.end(); oi++, idx_f++) {
      std::vector<ray::vector> color = point_color(m, *oi);

      ymin = std::numeric_limits<double>::max();
      ymax = std::numeric_limits<double>::min();

      for(auto pi = oi->begin(); pi != oi->end(); pi++) {
        if(obj.at(*pi)[1] < ymin) ymin = std::ceil (obj.at(*pi)[1]);
        if(obj.at(*pi)[1] > ymax) ymax = std::floor(obj.at(*pi)[1]);
      }

      ymin = std::min(std::max(ymin, 0.0), double(dst.rows));
      ymax = std::min(std::max(ymax, 0.0), double(dst.rows));

      for(int curr_y = ymin; curr_y < ymax + 1; curr_y++) {
        x_limits[0] = -1;
        x_limits[1] = -1;

        /* calculate intersections */
        auto s = oi->end() - 1; i = 0;
        idx_s = color.size() - 1; idx_e = 0;
        for(auto e = oi->begin(); e != oi->end(); e++, idx_e++) {
          a = obj.at(*e);
          b = obj.at(*s);
          cca = color[idx_e];
          ccb = color[idx_s];
          za = z_values[idx_f][idx_e];
          zb = z_values[idx_f][idx_s];

          if((a[1] < curr_y && b[1] >= curr_y) ||
             (b[1] < curr_y && a[1] >= curr_y)) {
            x_limits[i] = int(a[0] + (curr_y - a[1])/(b[1] - a[1]) * (b[0] - a[0]));
            rl[i] =  (cca[0] + (curr_y - a[1])/(b[1] - a[1]) * (ccb[0] - cca[0]));
            gl[i] =  (cca[1] + (curr_y - a[1])/(b[1] - a[1]) * (ccb[1] - cca[1]));
            bl[i] =  (cca[2] + (curr_y - a[1])/(b[1] - a[1]) * (ccb[2] - cca[2]));
            zl[i] =  (za     + (curr_y - a[1])/(b[1] - a[1]) * (zb     - za));

            i++;
          }

          s = e;
          idx_s = idx_e;
        }

        /* make sure we are in the right order */
        if(x_limits[0] > x_limits[1]) {
          std::swap(x_limits[0], x_limits[1]);
          std::swap(rl[0], rl[1]);
          std::swap(gl[0], gl[1]);
          std::swap(bl[0], bl[1]);
          std::swap(zl[0], zl[1]);
        }

        double r = rl[0], dr = (rl[1] - rl[0])/(x_limits[1] - x_limits[0]);
        double g = gl[0], dg = (gl[1] - gl[0])/(x_limits[1] - x_limits[0]);
        double b = bl[0], db = (bl[1] - bl[0])/(x_limits[1] - x_limits[0]);
        double z = zl[0], dz = (zl[1] - zl[0])/(x_limits[1] - x_limits[0]);

        if(x_limits[0] >= dst.cols || x_limits[1] < 0)
          continue;
        if(x_limits[0] < 0 ) {
          z += (0 - x_limits[0]) * dz;
          r += (0 - x_limits[0]) * dr;
          g += (0 - x_limits[0]) * dg;
          b += (0 - x_limits[0]) * db;
          x_limits[0] = 0;
        }
        if(x_limits[1] > dst.cols)
          x_limits[1] = dst.cols;

        for(j = x_limits[0]; j < x_limits[1]; j++) {
          if(zbuffer.at<double>(curr_y, j) > z) {
            dst.at<cv::Vec<unsigned char, 3> >(curr_y, j) =
                cv::Vec<unsigned char, 3> (
                    std::round(b), std::round(g), std::round(r));
            zbuffer.at<double>(curr_y, j) = z;
          }
          r += dr;
          g += dg;
          b += db;
          z += dz;
        }
      }
    }
  }
}


std::vector<ray::vector> ray::camera::point_color(model* m, polygon& p) {
  ray::vector Lp, Rl;
  ray::vector curr, n, v;
  ray::material mat = m->mat(p.material());
  std::vector<ray::vector> ret;

  for(polygon::iterator iter = p.begin(); iter != p.end(); iter++) {
    ray::vector color;
    curr = p.owner()[*iter];
    n = p.owner().norm(*iter);
    v = _fp - curr;
    v.normalize();
    n.normalize();

    for(model::l_iterator li = m->l_begin(); li != m->l_end(); li++) {
      Lp = li->direction(curr);
      Lp.normalize();

      if(Lp.dot(n) < 0)
        continue;

      Rl = n * (Lp.dot(n) * 2) - Lp;
      Rl.normalize();

      color += (mat.diffuse() * li->illumination() * Lp.dot(n)) +
          (li->illumination() * mat.ks() *
              std::pow(std::max(0.0, v.dot(Rl)), mat.alpha()));
    }

    color[0] = std::min(color[0], 255.0);
    color[1] = std::min(color[1], 255.0);
    color[2] = std::min(color[2], 255.0);

    ret.push_back(color);
  }

  return ret;
}

std::vector<std::vector<double> > ray::camera::point_z(const ray::object& obj) {
  std::vector<std::vector<double> > ret;

  for(auto oi = obj.begin(); oi != obj.end(); oi++) {
    std::vector<double> curr;

    for(auto pi = oi->begin(); pi != oi->end(); pi++) {
      curr.push_back(-obj.at(*pi)[2]);
    }

    ret.push_back(curr);
  }

  return ret;
}

/**
 * TODO
 *
 * @param m
 * @return
 */
void ray::camera::click(model* m, cv::Mat& dst) {
  /* locals */
  std::vector<std::thread*> threads;
  ray::vector U, L;

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
  for(int x = umin(); x <= umax(); x++) {
    for(int y = vmin(); y <= vmax(); y++) {
      L = vrp() + u()*x + v()*y;
      U = L - fp(); U.normalize();
#ifdef DEBUG
      if(x == X_PRINT && y == Y_PRINT)
        print = true;
      ray::l_ray(m, L, U, dst.at<cv::Vec<uc, 3> >(vmax() - y, x - umin()))();
      if(x == X_PRINT && y == Y_PRINT)
        print = false;
    }
  }

  cv::Vec<uc, 3> fill(255, 255, 255);
  int x = X_PRINT - umin();
  int y = Y_PRINT - vmin();
  dst.at<cv::Vec<uc, 3> >(x - 1, y - 1) = fill;
  dst.at<cv::Vec<uc, 3> >(x - 1, y + 1) = fill;
  dst.at<cv::Vec<uc, 3> >(x - 1, y    ) = fill;
  dst.at<cv::Vec<uc, 3> >(x    , y - 1) = fill;
  dst.at<cv::Vec<uc, 3> >(x    , y + 1) = fill;
  dst.at<cv::Vec<uc, 3> >(x + 1, y - 1) = fill;
  dst.at<cv::Vec<uc, 3> >(x + 1, y + 1) = fill;
  dst.at<cv::Vec<uc, 3> >(x + 1, y    ) = fill;
#else
      ray::l_ray(m, L, U, dst.at<cv::Vec<uc, 3> >(vmax() - y, x - umin()))();
    }
    std::cout << "row: " << x << std::endl;
  }
      /*rays.push(new ray::l_ray(m, L, U,
          dst.at<cv::Vec<uc, 3> >(vmax() - y, x - umin())));
    }
  }

  numb_on = 1;
  block_on = 1;

  int n_thread = std::max(int(std::thread::hardware_concurrency()- 1), 1);
  for(int i = 0; i < 2; i++) {
    running++;
    threads.push_back(new std::thread(wrapper));
  }

  while(running) {
    cv::imshow("win", dst);
    if(numb_on == std::thread::hardware_concurrency()) {
      cv::waitKey(0);
      block_on++;
      wait_on.notify_all();
      while(numb_on != 1)
        usleep(100);
    }

    cv::waitKey(30);
  }

  for(int i = 0; i < 2; i++) {
    threads[i]->join();
  }*/
#endif

  /* create the output image */
  cv::imwrite("output.png", dst);
  cv::imshow("win", dst);
  cv::waitKey(-1);
}

ray::matrix<4, 4> ray::camera::projection() const {
  ray::matrix<4, 4> proj = ray::identity<4>();

  proj[3][2] = 1.0/_fl;
  proj[3][3] = 0;

  return proj;
}

ray::matrix<4, 4> ray::camera::rotation() const {
  ray::matrix<4, 4> rota = ray::identity<4>();
  ray::matrix<4, 4> tran = ray::identity<4>();

  rota[0][0] = _u[0]; rota[0][1] = _u[1]; rota[0][2] = _u[2];
  rota[1][0] = _v[0]; rota[1][1] = _v[1]; rota[1][2] = _v[2];
  rota[2][0] = _n[0]; rota[2][1] = _n[1]; rota[2][2] = _n[2];

  tran[0][3] = -_fp[0];
  tran[1][3] = -_fp[1];
  tran[2][3] = -_fp[2];

  return rota * tran;
}

ray::display::display(ray::model* m, ray::camera* cam, bool wire) :
    _m(m), _cam(cam), _rot(), _wire(wire), last_x(-1), last_y(-1), _state(none),
    image  (cam->height(), cam->width(), CV_8UC3),
    zbuffer(cam->height(), cam->width(), CV_64FC1) {
  cv::namedWindow(ZBUF, CV_WINDOW_AUTOSIZE);
  cv::namedWindow(WIND, CV_WINDOW_AUTOSIZE);
  cvSetMouseCallback(WIND, ray::display::mouse, this);
  cvSetMouseCallback(ZBUF, ray::display::mouse, this);
  gettimeofday(&last_t, NULL);
  dir = cam->fl() < 0 ? -1 : 1;
}

void ray::display::show() {
  double min, max;

  for(auto iter = image.begin<cv::Vec<uc, 3> >();
      iter != image.end<cv::Vec<uc, 3> >(); iter++) {
    *iter = cv::Vec<uc, 3>();
  }

  if(_wire) _cam->draw_wire(_m, image);
  else      _cam->draw_proj(_m, image, zbuffer);

  min = *std::min_element(zbuffer.begin<double>(), zbuffer.end<double>());
  std::transform(zbuffer.begin<double>(), zbuffer.end<double>(),
      zbuffer.begin<double>(), [&min](double val)
      { return val == std::numeric_limits<double>::max() ?  val : val - min; });
  std::transform(zbuffer.begin<double>(), zbuffer.end<double>(),
      zbuffer.begin<double>(), [](double val)
      { return val == std::numeric_limits<double>::max() ? 0 : val; });
  max = *std::max_element(zbuffer.begin<double>(), zbuffer.end<double>());
  std::transform(zbuffer.begin<double>(), zbuffer.end<double>(),
      zbuffer.begin<double>(), [&max](double val)
      { return val / max; });

  cv::imshow(ZBUF, zbuffer);
  cv::imshow(WIND, image);
}

void ray::display::exec() {
  show();
  cv::waitKey(-1);
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

  /* this function can get overloaded      */
  /* so we calculate  time since last call */
  gettimeofday(&curr, NULL);
  seconds  = curr.tv_sec  - last_t.tv_sec;
  useconds = curr.tv_usec - last_t.tv_usec;
  if((seconds * 1000 + useconds/1000.0) < 50) {
    return;
  }

  switch(_state) {
    case none: /* do nothing */ break;
    case left:
      _cam->rotate((x - last_x) * 0.0174 * dir, _rot, ray::camera::x_axis);
      _cam->rotate((y - last_y) * 0.0174 * dir, _rot, ray::camera::y_axis);
      break;
    case right:
      _cam->translate( (x - last_x) * 10 * dir, ray::camera::x_axis);
      _cam->translate(-(y - last_y) * 10 * dir, ray::camera::y_axis);
      break;
    case middle:
      _cam->translate((y - last_y) * 10, ray::camera::z_axis);
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
