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

#define X_PRINT 0
#define Y_PRINT 0

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

ray::camera::camera(const obj::objstream::camera& src) :
    _fp(src.fp()), _n(src.vpn()), _u(src.vup().cross(src.vpn())), _fl(src.d()) {
  _v = _n.cross(_u);

  _n.normalize();
  _u.normalize();
  _v.normalize();

  _vrp = _fp + (_n * (-_fl));
}

void ray::camera::draw_wire(model* m, cv::Mat& dst) {
  int x1, y1;
  int x2, y2;

  ray::matrix<4, 4> proj = projection();

  for(auto iter = m->begin(); iter != m->end(); iter++) {
    ray::object obj = *(iter->second);

    obj *= proj;

    for(unsigned int i = 0; i < obj.size(); i++) {
      ray::vector curr = obj[i];
      for(int j = 0; j < V_SIZE; j++) {
        curr[j] /= curr[V_SIZE - 1];
      }
    }

    for(auto oi = obj.begin(); oi != obj.end(); oi++) {
      for(auto pi = oi->begin(); pi != oi->end() - 1; pi++) {
        x1 = -int(obj[*pi][0]) - _umin;
        y1 = -int(obj[*pi][1]) - _vmin;
        x2 = -int(obj[*(pi + 1)][0]) - _umin;
        y2 = -int(obj[*(pi + 1)][1]) - _vmin;

        cv::line(dst, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(255, 255, 255));;
      }

      auto s = oi->end() - 1;
      auto e = oi->begin();

      x1 = -int(obj[*s][0]) - _umin;
      y1 = -int(obj[*s][1]) - _vmin;
      x2 = -int(obj[*e][0]) - _umin;
      y2 = -int(obj[*e][1]) - _vmin;

      cv::line(dst, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(255, 255, 255));

    }
  }
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

  cv::Vec<uc, 3> fill(255);
  raw_image.at<cv::Vec<uc, 3> >(X_PRINT - umin() - 1, Y_PRINT - vmin() - 1) = fill;
  raw_image.at<cv::Vec<uc, 3> >(X_PRINT - umin() - 1, Y_PRINT - vmin() + 1) = fill;
  raw_image.at<cv::Vec<uc, 3> >(X_PRINT - umin() - 1, Y_PRINT - vmin()    ) = fill;
  raw_image.at<cv::Vec<uc, 3> >(X_PRINT - umin()    , Y_PRINT - vmin() - 1) = fill;
  raw_image.at<cv::Vec<uc, 3> >(X_PRINT - umin()    , Y_PRINT - vmin() + 1) = fill;
  raw_image.at<cv::Vec<uc, 3> >(X_PRINT - umin() + 1, Y_PRINT - vmin() - 1) = fill;
  raw_image.at<cv::Vec<uc, 3> >(X_PRINT - umin() + 1, Y_PRINT - vmin() + 1) = fill;
  raw_image.at<cv::Vec<uc, 3> >(X_PRINT - umin() + 1, Y_PRINT - vmin()    ) = fill;
#else
      rays.push(new ray::l_ray(m, L, U,
          dst.at<cv::Vec<uc, 3> >(vmax() - y, x - umin())));
    }
  }

  numb_on = 1;
  block_on = 1;

  int n_thread = std::max(int(std::thread::hardware_concurrency()- 1), 1);
  for(int i = 0; i < n_thread; i++) {
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

  for(int i = 0; i < n_thread; i++) {
    threads[i]->join();
  }

  std::cout << "hello?" << std::endl;
  cv::imshow("win", dst);
  cv::waitKey(-1);
#endif

  /* create the output image */
  cv::imwrite("output.png", dst);
  cv::imshow("win", dst);
  cv::waitKey(-1);
}

ray::matrix<4, 4> ray::camera::projection() const {
  ray::matrix<4, 4> proj = ray::identity<4>();
  ray::matrix<4, 4> rota = ray::identity<4>();
  ray::matrix<4, 4> tran = ray::identity<4>();

  proj[3][2] = 1.0/_fl;
  proj[3][3] = 0;

  rota[0][0] = _u[0]; rota[0][1] = _u[1]; rota[0][2] = _u[2];
  rota[1][0] = _v[0]; rota[1][1] = _v[1]; rota[2][2] = _v[2];
  rota[2][0] = _n[0]; rota[2][1] = _n[1]; rota[2][2] = _n[2];

  tran[0][3] = -_fp[0];
  tran[1][3] = -_fp[1];
  tran[2][3] = -_fp[2];

  return proj * rota * tran;
}
