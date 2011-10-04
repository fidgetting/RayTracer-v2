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

bool ray::l_ray::operator()() {
#ifdef BLOCKING
  if(_depth == block_on) {
    std::unique_lock<std::mutex> lock(lock_on);
    numb_on++;
    wait_on.wait(lock);
    numb_on--;
  }
#endif
  _pixel = _pixel + _generator->color(this);
  _pixel[0] = std::min(int(_pixel[0]), 255);
  _pixel[1] = std::min(int(_pixel[1]), 255);
  _pixel[2] = std::min(int(_pixel[2]), 255);

  return !(_cont < 0.0039 || _depth > MAX_DEPTH ||
      (_pixel[0] == 255 && _pixel[1] == 255 && _pixel[2] == 255));
}

cv::Vec<uc, 3> operator+(const cv::Vec<uc, 3>& rhs, const ray::vector& lhs) {
  cv::Vec<uc, 3> ret;

  ret[0] = rhs[0] + lhs[0];
  ret[1] = rhs[1] + lhs[1];
  ret[2] = rhs[2] + lhs[2];

  return ret;
}
