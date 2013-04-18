/*
 * Camera.hpp
 *
 *  Created on: Jan 3, 2013
 *      Author: norton
 */

#pragma once

/* local includes */
#include <Matrix.tpp>
#include <Vector.hpp>
#include <Ray.hpp>

namespace ray {

  class Model;

  class Pixel {
    public:

      Pixel() :
        data({0, 0, 0}) { }
      Pixel(Vector vec) :
        data({uint8_t(vec.x()), uint8_t(vec.y()), uint8_t(vec.z())}) { }
      Pixel(uint8_t r, uint8_t g, uint8_t b) :
        data({r, g, b}) { }

      inline uint8_t r() const { return data[0]; }
      inline uint8_t g() const { return data[1]; }
      inline uint8_t b() const { return data[2]; }

    private:

      /** data for the pixel */
      uint8_t data[3];
  };

  inline bool operator==(const Pixel& a, const Pixel& b) {
    return a.r() == b.r() && a.b() == b.b() && a.g() == b.g();
  }

  inline bool operator!=(const Pixel& a, const Pixel& b) {
    return !(a == b);
  }

  inline std::ostream& operator<<(std::ostream& ostr, const Pixel& p) {
    return (ostr << "(" << int(p.r()) << " " << int(p.g()) << " " << int(p.b()) << ")");
  }

  class Camera {
    public:

      Camera();
      Camera(Vector fp, Vector vrp, Vector up);

      ~Camera() { }

      /* move the camera around the world */
      enum axis { x_axis, y_axis, z_axis };
      void move  (double amount, axis which);
      void rotate(double amount, axis which, Vector around);

      /* get information about camera for rendering */
      Matrix<Ray>    getRays(int rows, int cols) const;
      Matrix<double>  getProjection() const;
      Matrix<double>  getRotation(int rows, int cols) const;

      Vector  _fp() const { return  fp; }
      Vector _vrp() const { return vrp; }
      Vector   _n() const { return   n; }
      Vector   _u() const { return   u; }
      Vector   _v() const { return   v; }
      double   _fl() const { return  fl; }

      /* camera creation */
      static Camera fromModel(const Model& m);

    private:

      /** Focal point of the Camera */
      Vector fp;
      /** View Reference Point of the Camera */
      Vector vrp;

      /** Normal to the plane of the Camera */
      Vector n;
      /** Horizontal for the plane of the Camera */
      Vector u;
      /** Vertical for the plane of the Camera */
      Vector v;

      /** focal length of the Camera */
      double fl;

      /** sides of the plane of the Camera */
      double umin, umax;
      /** top and bottom of the plane of the Camera */
      double vmin, vmax;
  };

}
