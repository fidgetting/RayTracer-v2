/*
 * Transform.h
 *
 *  Created on: Aug 30, 2010
 *      Author: norton
 */

#ifndef TRANSFORM_H_INCLUDE
#define TRANSFORM_H_INCLUDE

#include <matrix.tpp>
#include <vector.h>

namespace ray {

  class scale {
    public:
      scale() : _x(0), _y(0), _z(0) { }
      virtual ~scale() { }

      matrix<4, 4> operator()();

      inline double  x() const { return _x; }
      inline double& x()       { return _x; }
      inline double  y() const { return _y; }
      inline double& y()       { return _y; }
      inline double  z() const { return _z; }
      inline double& z()       { return _z; }

    protected:

      double _x;
      double _y;
      double _z;
  };

  class translate {
    public:
      translate() : _x(0), _y(0), _z(0) { }
      virtual ~translate() { }

      matrix<4, 4> operator()();

      inline double  x() const { return _x; }
      inline double& x()       { return _x; }
      inline double  y() const { return _y; }
      inline double& y()       { return _y; }
      inline double  z() const { return _z; }
      inline double& z()       { return _z; }

    protected:

      double _x;
      double _y;
      double _z;
  };

  class rotate {
    public:
      rotate() : _r(0), _w() { }
      virtual ~rotate() { }

      matrix<4, 4> operator()();

      inline double  r() const { return _r;    }
      inline double& r()       { return _r;    }
      inline double  x() const { return _w[0]; }
      inline double& x()       { return _w[0]; }
      inline double  y() const { return _w[1]; }
      inline double& y()       { return _w[1]; }
      inline double  z() const { return _w[2]; }
      inline double& z()       { return _w[2]; }

    protected:

      double      _r;
      ray::vector _w;
  };

}

#endif /* TRANSFORM_H_INCLUDE  */
