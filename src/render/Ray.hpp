/*
 * Ray.h
 *
 *  Created on: Dec 31, 2012
 *      Author: norton
 */

#pragma once

/* local includes */
#include <Vector.hpp>

#include <render.hpp>

/* std includes */
#include <iostream>
#include <limits>

namespace ray {

  class Surface;

  class Ray {
    public:

      Ray();
      Ray(const Vector& L, const Vector& U, const Surface* source = nullptr);

      inline const Vector&  U() const { return  _U; }
      inline const Vector&  L() const { return  _L; }
      inline const Vector& iU() const { return _iU; }
      inline const Vector& iL() const { return _iL; }

      inline bool posi(int idx) const { return _positive[idx]; }
      inline bool zero(int idx) const { return _nonzero[idx];  }

      inline const Surface* source() const { return _source; }

      operator render::d_Ray() const;

    private:

      /** component Vectors */
      Vector _L, _U, _iU, _iL;

      /** which elements of the direction Vector are positive */
      bool _positive[3];

      /** which elements of the direction Vector are non-zero */
      bool _nonzero[3];

      /** the surface that the Ray bounced off of */
      const Surface* _source;
  };

  class Intersection {
    public:

      Intersection() :
        _source  (nullptr),
        _location(),
        _normal  (),
        _viewing (),
        _distance(std::numeric_limits<double>::max()) { }

      Intersection(
          const Surface* source  ,
          Vector         location,
          Vector         normal  ,
          Vector         viewing ,
          double         distance) :
        _source  (source  ),
        _location(location),
        _normal  (normal  ),
        _viewing (viewing ),
        _distance(distance) { }

      inline const Surface*   source() const { return _source;   }
      inline       Vector          i() const { return _location; }
      inline       Vector          n() const { return _normal;   }
      inline       Vector          v() const { return _viewing;  }
      inline       double   distance() const { return _distance; }

      static Intersection best(const Intersection& l, const Intersection& r);

    private:

      /** Surface that this Intersection occurred on */
      const Surface* _source;

      /** The location of the Intersection in world coordinates */
      Vector   _location;

      /** The normal to the Surface at the location of Intersection */
      Vector   _normal;

      /** the viewing Vector for the intersection */
      Vector   _viewing;

      /** The distance from the Ray's source to the location of Intersection */
      double   _distance;
  };

  std::ostream& operator<<(std::ostream& ostr, const Ray& ray);
  std::ostream& operator<<(std::ostream& ostr, const Intersection& inter);

}

