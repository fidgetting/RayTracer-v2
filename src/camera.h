/*
 * Camera.h
 *
 *  Created on: Sep 21, 2010
 *      Author: norton
 */

#ifndef CAMERA_H_INCLUDE
#define CAMERA_H_INCLUDE

#include <model.h>
#include <ray.h>

#include <utility>

namespace ray {

  class camera {
    public:
      camera() : _fp(), _n(), _u(), _v() { };
      ~camera() { };

      inline ray::vector&    fp()         { return _fp;   }
      inline ray::vector     fp()   const { return _fp;   }
      inline ray::vector&    vrp()        { return _vrp;  }
      inline ray::vector     vrp()  const { return _vrp;  }
      inline ray::vector&    n()          { return _n;    }
      inline ray::vector     n()    const { return _n;    }
      inline ray::vector&    u()          { return _u;    }
      inline ray::vector     u()    const { return _u;    }
      inline ray::vector&    v()          { return _v;    }
      inline ray::vector     v()    const { return _v;    }
      inline double&         fl()         { return _fl;   }
      inline double          fl()   const { return _fl;   }
      inline int&            umin()       { return _umin; }
      inline int             umin() const { return _umin; }
      inline int&            umax()       { return _umax; }
      inline int             umax() const { return _umax; }
      inline int&            vmin()       { return _vmin; }
      inline int             vmin() const { return _vmin; }
      inline int&            vmax()       { return _vmax; }
      inline int             vmax() const { return _vmax; }

      void click(model* m);
      ray::vector color(l_ray* r) const;

#ifdef DEBUG
      static bool print;
#else
      static int running;
#endif

    private:

      ray::vector reflectance(l_ray* r, vector p, vector n,
                                 const material& mat, const surface* s) const;

      bool shadowed(const vector& pt, const vector& dir, const model* m,
                    const surface* s) const;

      ray::vector _fp, _vrp;
      ray::vector _n, _u, _v;
      double _fl;
      int _umin, _umax;
      int _vmin, _vmax;
  };

}

#endif /* CAMERA_H_INCLUDE */
