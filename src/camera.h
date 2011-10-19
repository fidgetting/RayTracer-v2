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

#include <objstream.hpp>

#include <utility>

#include <cv.h>
#include <cvaux.h>
#include <cxcore.h>
#include <highgui.h>

namespace ray {

  class camera {
    public:
      camera(const obj::objstream::camera& src);
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

      void rotate(double amount, ray::vector around);
      void draw_wire(model* m, cv::Mat& dst);

      void click(model* m, cv::Mat& dst);
      matrix<4, 4> projection() const;

#ifdef DEBUG
      static bool print;
#else
      static int running;
#endif

    private:

      ray::vector _fp, _vrp;
      ray::vector _n, _u, _v;
      double _fl;
      int _umin, _umax;
      int _vmin, _vmax;
  };

}

#endif /* CAMERA_H_INCLUDE */
