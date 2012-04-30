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
#include <sys/time.h>
#include <unistd.h>

#include <cv.h>
#include <cvaux.h>
#include <cxcore.h>
#include <highgui.h>

#define DEBUG 1

namespace ray {

  class camera {
    public:
      camera(ray::model& mod);
      ~camera() { };

      enum axis {
        x_axis = 0,
        y_axis = 1,
        z_axis = 2
      };

      const static int height;
      const static int width;

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
      inline double&         umin()       { return _umin; }
      inline double          umin() const { return _umin; }
      inline double&         umax()       { return _umax; }
      inline double          umax() const { return _umax; }
      inline double&         vmin()       { return _vmin; }
      inline double          vmin() const { return _vmin; }
      inline double&         vmax()       { return _vmax; }
      inline double          vmax() const { return _vmax; }

      void translate(double amount, axis which);
      void rotate(double amount, ray::vector around, axis which);
      void draw_proj(model& m, cv::Mat& dst, cv::Mat& zbuffer);

      std::vector<double> point_z(const object& obj);

      cv::Mat click(model& m);
      matrix<4, 4> projection() const;
      matrix<4, 4> rotation() const;

      static int x_print;
      static int y_print;

      static bool animation;

#ifdef DEBUG
      static bool print;
#else
      static int running;
#endif

    private:

      ray::vector _fp, _vrp;
      ray::vector _n, _u, _v;
      double _fl;
      double _umin, _umax;
      double _vmin, _vmax;
  };

  class display {
    public:

      enum event_t {
        move = 0,
        l_down = 1,
        r_down = 2,
        m_down = 3,
        l_up = 4,
        r_up = 5,
        m_up = 6
      };

      enum state_t {
        none = 0,
        left = 1,
        right = 2,
        middle = 3,
        ray = 4
      };

      display(ray::model& m, ray::camera& cam, bool wire = true);

      void show();
      void exec();

      static void mouse(int event, int x, int y, int flags, void* disp);

      void move_e(int x, int y);
      void left_e(bool down);
      void right_e(bool down);
      void middle_e(bool down);

    protected:

      ray::model*    _m;
      ray::camera*   _cam;
      ray::vector    _rot;
      bool           _wire;
      int            last_x;
      int            last_y;
      int            dir;
      struct timeval last_t;
      state_t        _state;

      cv::Mat image;
      cv::Mat zbuffer;
  };
}

#endif /* CAMERA_H_INCLUDE */
