/*
 * ray.h
 *
 *  Created on: Sep 14, 2011
 *      Author: norton
 */

#ifndef RAY_H_INCLUDE
#define RAY_H_INCLUDE

#include <iostream>
#include <vector.h>
#include <surface.h>

#define MAX_DEPTH 700

#include <cv.h>
#include <cvaux.h>
#include <cxcore.h>
#include <highgui.h>

typedef unsigned char uc;

namespace ray {

  class camera;

  class l_ray {
    public:

      l_ray(model* _m, camera* _gen, const ray::vector& _src_p,
          const ray::vector _dir, cv::Vec<uc, 3>& _pixel) :
            _m(_m),     _generator(_gen), _src_point(_src_p), _direction(_dir),
            _pixel(_pixel), _src(NULL), _cont(1.0), _depth(0), _density(1.0) { }
      virtual ~l_ray() { }

      bool operator()();

      inline model*          world()   const { return _m;         }
      inline vector&         src()           { return _src_point; }
      inline vector          src()     const { return _src_point; }
      inline vector&         dir()           { return _direction; }
      inline vector          dir()     const { return _direction; }
      inline const surface*& surf()          { return _src;       }
      inline const surface*  surf()    const { return _src;       }
      inline double&         cont()          { return _cont;      }
      inline double          cont()    const { return _cont;      }
      inline int&            depth()         { return _depth;     }
      inline int             depth()   const { return _depth;     }
      inline double&         density()       { return _density;   }
      inline double          density() const { return _density;   }

    protected:

      model*               _m;
      camera*              _generator;
      ray::vector          _src_point;
      ray::vector          _direction;
      cv::Vec<uc, 3>&      _pixel;
      const surface*       _src;
      double               _cont;
      int                  _depth;
      double               _density;
  };
}

cv::Vec<uc, 3> operator+(const cv::Vec<uc, 3>& rhs, const ray::vector& lhs);

#endif /* RAY_H_INClUDE */
