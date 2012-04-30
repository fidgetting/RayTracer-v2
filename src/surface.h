/*
 * Surface.h
 *
 *  Created on: Oct 25, 2010
 *      Author: norton
 */

#ifndef SURFACE_H_INCLUDE
#define SURFACE_H_INCLUDE

/* local includes */
#include <model.h>
#include <vector.h>
#include <matrix.tpp>
#include <ray.h>

/* std library includes */
#include <stdint.h>
#include <string>
#include <vector>

namespace ray {

  struct ray_info {

      ray_info(const ray::vector& L, const ray::vector& U);

      ray::vector _U, _L, _iL, _iU;
      bool positive[3];
      bool nonzero[3];
      bool print;

      inline const ray::vector&  U() const { return  _U; }
      inline const ray::vector&  L() const { return  _L; }
      inline const ray::vector& iL() const { return _iL; }
  };

  class bbox {
    public:

      bbox() : _min(0.0), _max(0.0), _len(-1.0) { }
      virtual ~bbox() { }

      inline ray::vector  min() const { return _min; }
      inline ray::vector& min()       { return _min; }
      inline ray::vector  max() const { return _max; }
      inline ray::vector& max()       { return _max; }
      inline ray::vector  len() const { return _len; }
      inline ray::vector& len()       { return _len; }

      void expand_by(const ray::bbox& box);

      bool contains(const ray::bbox& box) const;
      bool intersection(const ray_info& ray) const;

      ray::vector center() const;

    protected:

      ray::vector _min;
      ray::vector _max;
      ray::vector _len;
  };

  class surface {
    public:
      surface() : _id(id_gen++) { };
      virtual ~surface() { };

      virtual const ray::bbox& compute_bbox() = 0;
      virtual const ray::surface* intersection(const ray::ray_info& ray,
          const surface* skip, vector& I, double& r, vector& n) const = 0;
      virtual void fill(model* m, const camera* cam, ray::object& obj,
          cv::Mat& dst, cv::Mat& z_buf, const std::vector<double> z_vals) const = 0;

      inline virtual ray::vector center() const { return _bounds.center(); }

      inline std::string&     material()       { return _material; }
      inline std::string      material() const { return _material; }
      inline int              id()       const { return _id;       }
      inline const ray::bbox& bounds()   const { return _bounds;   }

    protected:

      std::string _material;
      ray::bbox   _bounds;
      int         _id;

      static int id_gen;
  };

  class s_tree : public surface {
    public:

      s_tree() { }
      s_tree(const s_tree& cpy);
      virtual ~s_tree();

      virtual const ray::bbox& compute_bbox();
      virtual const surface* intersection(const ray::ray_info& ray,
          const surface* skip, vector& I, double& r, vector& n) const;
      virtual void fill(model* m, const camera* cam, ray::object& obj,
          cv::Mat& dst, cv::Mat& z_buf, const std::vector<double> z_vals) const;

      void push(ray::surface* s);

      void split();

      unsigned int size() const;

    protected:

      std::vector<surface*> _kids;
  };

  class sphere : public surface {
    public:

      typedef std::vector<surface*>::      iterator       iterator;
      typedef std::vector<surface*>::const_iterator const_iterator;

      sphere(const ray::vector& _center, double _radius);
      virtual ~sphere() { };

      /*inline ray::vector& center()       { return _center; }
      inline ray::vector  center() const { return _center; }*/
      inline double&      radius()       { return _radius; }
      inline double       radius() const { return _radius; }

      inline unsigned int size()  const { return _subsurfaces.size();  }
      inline void         clear()       {        _subsurfaces.clear(); }
      inline void push_back(surface* s) { _subsurfaces.push_back(s);   }

      inline       iterator begin()       { return _subsurfaces.begin(); }
      inline       iterator end()         { return _subsurfaces.end();   }
      inline const_iterator begin() const { return _subsurfaces.begin(); }
      inline const_iterator end()   const { return _subsurfaces.end();   }

      ray::vector normal(const ray::vector& v) const;

      virtual const ray::bbox& compute_bbox();
      virtual const surface* intersection(const ray::ray_info& ray,
          const surface* skip, vector& I, double& r, vector& n) const;
      virtual void fill(model* m, const camera* cam, ray::object& obj,
          cv::Mat& dst, cv::Mat& z_buf, const std::vector<double> z_vals) const;

    protected:

      ray::vector           _center;
      double                _radius;
      std::vector<surface*> _subsurfaces;
  };

  class triangle : public surface {
    public:

      triangle(uint32_t va, uint32_t vb, uint32_t vc,
               uint32_t na, uint32_t nb, uint32_t nc,
               ray::object& owner);

      inline ray::vector operator[](int i) const
        { return _owner[_v_idx[i]]; }
      inline uint32_t at(int i) const
        { return _v_idx[i]; }

      inline ray::vector      n() const { return      _n; }
      inline ray::vector   perp() const { return   _perp; }
      inline uint32_t    d_axis() const { return _d_axis; }
      inline uint32_t    v_axis() const { return _v_axis; }

      ray::vector normal(const ray::vector& _v) const;

      virtual const ray::bbox& compute_bbox();
      virtual const surface* intersection(const ray::ray_info& ray,
          const surface* skip, vector& I, double& r, vector& n) const;
      virtual void fill(model* m, const camera* cam, ray::object& obj,
          cv::Mat& dst, cv::Mat& z_buf, const std::vector<double> z_vals) const;

    protected:

      uint32_t     _v_idx[3];
      uint32_t     _n_idx[3];
      ray::vector  _n;
      ray::vector  _perp;
      ray::object& _owner;
      uint32_t     _d_axis;
      uint32_t     _v_axis;
  };
}

#endif /* SURFACE_H_INCLUDE */
