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

/* std library includes */
#include <string>
#include <tuple>
#include <vector>

namespace ray {

  class surface {
    public:
      surface() : _id(id_gen++) { };
      virtual ~surface() { };

      virtual vector normal(const vector& v) const = 0;
      virtual vector center()                const = 0;
      virtual double radius()                const = 0;

      virtual std::tuple<vector, double, const surface*>
        intersection(const vector& U, const vector& L, const surface* skip)
        const = 0;

      inline std::string& material()       { return _material; }
      inline std::string  material() const { return _material; }
      inline int          id()       const { return _id;       }
      inline bool&        src()            { return _src;      }
      inline bool         src()      const { return _src;      }

    protected:

      std::string _material;
      int         _id;
      bool        _src;

      static int id_gen;
  };

  /*class sphere : public surface {
    public:

      typedef std::vector<surface*>::iterator iterator;
      typedef std::vector<surface*>::const_iterator const_iterator;

      sphere() { }
      sphere(const point& _center, double _radius) : _center(_center), _radius(_radius) { }
      virtual ~sphere() { for(auto iter = _subsurfaces.begin(); iter != _subsurfaces.end(); iter++) delete *iter; }

      inline point& center() { return _center; }
      inline double& radius() { return _radius; }
      inline unsigned int size() { return _subsurfaces.size(); }
      inline void clear() { _subsurfaces.clear(); }
      inline void push_back(surface* s) { _subsurfaces.push_back(s); }
      inline void insert(const iterator& pos, surface* s) { _subsurfaces.insert(pos, s); }
      inline void insert(const iterator& s, const iterator& e) { _subsurfaces.insert(end(), s, e); }
      bool close(const sphere& oth) const;

      inline       iterator begin()       { return _subsurfaces.begin(); }
      inline       iterator end()         { return _subsurfaces.end();   }
      inline const_iterator begin() const { return _subsurfaces.begin(); }
      inline const_iterator end()   const { return _subsurfaces.end();   }

      virtual inline vector<3> normal(const vector<3>& v) const { return v - _center; }
      virtual inline point     center()                   const { return _center; }
      virtual inline double    radius()                   const { return _radius; }

      virtual std::tuple<point, double, const surface*>
      intersection(const vector<3>& ray, const point& src, const surface* skip) const;

    protected:

      point                 _center;       ///< the index of the center of the sphere within a shape
      double                _radius;       ///< the radius of the sphere
      std::vector<surface*> _subsurfaces;  ///< list of surfaces that are contained within this sphere
  };*/

  class polygon : public surface {
    public:

      typedef std::vector<int>::      iterator       iterator;
      typedef std::vector<int>::const_iterator const_iterator;

      polygon(ray::object* owner) : _indeces(), _n(), _owner(owner) { }
      virtual ~polygon() { }

      inline void add_vertex(int idx) { _indeces.push_back(idx); }
      void set_normal();

      inline int           size()             const { return _indeces.size();  }
      inline vector&     normal()                   { return _n;               }
      inline vector  operator[](int i)        const { return (*_owner)[i];     }
      inline vector  get(iterator iter)       const { return (*_owner)[*iter]; }
      inline vector  get(const_iterator iter) const { return (*_owner)[*iter]; }

      inline       iterator begin()       { return _indeces.begin(); }
      inline       iterator end()         { return _indeces.end();   }
      inline const_iterator begin() const { return _indeces.begin(); }
      inline const_iterator end()   const { return _indeces.end();   }

      void operator*=(const ray::matrix<4, 4>& transform);

      virtual vector normal(const vector& v) const;
      virtual vector center()                const;
      virtual double radius()                const;

      inline ray::object& owner() { return *_owner; }

      virtual std::tuple<vector, double, const surface*>
        intersection(const vector& U, const vector& L, const surface* skip)
        const;

    protected:

      std::vector<int> _indeces;
      ray::vector      _n;
      ray::object*     _owner;
  };
}

/*inline bool operator==(const ray::sphere& lhs, const ray::sphere& rhs) {
  return lhs.center() == rhs.center() && lhs.radius() == rhs.radius();
}*/

#endif /* SURFACE_H_INCLUDE */
