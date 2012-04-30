/*
 * model.h
 *
 *  Created on: Sep 12, 2011
 *      Author: norton
 */

#ifndef MODEL_H_INCLUDE
#define MODEL_H_INCLUDE

#include <matrix.tpp>
#include <objstream.hpp>
#include <vector.h>

#include <iostream>
#include <vector>

namespace ray {

  class surface;
  class s_tree;
  class triangle;
  class sphere;

  class material {
    public:

      material() { }
      material(const obj::objstream::material& mat);

      inline std::string    name()    const { return _name;    }
      inline std::string&   name()          { return _name;    }
      inline double         ks()      const { return _ks;      }
      inline double&        ks()            { return _ks;      }
      inline double         alpha()   const { return _alpha;   }
      inline double&        alpha()         { return _alpha;   }
      inline double         kt()      const { return _kt;      }
      inline double&        kt()            { return _kt;      }
      inline matrix<4, 4>   diffuse() const { return _diffuse; }
      inline matrix<4, 4>&  diffuse()       { return _diffuse; }

    protected:

      std::string  _name;
      double       _ks, _alpha;
      double       _kt;
      matrix<4, 4> _diffuse;
  };

  class light {
    public:

      light() : _illumination(), _position() { }

      inline ray::vector  illumination() const { return _illumination; }
      inline ray::vector& illumination()       { return _illumination; }
      inline ray::vector  position()     const { return _position;     }
      inline ray::vector& position()           { return _position;     }

      ray::vector direction(ray::vector src) const;

    protected:

      ray::vector _illumination;
      ray::vector _position;
  };

  class object {
    public:

      object();
      object(const object& obj);
      virtual ~object();

      const object& operator=(const object& obj);

      void push_v(const obj::objstream::vertex& v);
      void push_n(const obj::objstream::vertex& n);

      inline uint32_t v_size() const { return _size;        }
      inline uint32_t n_size() const { return _norm.size(); }

      inline const vector operator[](int i) const { return &_data[i * V_SIZE]; }
      inline const double*        at(int i) const { return &_data[i * V_SIZE]; }
      inline       double*        at(int i)       { return &_data[i * V_SIZE]; }

      inline       ray::s_tree* root()       { return _root; }
      inline const ray::s_tree* root() const { return _root; }

      inline vector  norm(int idx) const { return _norm[idx]; }
      inline vector& norm(int idx)       { return _norm[idx]; }

      void operator*=(const ray::matrix<4, 4>& mat);

    protected:
      double*                  _data;
      uint32_t                 _size;
      uint32_t                 _capa;
      ray::s_tree*             _root;
      std::vector<ray::vector> _norm;
  };

  class model {
    public:

      typedef std::map<std::string, object*> obj_map_t;
      typedef obj_map_t::              iterator               iterator;
      typedef obj_map_t::        const_iterator         const_iterator;
      typedef obj_map_t::      reverse_iterator       reverse_iterator;
      typedef obj_map_t::const_reverse_iterator const_reverse_iterator;

      typedef std::vector<light>::      iterator       l_iterator;
      typedef std::vector<light>::const_iterator l_const_iterator;

      const static std::string default_mat_name;

      model();
      virtual ~model();

      void build(const obj::objstream& src);

      inline       l_iterator l_begin()       { return _lights.begin(); }
      inline       l_iterator l_end()         { return _lights.end();   }
      inline l_const_iterator l_begin() const { return _lights.begin(); }
      inline l_const_iterator l_end()   const { return _lights.end();   }

      inline               iterator begin()        { return _objects.begin();  }
      inline               iterator end()          { return _objects.end();    }
      inline         const_iterator begin()  const { return _objects.begin();  }
      inline         const_iterator end()    const { return _objects.end();    }
      inline       reverse_iterator rbegin()       { return _objects.rbegin(); }
      inline       reverse_iterator rend()         { return _objects.rend();   }
      inline const_reverse_iterator rbegin() const { return _objects.rbegin(); }
      inline const_reverse_iterator rend()   const { return _objects.rend();   }

      material& mat(const std::string& name);

      ray::vector center() const;
      double dimension(int dim) const;

      inline double  width() const { return dimension(X); }
      inline double height() const { return dimension(Y); }
      inline double  depth() const { return dimension(Z); }

      inline void push_light(light l) { _lights.push_back(l); }

      ray::vector reflectance(vector p, vector v, vector n,
          const surface* s, bool shadows = true);
      bool shadowed(const ray::vector& pt, const ray::vector& U,
          const surface* s) const;

      static bool smooth_shading;
      static int  vertex_spheres;

    protected:
      std::vector<light>              _lights;
      std::vector<int>                _illumination;
      std::map<std::string, object*>  _objects;
      std::map<std::string, material> _materials;
  };
}

#endif /* MODEL_H_ */
