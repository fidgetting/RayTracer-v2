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

  class polygon;

  class material {
    public:

      material() { }
      ~material() { }

      inline std::string    name()    const { return _name;    }
      inline std::string&   name()          { return _name;    }
      inline double         ks()      const { return _ks;      }
      inline double&        ks()            { return _ks;      }
      inline double         alpha()   const { return _alpha;   }
      inline double&        alpha()         { return _alpha;   }
      inline double         kt()      const { return _kt;      }
      inline double&        kt()            { return _kt;      }
      inline double         density() const { return _density; }
      inline double&        density()       { return _density; }
      inline matrix<4, 4>   diffuse() const { return _diffuse; }
      inline matrix<4, 4>&  diffuse()       { return _diffuse; }

    protected:

      std::string  _name;
      double       _ks, _alpha;
      double       _kt, _density;
      matrix<4, 4> _diffuse;
  };

  class light {
    public:

      light() : _illumination(), _position() { }
      ~light() { }

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

      typedef std::vector<polygon> poly_t;
      typedef poly_t::      iterator       iterator;
      typedef poly_t::const_iterator const_iterator;

      object();
      object(const object& obj);
      virtual ~object();

      const object& operator=(const object& obj);

      void push_vector(const obj::objstream::vertex& v);
      void push_polygon(const ray::polygon& p);

      unsigned int index(const vector& v) const;

      inline const vector operator[](int i) const { return ray::vector(&_data[i * V_SIZE]); }
      inline       vector operator[](int i)       { return ray::vector(&_data[i * V_SIZE]); }

      inline const double*         at(int i) const { return &_data[i * V_SIZE]; }
      inline       double*         at(int i)       { return &_data[i * V_SIZE]; }

      inline unsigned int     size() const { return _size; }
      inline unsigned int capacity() const { return _capa; }

      void operator*=(const ray::matrix<4, 4>& mat);

      inline       iterator begin()       { return _surf.begin(); }
      inline       iterator end()         { return _surf.end();   }
      inline const_iterator begin() const { return _surf.begin(); }
      inline const_iterator end()   const { return _surf.end();   }

    protected:
      double*      _data;
      unsigned int _size;
      unsigned int _capa;
      poly_t       _surf;
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

      model() { }
      ~model();

      void build(const obj::objstream& src);
      void cmd(  const obj::objstream& src);

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
      const material& mat(const std::string& name) const;

    protected:
      std::vector<light>              _lights;
      std::vector<int>                _illumination;
      std::map<std::string, object*>  _objects;
      std::map<std::string, material> _materials;
  };

}

std::ostream& operator<<(std::ostream& ostr, const ray::model& m);
std::ostream& operator<<(std::ostream& ostr, const ray::object& o);

#endif /* MODEL_H_ */
