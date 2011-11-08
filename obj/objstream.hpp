/* ****************************************************************************
 * Copyright (C) 2011 Alex Norton                                             *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify it    *
 * under the terms of the BSD 2-Clause License.                               *
 *                                                                            *
 * This program is distributed in the hope that it will be useful, but        *
 * WITHOUT ANY WARRENTY; without even the implied warranty of                 *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 **************************************************************************** */

#ifndef OBJSTREAM_HPP_INCLUDE
#define OBJSTREAM_HPP_INCLUDE

#include <matrix.tpp>
#include <vector.h>

#include <map>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

namespace obj {

  class objstream;
  extern objstream* dest;

  class objstream {
    public:

      /* ******************************************************************** */
      /* *** group definitions ********************************************** */
      /* ******************************************************************** */

      class vertex {
        public:
          vertex(double x, double y, double z, double w = 1.0) :
            _x(x), _y(y), _z(z), _w(w) { }

          inline double x() const { return _x; }
          inline double y() const { return _y; }
          inline double z() const { return _z; }
          inline double w() const { return _w; }

        private:
          double _x;
          double _y;
          double _z;
          double _w;
      };

      class texture {
        public:
          texture(double x, double y) :
            _x(x), _y(y) { }

          inline double x() const { return _x; }
          inline double y() const { return _y; }

        private:
          double _x;
          double _y;
      };

      class face {
        public:

          typedef std::vector<int>::const_iterator iterator;

          face(std::vector<int>& verticies, std::vector<int>& textures,
              std::vector<int>& normals, const std::string& mat) :
                _verts(verticies), _texts(textures),
                _norms(normals), _mat(mat) { }

          inline iterator v_begin() const { return _verts.begin(); }
          inline iterator v_end()   const { return _verts.end();   }
          inline iterator t_begin() const { return _texts.begin(); }
          inline iterator t_end()   const { return _texts.end();   }
          inline iterator n_begin() const { return _norms.begin(); }
          inline iterator n_end()   const { return _norms.end();   }

          inline unsigned int v_size() const { return _verts.size(); }
          inline unsigned int t_size() const { return _texts.size(); }
          inline unsigned int n_size() const { return _norms.size(); }

          inline const std::string& mat() const { return _mat; }

        private:
          std::vector<int> _verts;
          std::vector<int> _texts;
          std::vector<int> _norms;
          std::string _mat;
      };

      /* ******************************************************************** */
      /* *** transformations ************************************************ */
      /* ******************************************************************** */

      class transform {
        public:

          transform() { }
          virtual ~transform() { }

          virtual ray::matrix<4, 4> matrix() const = 0;
      };

      class rotate : public transform {
        public:

          rotate(double r, double x, double y, double z) :
            _r(r), _w(x, y, z) { }
          virtual ~rotate() { }

          inline double r() const { return _r;    }
          inline double x() const { return _w[0]; }
          inline double y() const { return _w[1]; }
          inline double z() const { return _w[2]; }

          virtual ray::matrix<4, 4> matrix() const;

        private:

          double      _r;
          ray::vector _w;
      };

      class translate : public transform {
        public:

          translate(double x, double y, double z) :
            _x(x), _y(y), _z(z) { }
          virtual ~translate() { }

          inline double x() const { return _x; }
          inline double y() const { return _y; }
          inline double z() const { return _z; }

          virtual ray::matrix<4, 4> matrix() const;

        private:

          double _x;
          double _y;
          double _z;
      };

      class scale : public transform {
        public:

          scale(double x, double y, double z) :
            _x(x), _y(y), _z(z) { }
          virtual ~scale() { }

          inline double x() const { return _x; }
          inline double y() const { return _y; }
          inline double z() const { return _z; }

          virtual ray::matrix<4, 4> matrix() const;

        private:

          double _x;
          double _y;
          double _z;
      };

      class arbitrary : public transform {
        public:
          arbitrary() : _mat() { }
          virtual ~arbitrary() { }

          inline double* operator[](int idx) { return _mat[idx]; }

          virtual ray::matrix<4, 4> matrix() const;

        private:
          ray::matrix<4, 4> _mat;
      };

      /* ******************************************************************** */
      /* *** bring them all together **************************************** */
      /* ******************************************************************** */

      class group {
        public:

          typedef std::vector<vertex>    ::const_iterator    vertex_iterator;
          typedef std::vector<texture>   ::const_iterator   texture_iterator;
          typedef std::vector<vertex>    ::const_iterator    normal_iterator;
          typedef std::vector<face>      ::const_iterator      face_iterator;

          typedef std::vector<transform*>::const_reverse_iterator
              transform_iterator;

          group() { }
          ~group();

          /* modifiers */
          inline void push_v(const    vertex& v) { _verts.push_back(v); }
          inline void push_t(const   texture& t) { _texts.push_back(t); }
          inline void push_n(const    vertex& n) { _norms.push_back(n); }
          inline void push_f(const      face& f) { _faces.push_back(f); }
          inline void push_m(      transform* m) { _trans.push_back(m); }

          /* accessors */
          inline const  vertex& v_at(unsigned int i) { return _verts.at(i); }
          inline const texture& t_at(unsigned int i) { return _texts.at(i); }
          inline const  vertex& n_at(unsigned int i) { return _norms.at(i); }

          inline unsigned int v_size() const { return _verts.size(); }
          inline unsigned int t_size() const { return _texts.size(); }
          inline unsigned int n_size() const { return _norms.size(); }
          inline unsigned int f_size() const { return _faces.size(); }

          inline  vertex_iterator vert_begin() const { return _verts.begin(); }
          inline  vertex_iterator vert_end()   const { return _verts.end();   }
          inline texture_iterator text_begin() const { return _texts.begin(); }
          inline texture_iterator text_end()   const { return _texts.end();   }
          inline  normal_iterator norm_begin() const { return _norms.begin(); }
          inline  normal_iterator norm_end()   const { return _norms.end();   }
          inline    face_iterator face_begin() const { return _faces.begin(); }
          inline    face_iterator face_end()   const { return _faces.end();   }

          inline transform_iterator tran_begin() const { return _trans.rbegin(); }
          inline transform_iterator tran_end()   const { return _trans.rend();   }

        private:
          std::vector<vertex>     _verts;
          std::vector<texture>    _texts;
          std::vector<vertex>     _norms;
          std::vector<face>       _faces;
          std::vector<transform*> _trans;
      };

      /* ******************************************************************** */
      /* *** how and what are we going to render **************************** */
      /* ******************************************************************** */

      class view {
        public:

          view(const std::string& name) : _name(name) { }
          virtual ~view() { }

          inline std::string name() const { return _name; }

          inline int  minx() const { return _minx; }
          inline int& minx()       { return _minx; }
          inline int  miny() const { return _miny; }
          inline int& miny()       { return _miny; }
          inline int  maxx() const { return _maxx; }
          inline int& maxx()       { return _maxx; }
          inline int  maxy() const { return _maxy; }
          inline int& maxy()       { return _maxy; }

        protected:

          std::string _name;
          int _minx, _miny;
          int _maxx, _maxy;
      };

      class wireframe : public view {
        public:
          wireframe(const std::string& name) : view(name) { }
          virtual ~wireframe() { }
      };

      class shader : public view {
        public:
          shader(const std::string& name) : view(name) { }
          virtual ~shader() { }
      };

      class camera {
        public:

          camera() { }
          camera(std::string name, double d) : _name(name), _d(d), _fp(),
              _vpn(), _vup() { }
          ~camera() { }

          inline std::string        name() const { return _name; }
          inline double                d() const { return _d;    }
          inline ray::vector    fp() const { return _fp;   }
          inline ray::vector&   fp()       { return _fp;   }
          inline ray::vector   vpn() const { return _vpn;  }
          inline ray::vector&  vpn()       { return _vpn;  }
          inline ray::vector   vup() const { return _vup;  }
          inline ray::vector&  vup()       { return _vup;  }

        private:

          std::string _name;
          double      _d;
          ray::vector _fp;
          ray::vector _vpn;
          ray::vector _vup;
      };

      class light {
        public:

          light() { }

          inline ray::vector  poss() const { return _poss; }
          inline ray::vector& poss()       { return _poss; }
          inline ray::vector  illu() const { return _illu; }
          inline ray::vector& illu()       { return _illu; }

        private:

          ray::vector _poss;
          ray::vector _illu;
      };

      class material {
        public:

          material() : _name("") { }
          material(const std::string& name) : _name(name) { }

          inline std::string   name() const { return _name;  }
          inline std::string&  name()       { return _name;  }
          inline ray::vector    rgb() const { return _rgb;   }
          inline ray::vector&   rgb()       { return _rgb;   }
          inline double           s() const { return _s;     }
          inline double&          s()       { return _s;     }
          inline double       alpha() const { return _alpha; }
          inline double&      alpha()       { return _alpha; }

        private:

          std::string _name;
          ray::vector _rgb;
          double _s, _alpha;
      };

      /* ******************************************************************** */
      /* *** the actual class *********************************************** */
      /* ******************************************************************** */

      objstream(const std::string& f_name);
      ~objstream();

      typedef std::map<std::string, group> group_map_t;
      typedef group_map_t::              iterator               iterator;
      typedef group_map_t::        const_iterator         const_iterator;
      typedef group_map_t::      reverse_iterator       reverse_iterator;
      typedef group_map_t::const_reverse_iterator const_reverse_iterator;

      inline               iterator begin()        { return _groups.begin();  }
      inline               iterator end()          { return _groups.end();    }
      inline         const_iterator begin()  const { return _groups.begin();  }
      inline         const_iterator end()    const { return _groups.end();    }
      inline       reverse_iterator rbegin()       { return _groups.rbegin(); }
      inline       reverse_iterator rend()         { return _groups.rend();   }
      inline const_reverse_iterator rbegin() const { return _groups.rbegin(); }
      inline const_reverse_iterator rend()   const { return _groups.rend();   }

      typedef std::vector<light*> light_vec_t;
      typedef light_vec_t::      iterator       l_iterator;
      typedef light_vec_t::const_iterator l_const_iterator;

      inline       l_iterator l_begin()       { return _lights.begin(); }
      inline       l_iterator l_end()         { return _lights.end();   }
      inline l_const_iterator l_begin() const { return _lights.begin(); }
      inline l_const_iterator l_end()   const { return _lights.end();   }

      typedef std::map<std::string, material> mat_map_t;
      typedef mat_map_t::      iterator       m_iterator;
      typedef mat_map_t::const_iterator m_const_iterator;

      inline       m_iterator m_begin()       { return _materials.begin(); }
      inline       m_iterator m_end()         { return _materials.end();   }
      inline m_const_iterator m_begin() const { return _materials.begin(); }
      inline m_const_iterator m_end()   const { return _materials.end();   }

      inline const std::string& src_file() const { return _fname;  }
      inline const std::string& mat_file() const { return _matlib; }

      inline void push(view* v) { _views.push_back(v); }
      inline void push_l(light* l) { _lights.push_back(l); }
      inline int  size() const { return _views.size(); }

      inline group& operator[](const std::string& name)
      { return _groups[name]; }

      inline camera& cam(const std::string& name)
      { return _cameras[name]; }

      inline material& mat(const std::string& name)
      { return _materials[name]; }

      inline view* operator[](int idx)
      { return _views[idx]; }

      private:

      std::map<std::string, group>    _groups;
      std::map<std::string, camera>   _cameras;
      std::map<std::string, material> _materials;
      std::vector<view*>              _views;
      std::vector<light*>             _lights;
      std::string                     _matlib;
      std::string                     _fname;
  };
}

std::ostream& operator<<(std::ostream& ostr, const obj::objstream& o);
std::ostream& operator<<(std::ostream& ostr, const obj::objstream::  group& g);
std::ostream& operator<<(std::ostream& ostr, const obj::objstream::texture& v);
std::ostream& operator<<(std::ostream& ostr, const obj::objstream:: vertex& v);
std::ostream& operator<<(std::ostream& ostr, const obj::objstream::   face& f);

#endif /* OBJSTREAM_HPP_INCLUDE */

