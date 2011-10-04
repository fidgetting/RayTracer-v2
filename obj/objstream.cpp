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

#include <objstream.hpp>

#include <cstring>
#include <cmath>
#include <exception>
#include <fstream>

extern int yyparse(void);
extern FILE* yyin;

obj::objstream* obj::dest;

/**
 * creates a transformation matrix using the paramters specified in the input
 * file
 *
 * @return rotate transformation matrix
 */
ray::matrix<4, 4> obj::objstream::rotate::matrix() const {
  ray::matrix<4, 4> R, z;
  ray::vector vecs[3];
  ray::vector m;

  /* create w */
  vecs[2] = _w;
  vecs[2].normalize();

  /* create u */
  m = vecs[2];
  if(     m[0] <= m[1] && m[0] <= m[2]) m[0] = 1;
  else if(m[1] <= m[2])                 m[1] = 1;
  else                                  m[2] = 1;
  m.normalize();
  vecs[0] = vecs[2].cross(m);
  vecs[0].normalize();

  /* create v */
  vecs[1] = vecs[0].cross(vecs[2]);
  vecs[1].normalize();

  /* create R */
  R[3][3] = 1;
  for(int i = 0; i < 3; i++)
    for(int j = 0; j < 3; j++)
      R[i][j] = vecs[i][j];

  /* create z */
  z[0][0] =  std::cos(_r);
  z[0][1] = -std::sin(_r);
  z[1][0] =  std::sin(_r);
  z[1][1] =  std::cos(_r);
  z[2][2] = 1;
  z[3][3] = 1;

  return R.t() * z * R;
}

/**
 * creates a transformation matrix using the paramters specified in the input
 * file
 *
 * @return translate transformation matrix
 */
ray::matrix<4, 4> obj::objstream::translate::matrix() const {
  ray::matrix<4, 4> ret = ray::identity<4>();

  ret[0][3] = _x;
  ret[1][3] = _y;
  ret[2][3] = _z;
  ret[3][3] = 1.0;

  return ret;
}

/**
 * creates a transformation matrix using the paramters specified in the input
 * file
 *
 * @return scale transformation matrix
 */
ray::matrix<4, 4> obj::objstream::scale::matrix() const {
  ray::matrix<4, 4> ret;

  ret[0][0] = _x;
  ret[1][1] = _y;
  ret[2][2] = _z;
  ret[3][3] = 1.0;

  return ret;
}

/**
 * TODO
 *
 * @return
 */
ray::matrix<4, 4> obj::objstream::arbitrary::matrix() const {
  return _mat;
}

/**
 * TODO
 */
obj::objstream::group::~group() {
  for(std::vector<transform*>::iterator iter = _trans.begin();
      iter != _trans.end(); iter++) {
    delete *iter;
  }
}

/**
 * Ctor for the objstream class. This takes the file, opens and parses it.
 *
 * @param f_name obj file name
 */
obj::objstream::objstream(const std::string& f_name) : _fname(f_name) {
  /* open the file and place it in yyin */
  if(!(yyin = fopen(f_name.c_str(), "r")))
    throw std::exception();

  /* the parser will place everything directly in the objstream. Set a *
   * pointer so that it can do this correctly                          */
  dest = this;

  /* parse the close the input file */
  yyparse();
  fclose(yyin);
}

/**
 * TODO
 *
 * @param ostr
 * @param o
 * @return
 */
std::ostream& operator<<(std::ostream& ostr, const obj::objstream& o) {
  for(obj::objstream::const_iterator iter = o.begin(); iter != o.end(); iter++){
    ostr << iter->first  << ":" << std::endl;
    ostr << iter->second << std::endl;
  }
  return ostr;
}

/**
 * TODO
 *
 * @param ostr
 * @param g
 * @return
 */
std::ostream& operator<<(std::ostream& ostr, const obj::objstream::  group& g) {
  ostr << "  vertices:" << std::endl;
  for(obj::objstream::group::vertex_iterator iter = g.vert_begin();
      iter != g.vert_end(); iter++) {
    ostr << "    " << (*iter) << std::endl;
  }

  ostr << "  textures:" << std::endl;
  for(obj::objstream::group::texture_iterator iter = g.text_begin();
      iter != g.text_end(); iter++) {
    ostr << "    " << (*iter) << std::endl;
  }

  ostr << "  normals:" << std::endl;
  for(obj::objstream::group::normal_iterator iter = g.norm_begin();
      iter != g.norm_end(); iter++) {
    ostr << "    " << (*iter) << std::endl;
  }

  ostr << "  faces:" << std::endl;
  for(obj::objstream::group::face_iterator iter = g.face_begin();
      iter != g.face_end(); iter++) {
    ostr << "    " << (*iter) << std::endl;
  }

  return ostr;
}

/**
 * TODO
 *
 * @param ostr
 * @param t
 * @return
 */
std::ostream& operator<<(std::ostream& ostr, const obj::objstream::texture& t) {
  return ostr << "[" << t.x() << "," << t.y() << "]";
}

/**
 * TODO
 *
 * @param ostr
 * @param v
 * @return
 */
std::ostream& operator<<(std::ostream& ostr, const obj::objstream::vertex& v) {
  return ostr << "[" << v.x() << "," << v.y() << "," << v.z() << "]";
}

/**
 * TODO
 *
 * @param ostr
 * @param f
 * @return
 */
std::ostream& operator<<(std::ostream& ostr, const obj::objstream::face& f) {
  ostr << "face: [ ";
  obj::objstream::face::iterator v, t, n;
  v = f.v_begin();
  t = f.t_begin();
  n = f.n_begin();

  while(v != f.v_end()) {
    ostr << *v;

    if(f.t_size() != 0 || f.n_size() != 0)
      ostr << "/";
    if(f.t_size() != 0) {
      ostr << *t;
      t++;
    }
    if(f.n_size() != 0) {
      ostr << "/" << *n;
      n++;
    }

    ostr << " ";
    v++;
  }

  return ostr << "]" << f.mat();
}
