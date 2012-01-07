/*
 * main.cpp
 *
 *  Created on: Aug 24, 2010
 *      Author: norton
 */

/* local includes */
#include <model.h>
#include <surface.h>
#include <camera.h>

/* library includes */
#include <algorithm>
#include <cstring>
#include <iostream>

bool ray::model::smooth_shading;
int  ray::model::vertex_spheres;

/* ************************************************************************** */
/* *** light and material *************************************************** */
/* ************************************************************************** */

/**
 * Creates a material based upon the material read from the input file
 */
ray::material::material(const obj::objstream::material& mat) :
    _name(mat.name()), _ks(mat.s()), _alpha(mat.alpha()), _kt(mat.t()),
    _diffuse(ray::identity<4>()) {
  _diffuse[0][0] = mat.rgb()[0];
  _diffuse[1][1] = mat.rgb()[1];
  _diffuse[2][2] = mat.rgb()[2];
}

/**
 * gets the direction that the light is from a certain ponit
 *
 * @param src the location which we are requesting info for
 * @return the direction of the light
 */
ray::vector ray::light::direction(ray::vector src) const {
  vector ret;

  ret = _position;
  if(_position[0] == 0) {
    ret.negate();
  } else {
    ret = ret - src;
  }

  return ret;
}

/* ************************************************************************** */
/* *** object *************************************************************** */
/* ************************************************************************** */

/**
 * basic contructor for the object class
 */
ray::object::object() :
    _data(new double[V_SIZE]), _size(0), _capa(1), _root(new ray::s_tree()),
    _norm() { }

/**
 * Copy contructor for object class.
 *
 * @param obj
 */
ray::object::object(const object& cpy) :
    _data(new double[cpy._capa * V_SIZE]), _size(cpy._size), _capa(cpy._capa),
     _root(cpy._root), _norm(cpy._norm) {
  memcpy(_data, cpy._data, _capa * V_SIZE * sizeof(double));
}

/**
 * TODO
 */
ray::object::~object() {
  //delete[] _data;
  //delete   _root;
}

/**
 * Pushes a new vertex onto the list of vertices for the model
 *
 * @param v
 */
void ray::object::push_v(const obj::objstream::vertex& v) {
  /* check if the object has enough space */
  if(_size == _capa) {
    _capa *= 2;
    double* dnew = new double[_capa * V_SIZE];
    memcpy(dnew, _data, _size * V_SIZE * sizeof(double));
    delete[] _data;
    _data = dnew;
  }

  int idx = V_SIZE * _size;

  /* put the new vector in the matrix */
  _data[idx + 0] = v.x();
  _data[idx + 1] = v.y();
  _data[idx + 2] = v.z();
  _data[idx + 3] = v.w();

  /* make sure it is normalized to w */
  if(v.w() != 1.0) {
    _data[idx + 0] /= _data[idx + 3];
    _data[idx + 1] /= _data[idx + 3];
    _data[idx + 2] /= _data[idx + 3];
    _data[idx + 3] /= _data[idx + 3];
  }
  _size++;
}

/**
 * TODO
 *
 * @param n
 */
void ray::object::push_n(const obj::objstream::vertex& n) {
  _norm.push_back(ray::vector(n.x(), n.y(), n.z()));
}

/**
 * TODO
 *
 * @param obj
 * @return
 */
const ray::object& ray::object::operator =(const ray::object& asn) {
  if(this != &asn) {
    delete[] _data;

    _data = new double[asn._capa * V_SIZE];
    _size = asn._size;
    _capa = asn._capa;
    memcpy(_data, asn._data, _capa * V_SIZE * sizeof(double));

    _norm = asn._norm;
    _root = asn._root;
  }

  return asn;
}

/**
 * TODO
 *
 * @param mat
 */
void ray::object::operator *=(const ray::matrix<4, 4>& mat) {
  double storage[4];

  for(unsigned int j = 0; j < _size; j++) {
    for(int i = 0; i < mat.rows(); i++) {
      storage[i] = 0;
      for(int k = 0; k < mat.cols(); k++) {
        storage[i] += at(j)[k] * mat[i][k];
      }
    }
    for(int i = 0; i < mat.rows(); i++) {
      at(j)[i] = storage[i];
    }
  }
}

/* ************************************************************************** */
/* *** model **************************************************************** */
/* ************************************************************************** */

/**
 * Basic constructor for the model class
 */
ray::model::model() { }

/**
 * Destructor for the model class. needs to destroy surfaces since polymorphic
 * surfaces must be stored as pointers;
 */
ray::model::~model() {
  for(std::pair<std::string, object*> obj : _objects) {
    delete obj.second;
  }
}

/**
 * Given a parser that has parsed a file, this will build the model.
 *
 * @param src the parser that will act as the source of the model
 */
void ray::model::build(const obj::objstream& src) {
  typedef obj::objstream src_t;
  ray::triangle* t;
  int size;

  if((size = src.n_size()) == 0) {
    model::smooth_shading = false;
  }

  for(src_t::const_iterator iter = src.begin(); iter != src.end(); iter++) {
    object* obj = new object();

    for(unsigned int i = 0; i < src.v_size(); i++) {
      obj->push_v(src.v_at(i));
    }

    for(unsigned int i = 0; i < src.n_size(); i++) {
      obj->push_n(src.n_at(i));
    }

    for(src_t::group::face_iterator fi = iter->second.face_begin();
        fi != iter->second.face_end(); fi++) {
      src_t::face::iterator avi = fi->v_begin();
      src_t::face::iterator bvi = fi->v_begin() + 1;
      src_t::face::iterator cvi = fi->v_begin() + 2;
      src_t::face::iterator ani = fi->n_begin();
      src_t::face::iterator bni = fi->n_begin() + 1;
      src_t::face::iterator cni = fi->n_begin() + 2;

      while(cvi != fi->v_end()) {

        if(size == 0) {
          t = new ray::triangle( *avi, *bvi, *cvi, 0, 0, 0, *obj);
        } else {
          t = new ray::triangle( *avi, *bvi, *cvi, *ani, *bni, *cni, *obj);
        }

        t->material() = fi->mat();
        obj->root()->push(t);

        if(size != 0)
          bni++; cni++;
        bvi++; cvi++;
      }
    }

    _objects[iter->first] = obj;
  }
}

/**
 * TODO
 *
 * @param src
 */
void ray::model::cmd(const obj::objstream& src) {
  typedef obj::objstream src_t;
  ray::sphere* sphere;

  if(vertex_spheres) {
    obj::objstream::material mat;
    mat.name() = "vertex_sphere_color";
    mat.rgb() = ray::vector(0, 0.5, 0);
    mat.s() = 0.49;
    mat.alpha() = 168;
    _materials["vertex_sphere_color"] = mat;
  }

  for(src_t::const_iterator iter = src.begin(); iter != src.end(); iter++) {
    object& obj = *_objects[iter->first];
    ray::matrix<4, 4> tran = ray::identity<4>();

    if(ray::model::vertex_spheres) {
      for(unsigned int i = 0; i < obj.v_size(); i++) {
        sphere = new ray::sphere(obj[i], model::vertex_spheres);
        sphere->material() = "vertex_sphere_color";
        obj.root()->push(sphere);
      }
    }

    for(src_t::group::transform_iterator ti = iter->second.tran_begin();
        ti != iter->second.tran_end(); ti++) {
      tran *= (*ti)->matrix();
    }

    obj *= tran;

    obj.root()->split();
    obj.root()->compute_bbox();
  }

  for(src_t::l_const_iterator iter = src.l_begin(); iter != src.l_end(); iter++) {
    _lights.push_back(**iter);
  }

  for(src_t::m_const_iterator iter = src.m_begin(); iter != src.m_end(); iter++) {
    _materials[iter->first] = iter->second;
  }
}

/**
 * TODO
 *
 * @param name
 * @return
 */
ray::material& ray::model::mat(const std::string& name) {
  if(_materials.find(name) == _materials.end())
    throw std::exception();
  return _materials[name];
}

/**
 * TODO
 *
 * @param m
 * @param p
 * @param v
 * @param n
 * @param s
 * @param shadows
 * @return
 */
ray::vector ray::model::reflectance(vector p, vector v, vector n,
    const surface* s, bool shadows) {
  /* locals */
  ray::material m = mat(s->material());
  ray::vector Lp, Rl;
  ray::vector ret;

  /* setup the necessary vectors */
  v.normalize();
  n.normalize();
  if(v.dot(n) < 0)
    n.negate();

  /* add each light to the color */
  for(auto light = l_begin(); light != l_end(); light++) {
    /* direct of the light source */
    Lp = light->direction(p);
    Lp.normalize();

    /* check for shadowed */
    if(Lp.dot(n) < 0 || (shadows && shadowed(p, light->direction(p), s))) {
      continue;
    }

    Rl = n * (Lp.dot(n) * 2) - Lp;
    Rl.normalize();

    ret += (m.diffuse() * light->illumination() * Lp.dot(n)) +
           (light->illumination() * m.ks() *
               std::pow(std::max(0.0, v.dot(Rl)), m.alpha()));
  }

  return ret;
}

/**
 *
 * @param m
 * @param pt
 * @param U
 * @param s
 * @return
 */
bool ray::model::shadowed(const ray::vector& pt, const ray::vector& U,
    const surface* s) const {
  ray::vector i, n;
  ray::vector tmp = U;
  ray::ray_info info(pt, tmp);
  double d;

  for(auto iter = begin(); iter != end(); iter++) {
    if(iter->second->root()->intersection(info, s, i, d, n)) {
      if(d > 0 && d < U.length()) {
        return true;
      }
    }
  }

  return false;
}

