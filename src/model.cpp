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

/**
 * TODO
 */
ray::object::object() :
    _data(new double[4]), _size(0), _capa(1), _root(new ray::s_tree()) { }

/**
 * TODO
 *
 * @param obj
 */
ray::object::object(const object& obj) :
        _data(new double[obj._capa * 4]), _size(obj._size), _capa(obj._capa),
        _root(new ray::s_tree()) {
  memcpy(_data, obj._data, _capa * 4 * sizeof(double));
}

/**
 * TODO
 */
ray::object::~object() {
  delete[] _data;
  delete   _root;
}

/**
 * TODO
 *
 * @param obj
 * @return
 */
const ray::object& ray::object::operator =(const ray::object& obj) {
  if(this != &obj) {
    delete[] _data;

    _data = new double[obj._capa * V_SIZE];
    _size = obj._size;
    _capa = obj._capa;
    _root = obj._root;
    memcpy(_data, obj._data, _capa * V_SIZE * sizeof(double));
  }

  return obj;
}

/**
 * TODO
 *
 * @param v
 * @return
 */
unsigned int ray::object::index(const vector& v) const {
  return v.ptr() - _data;
}

/**
 * TODO
 *
 * @param v
 */
void ray::object::push_vector(const obj::objstream::vertex& v) {
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
void ray::object::push_normal(const obj::objstream::vertex& n) {
  _norm.push_back(ray::vector(n.x(), n.y(), n.z()));
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

/**
 * Destructor for the model class. needs to destroy surfaces since polymorphic
 * surfaces must be stored as pointers;
 */
ray::model::~model() { }

/**
 * Given a parser that has parsed a file, this will build the model.
 *
 * @param src the parser that will act as the source of the model
 */
void ray::model::build(const obj::objstream& src) {
  typedef obj::objstream src_t;

  ray::triangle* t;
  int size;

  for(src_t::const_iterator iter = src.begin(); iter != src.end(); iter++) {
    object* obj = new object();

    if((size = iter->second.n_size()) == 0) {
      model::smooth_shading = false;
    }

    for(src_t::group::vertex_iterator vi = iter->second.vert_begin();
        vi != iter->second.vert_end(); vi++) {
      obj->push_vector(*vi);
    }

    for(src_t::group::normal_iterator ni = iter->second.norm_begin();
        ni != iter->second.norm_end(); ni++) {
      obj->push_normal(*ni);
      obj->norm(obj->n_size() - 1).normalize();
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
        if(size == 0) t = new ray::triangle(*avi, *bvi, *cvi, 0, 0, 0, *obj);
        else t = new ray::triangle(*avi, *bvi, *cvi, *ani, *bni, *cni, *obj);

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

    for(src_t::group::transform_iterator ti = iter->second.tran_begin();
        ti != iter->second.tran_end(); ti++) {
      tran *= (*ti)->matrix();
    }

    obj *= tran;

    if(vertex_spheres) {
      for(unsigned int i = 0; i < obj.size(); i++) {
        ray::sphere* s = new ray::sphere(obj[i], model::vertex_spheres);
        s->material() = "vertex_sphere_color";
        obj.root()->push(s);
      }
    }

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

ray::vector ray::model::center() const {
  ray::vector ret;
  int total = 0;

  for(auto iter = begin(); iter != end(); iter++) {
    ray::object& obj = *iter->second;
    total += obj.size();

    for(unsigned int i = 0; i < obj.size(); i++) {
      ret += obj[i];
    }
  }

  ret[0] /= double(total);
  ret[1] /= double(total);
  ret[2] /= double(total);
  ret[3] = 0;

  return ret;
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


