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
ray::object::object() : _data(new double[4]), _size(0), _capa(1), _surf() { }

/**
 * TODO
 *
 * @param obj
 */
ray::object::object(const object& obj) :
        _data(new double[obj._capa * 4]), _size(obj._size), _capa(obj._capa),
        _surf(obj._surf) {
  memcpy(_data, obj._data, _capa * 4 * sizeof(double));
}

/**
 * TODO
 */
ray::object::~object() {
  delete[] _data;
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
    _surf = obj._surf;
    memcpy(_data, obj._data, _capa * V_SIZE * sizeof(double));
  }

  return obj;
}

/**
 * TODO
 *
 * @param p
 */
void ray::object::push_polygon(const ray::polygon& p) {
  _surf.push_back(p);
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

  for(src_t::const_iterator iter = src.begin(); iter != src.end(); iter++) {
    object* obj = new object();

    for(src_t::group::vertex_iterator vi = iter->second.vert_begin();
        vi != iter->second.vert_end(); vi++) {
      obj->push_vector(*vi);
    }

    for(src_t::group::face_iterator fi = iter->second.face_begin();
        fi != iter->second.face_end(); fi++) {
      polygon p(obj);

      for(src_t::face::iterator ii = fi->v_begin(); ii != fi->v_end(); ii++) {
        p.add_vertex(*ii);
      }

      obj->push_polygon(p);
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

  for(src_t::const_iterator iter = src.begin(); iter != src.end(); iter++) {
    object& obj = *_objects[iter->first];
    ray::matrix<4, 4> tran = ray::identity<4>();

    for(src_t::group::transform_iterator ti = iter->second.tran_begin();
        ti != iter->second.tran_end(); ti++) {
      tran *= (*ti)->matrix();
    }

    obj *= tran;
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
 * @param ostr
 * @param m
 * @return
 */
std::ostream& operator<<(std::ostream& ostr, const ray::model& m) {
  ostr << "# output.obj" << std::endl;
  ostr << "# "           << std::endl;
  ostr << "# file generated for ray tracer" << std::endl;
  ostr << "# ray tracer written by Alex Norton" << std::endl << std::endl;

  for(ray::model::const_iterator iter = m.begin(); iter != m.end(); iter++) {
    ostr << "g " << iter->first << "\n\n";
    ostr << *(iter->second) << "\n";
  }

  return ostr << std::endl;
}

/**
 * TODO
 *
 * @param ostr
 * @param o
 * @return
 */
std::ostream& operator<<(std::ostream& ostr, const ray::object& o) {
  for(unsigned int i = 0; i < o.size(); i++) {
    ostr << o[i] << "\n";
  }
  ostr << "\n";

  for(ray::object::const_iterator iter = o.begin(); iter != o.end(); iter++) {
    ostr << "f ";
    for(ray::polygon::const_iterator pi = iter->begin(); pi != iter->end(); pi++) {
      ostr << *pi << " ";
    }
    ostr << "\n";
  }

  return ostr << std::endl;
}

/* ************************************************************************** */
/* *** main function for ray tracer ***************************************** */
/* ************************************************************************** */

int main(int argc, char** argv) {

  if(argc != 3) {
    std::cout << "usage: " << argv[0] << " [transforms] [objects]" << std::endl;
    return -1;
  }

  obj::objstream obj(argv[2]);
  obj::objstream cmd(argv[1]);
  ray::model m;

  m.build(obj);
  m.cmd(cmd);

  for(int i = 0;i < cmd.size(); i++) {
    ray::camera c(cmd.cam(cmd[i]->name()));
    std::ostringstream ostr;

    c.umin() = cmd[i]->minx();
    c.umax() = cmd[i]->maxx();
    c.vmin() = cmd[i]->miny();
    c.vmax() = cmd[i]->maxy();

    typedef cv::Vec<uc, 3> elem_t;
    cv::Mat image(c.vmax() - c.vmin(), c.umax() - c.umin(), CV_8UC3);

    for(int j = 0; j < 500; j++) {
      for(auto iter = image.begin<elem_t>();
          iter != image.end<elem_t>(); iter++) {
        *iter = elem_t();
      }

      if(dynamic_cast<obj::objstream::wireframe*>(cmd[i]) != NULL) {
        c.draw_wire(&m, image);
      }

      //cv::imshow(cmd[i]->name(), image);
      //cv::waitKey(10);

      c.rotate(0.034906585, ray::vector());
    }

    ostr << "files/" << cmd[i]->name() << ".png";
    cv::imwrite(ostr.str().c_str(), image);
  }

  return 0;
}


