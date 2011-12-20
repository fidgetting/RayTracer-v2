/*
 * Surface.cpp
 *
 *  Created on: Oct 25, 2010
 *      Author: norton
 */

#include <surface.h>
#include <camera.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
using std::get;

#define max3_corrdinate(x, y, z) \
  ((x > y) ? ((x > z) ? X : Z) : ((y > z) ? Y : Z))

#define max3(x, y, z) ((x > y) ? ((x > z) ? x : z) : ((y > z) ? y : z))
#define min3(x, y, z) ((x < y) ? ((x < z) ? x : z) : ((y < z) ? y : z))

#define THIS (*this)

#define BUNDING_FACTOR 4

int ray::surface::id_gen = 0;


/* ************************************************************************** */
/* *** local functions ****************************************************** */
/* ************************************************************************** */

static bool surface_comp_x(const ray::surface* l, const ray::surface* r) {
  return (2.0 * l->bounds().min()[ray::X] + l->bounds().len()[ray::X])
       < (2.0 * r->bounds().min()[ray::X] + l->bounds().len()[ray::X]);
}

static bool surface_comp_y(const ray::surface* l, const ray::surface* r) {
  return (2.0 * l->bounds().min()[ray::Y] + l->bounds().len()[ray::Y])
       < (2.0 * r->bounds().min()[ray::Y] + l->bounds().len()[ray::Y]);
}

static bool surface_comp_z(const ray::surface* l, const ray::surface* r) {
  return (2.0 * l->bounds().min()[ray::Z] + l->bounds().len()[ray::Z])
       < (2.0 * r->bounds().min()[ray::Z] + l->bounds().len()[ray::Z]);
}

/* ************************************************************************** */
/* *** object methods ******************************************************* */
/* ************************************************************************** */

/**
 * TODO
 *
 * @param ray
 */
ray::ray_info::ray_info(const ray::vector& L, const ray::vector& U) {
  _L = L;
  _U = U;

  _iL[X] = 1.0 / _L[X];
  _iL[Y] = 1.0 / _L[Y];
  _iL[Z] = 1.0 / _L[Z];
  _iU[X] = 1.0 / _U[X];
  _iU[Y] = 1.0 / _U[Y];
  _iU[Z] = 1.0 / _U[Z];

  nonzero[X] = _U[X] != 0.0;
  nonzero[Y] = _U[Y] != 0.0;
  nonzero[Z] = _U[Z] != 0.0;
  positive[X] = _U[X] > 0.0;
  positive[Y] = _U[Y] > 0.0;
  positive[Z] = _U[Z] > 0.0;

  print = false;
}

/**
 * Expands the current bounding box to make sure that is contains the given
 * bounding box.
 *
 * @param box the box to expand to contain
 */
void ray::bbox::expand_by(const ray::bbox& box) {
  if(contains(box))
    return;

  if(_len[X] == -1) {
    _min = box.min();
    _max = box.max();
    _len = box.len();
    return;
  }

  if(_min[X] > box._min[X]) _min[X] = box._min[X];
  if(_min[Y] > box._min[Y]) _min[Y] = box._min[Y];
  if(_min[Z] > box._min[Z]) _min[Z] = box._min[Z];

  if(_max[X] < box._max[X]) _max[X] = box._max[X];
  if(_max[Y] < box._max[Y]) _max[Y] = box._max[Y];
  if(_max[Z] < box._max[Z]) _max[Z] = box._max[Z];

  _len[X] = _max[X] - _min[X];
  _len[Y] = _max[Y] - _min[Y];
  _len[Z] = _max[Z] - _min[Z];
}

/**
 * Tests if one bouding box is completely contained within another bounding box
 *
 * @param box the box to test if it is within this box
 * @return true if contained, false otherwise
 */
bool ray::bbox::contains(const ray::bbox& box) const {
  if((box._min[X] < _min[X]) ||
     (box._min[Y] < _min[Y]) ||
     (box._min[Z] < _min[Z]))
    return false;

  if((box._max[X] > _max[X]) ||
     (box._max[Y] > _max[Y]) ||
     (box._max[Z] > _max[Z]))
    return false;

  return true;
}

/**
 * Check if a ray intersect a bounding box. This is used for the bounding box
 * tree optimization.
 *
 * @param ray some pre-compiled information about the ray
 * @return true if the ray intersect, false otherwise
 */
bool ray::bbox::intersection(const ray::ray_info& ray) const {
  double tmin, tmax;
  double dmin, dmax;

  if(ray.nonzero[X]) {
    if(ray.positive[X]) {
      dmin = (_min[X] - ray._L[X]) * ray._iU[X];
      dmax = dmin + (_len[X] * ray._iU[X]);
      if(dmax < EPSILON)
        return false;
    } else {
      dmax = (_min[X] - ray._L[X]) * ray._iU[X];
      if(dmax < EPSILON)
        return false;
      dmin = dmax + (_len[X] * ray._iU[X]);
    }

    if(dmin > dmax)
      return false;
  } else {
    if((ray._L[X] < _min[X]) || (ray._L[X] > _len[X] + _min[X])) {
      return false;
    }

    dmin = DBL_MIN;
    dmax = DBL_MAX;
  }

  if(ray.nonzero[Y]) {
    if(ray.positive[Y]) {
      tmin = (_min[Y] - ray._L[Y]) * ray._iU[Y];
      tmax = tmin + (_len[Y] * ray._iU[Y]);
    } else {
      tmax = (_min[Y] - ray._L[Y]) * ray._iU[Y];
      tmin = tmax + (_len[Y] * ray._iU[Y]);
    }

    if(tmax < dmax) {
      if(tmax < EPSILON)
        return false;
      if(tmin > dmin) {
        if(tmin > tmax)
          return false;
        dmin = tmin;
      } else if(dmin > tmax) {
        return false;
      }
      dmax = tmax;
    } else {
      if(tmin > dmin) {
        if(tmin > dmax)
          return false;
        dmin = tmin;
      }
    }
  } else {
    if((ray._L[Y] < _min[Y]) || (ray._L[Y] > _len[Y] + _min[Y])) {
      return false;
    }
  }

  if(ray.nonzero[Z]) {
    if(ray.positive[Z]) {
      tmin = (_min[Z] - ray._L[Z]) * ray._iU[Z];
      tmax = tmin + (_len[Z] * ray._iU[Z]);
    } else {
      tmax = (_min[Z] - ray._L[Z]) * ray._iU[Z];
      tmin = tmax + (_len[Z] * ray._iU[Z]);
    }

    if(tmax < dmax) {
      if(tmax < EPSILON)
        return false;
      if(tmin > dmin) {
        if(tmin > tmax)
          return false;
        dmin = tmin;
      } else if(dmin > tmax) {
        return false;
      }
      dmax = tmax;
    } else {
      if(tmin > dmin) {
        if(tmin > dmax)
          return false;
        dmin = tmin;
      }
    }
  } else {
    if((ray._L[Y] < _min[Y]) || (ray._L[Y] > _len[Y] + _min[Y])) {
      return false;
    }
  }

  return true;
}

/**
 *
 * @param cpy
 */
ray::s_tree::s_tree(const ray::s_tree& cpy) {
  ray::s_tree* tree;
  ray::sphere* sph;
  ray::triangle* tri;

  _bounds = cpy._bounds;

  for(surface* s : cpy._kids) {
    if((tree = dynamic_cast<ray::s_tree*>(s)) != NULL)
      _kids.push_back(new ray::s_tree(*tree));
    else if((sph = dynamic_cast<ray::sphere*>(s)) != NULL)
      _kids.push_back(new ray::sphere(*sph));
    else if((tri = dynamic_cast<ray::triangle*>(s)) != NULL)
      _kids.push_back(new ray::triangle(*tri));
  }
}

/**
 * TODO
 */
ray::s_tree::~s_tree() {
  for(surface* curr : _kids) {
    delete curr;
  }
}

/**
 * Computes the bounding box for a surface tree
 *
 * @return the new bounding box
 */
const ray::bbox& ray::s_tree::compute_bbox() {
  _bounds.len()[X] = -1;
  _bounds.len()[Y] = -1;
  _bounds.len()[Z] = -1;

  for(surface* curr : _kids) {
    _bounds.expand_by(curr->compute_bbox());
  }

  return _bounds;
}

/**
 * TODO
 *
 * @param ray
 * @param skip
 * @param I
 * @param r
 * @param n
 * @return
 */
const ray::surface* ray::s_tree::intersection(const ray::ray_info& ray,
    const surface* skip, vector& I, double& r, vector& n) const {
  const ray::surface* best = NULL, * T_s;
  ray::vector T_I;
  ray::vector T_n;
  double T_r;

  if(!_bounds.intersection(ray))
    return NULL;

  r = DBL_MAX;

  for(surface* curr : _kids) {
    if(curr->bounds().intersection(ray)) {
      if((T_s = curr->intersection(ray, skip, T_I, T_r, T_n)) && T_r < r) {
        best = T_s;
        I = T_I;
        r = T_r;
        n = T_n;
      }
    }
  }

  return best;
}

/**
 * TODO
 *
 * @param s
 */
void ray::s_tree::push(ray::surface* s) {
  _bounds.expand_by(s->bounds());
  _kids.push_back(s);
}

/**
 *
 *
 */
void ray::s_tree::split() {
  int axis = X;
  ray::s_tree* l;
  ray::s_tree* r;
  double e, d = 0;

  /* if the group is small enough, return */
  if(_kids.size() < BUNDING_FACTOR)
    return;

  /* choose axis to split along */
  d = _bounds.max()[X] - _bounds.min()[X];
  e = _bounds.max()[Y] - _bounds.min()[Y];
  if(e > d) { d = e; axis = Y; }
  e = _bounds.max()[Z] - _bounds.min()[Z];
  if(e > d) { axis = Z; }

  /* sort along relevant axis */
  std::sort(_kids.begin(), _kids.end(),
      axis == X ? surface_comp_x :
      axis == Y ? surface_comp_y : surface_comp_z);

  l = new s_tree();
  r = new s_tree();

  for(unsigned int i = 0; i < (_kids.size() / 2); i++) {
    l->push(_kids[i]);
  }

  for(unsigned int i = (_kids.size() / 2); i < _kids.size(); i++) {
    r->push(_kids[i]);
  }

  _kids.clear();

  this->push(l);
  this->push(r);

  l->split();
  r->split();
}

/**
 * Gets the number of object in this surface tree
 *
 * @return the number of objects in the tree
 */
unsigned int ray::s_tree::size() const {
  s_tree* surf;
  unsigned int s = 0;

  for(surface* curr : _kids) {
    if((surf = dynamic_cast<s_tree*>(curr)) != NULL) {
      s += surf->size();
    } else {
      s++;
    }
  }

  return s;
}

/**
 * TODO
 *
 * @param _center
 * @param _radius
 */
ray::sphere::sphere(const ray::vector& _center, double _radius) :
    _center(_center), _radius(_radius) { }

/**
 * TODO
 *
 * @param v
 * @return
 */
ray::vector ray::sphere::normal(const ray::vector& v) const {
  return v - _center;
}

/**
 * Computes a bounding box for a sphere
 *
 * @return the new bouding box
 */
const ray::bbox& ray::sphere::compute_bbox() {
  _bounds.min()[X] = _center[X] - _radius;
  _bounds.min()[Y] = _center[Y] - _radius;
  _bounds.min()[Z] = _center[Z] - _radius;
  _bounds.max()[X] = _center[X] + _radius;
  _bounds.max()[Y] = _center[Y] + _radius;
  _bounds.max()[Z] = _center[Z] + _radius;
  _bounds.len()[X] = _bounds.max()[X] - _bounds.min()[X];
  _bounds.len()[Y] = _bounds.max()[Y] - _bounds.min()[Y];
  _bounds.len()[Z] = _bounds.max()[Z] - _bounds.min()[Z];

  return _bounds;
}

/**
 * Calculate the intersection between a ray and a sphere.
 *
 * @param U
 * @param L
 * @param skip
 * @return
 */
const ray::surface* ray::sphere::intersection(const ray::ray_info& ray,
    const surface* skip, vector& I, double& r, vector& n) const {
  double s, t_sq, r_sq, m_sq, q, tmp;
  const ray::surface* surf = NULL;
  const ray::surface* best = NULL;
  ray::vector U = ray.U(), L = ray.L();
  ray::vector T = center() - L;
  ray::vector I_tmp, n_tmp;

  if(skip == this && U.dot(normal(L)) > 0) {
    return NULL;
  }

  /* perform first check to see if we hit the circle */
  s = T.dot(U);
  t_sq = T.dot(T);
  r_sq = radius() * radius();
  if(s < 0 && t_sq > r_sq) {
    return NULL;
  }

  /* second easy rejection check */
  m_sq = t_sq - s*s;
  if(m_sq > r_sq) {
    return NULL;
  }

  /* we now know that the ray will intersect the sphere, fork for subsurfaces */
  if(_subsurfaces.size() == 0) {
    q = sqrt(r_sq - m_sq);
    if(t_sq > r_sq && (this != skip || U.dot(normal(L)) >= 0)) {
      s -= q;
    } else {
      s += q;
    }

    I = L + (U * s);
    r = s;
    n = this->normal(I);
    return this;
  }

  for(auto iter = _subsurfaces.begin(); iter != _subsurfaces.end(); iter++) {
    if(*iter != skip) {
      surf = (*iter)->intersection(ray, skip, I_tmp, tmp, n_tmp);

      if(surf && tmp >= 0 && tmp < r) {
        I    = I_tmp;
        r    = tmp;
        best = surf;
        n    = n_tmp;
      }
    }
  }

  return best;
}

/**
 * TODO
 *
 * @param a
 * @param b
 * @param c
 * @param owner
 */
ray::triangle::triangle(uint32_t va, uint32_t vb, uint32_t vc,
         uint32_t na, uint32_t nb, uint32_t nc,
         ray::object& owner) : _v_idx(), _n_idx(), _n(),
         _perp(), _owner(owner), _d_axis(2), _v_axis(2) {
  ray::vector v1, v2, P3minusP2;
  double x, y, z;
  bool swap = false;

  _v_idx[0] = va;
  _v_idx[1] = vb;
  _v_idx[2] = vc;
  _n_idx[0] = na;
  _n_idx[1] = nb;
  _n_idx[2] = nc;

  v1 = THIS[0] - THIS[1];
  v2 = THIS[2] - THIS[1];

  _n = v1.cross(v2);
  _n.normalize();

  x = fabs(_n[0]);
  y = fabs(_n[1]);
  z = fabs(_n[2]);

  _d_axis = max3_corrdinate(x, y, z);

  switch(_d_axis) {
    case X:
      if((THIS[1][Y] - THIS[2][Y]) * (THIS[1][Z] - THIS[0][Z]) <
         (THIS[1][Z] - THIS[2][Z]) * (THIS[1][Y] - THIS[0][Y]))
        swap = true;
      break;
    case Y:
      if((THIS[1][X] - THIS[2][X]) * (THIS[1][Z] - THIS[0][Z]) <
         (THIS[1][Z] - THIS[2][Z]) * (THIS[1][X] - THIS[0][X]))
        swap = true;
      break;
    case Z:
      if((THIS[1][X] - THIS[2][X]) * (THIS[1][Y] - THIS[0][Y]) <
         (THIS[1][Y] - THIS[2][Y]) * (THIS[1][X] - THIS[0][X]))
        swap = true;
      break;
  }

  if(swap) {
    std::swap(_v_idx[0], _v_idx[1]);
    std::swap(_n_idx[0], _n_idx[1]);
  }

  P3minusP2 = THIS[2] - THIS[1];

  x = fabs(P3minusP2[X]);
  y = fabs(P3minusP2[Y]);
  z = fabs(P3minusP2[Z]);

  _v_axis = max3_corrdinate(x, y, z);

  v1 = THIS[1] - THIS[2];
  v2 = THIS[0] - THIS[2];

  v2 = v2.cross(v1);
  _perp = v2.cross(v1);
  _perp.normalize();
}

/**
 * TODO
 *
 * @param inter
 * @return
 */
ray::vector ray::triangle::normal(const ray::vector& inter) const {
  double u, v;
  vector diff, nt1, nt2;

  if(!model::smooth_shading)
    return _n;

  u = (inter - THIS[0]).dot(_perp) / (THIS[2] - THIS[0]).dot(_perp);
  diff = THIS[0] + ((inter - THIS[0]) / u);
  v = ((diff[_v_axis]    - THIS[1][_v_axis]) /
       (THIS[2][_v_axis] - THIS[1][_v_axis]));

#define NORM(i) _owner.norm(_n_idx[i])
  nt1 = NORM(0) + ((NORM(1) - NORM(0)) * u);
  nt2 = NORM(0) + ((NORM(2) - NORM(0)) * u);

  diff = nt1 + ((nt2 - nt1) * v);

  diff.normalize();
#undef NORM

  return diff;
}

/**
 * Computes the bounding box for the triangle.
 *
 * @return the new bounding box
 */
const ray::bbox& ray::triangle::compute_bbox() {
  _bounds.min()[X] = min3(THIS[0][X], THIS[1][X], THIS[2][X]);
  _bounds.min()[Y] = min3(THIS[0][Y], THIS[1][Y], THIS[2][Y]);
  _bounds.min()[Z] = min3(THIS[0][Z], THIS[1][Z], THIS[2][Z]);
  _bounds.max()[X] = max3(THIS[0][X], THIS[1][X], THIS[2][X]);
  _bounds.max()[Y] = max3(THIS[0][Y], THIS[1][Y], THIS[2][Y]);
  _bounds.max()[Z] = max3(THIS[0][Z], THIS[1][Z], THIS[2][Z]);
  _bounds.len()[X] = _bounds.max()[X] - _bounds.min()[X];
  _bounds.len()[Y] = _bounds.max()[Y] - _bounds.min()[Y];
  _bounds.len()[Z] = _bounds.max()[Z] - _bounds.min()[Z];

  return _bounds;
}

/**
 * TODO
 *
 * @param U
 * @param L
 * @param skip
 * @param I
 * @param r
 * @param no
 * @return
 */
const ray::surface* ray::triangle::intersection(const ray::ray_info& ray,
    const surface* skip, vector& I, double& r, vector& no) const {
  double a, b, uu, uv, vv, wu, wv, D, gama, beta;
  vector u, v, n ,w;
  vector U = ray.U(), L = ray.L();


  if(skip == this)
    return NULL;

  u = THIS[1] - THIS[0];
  v = THIS[2] - THIS[0];
  n = u.cross(v);

  a = -n.dot(L - THIS[0]);
  b =  n.dot(U);
  if(fabs(b) < EPSILON)
    return NULL;

  if((r = a / b) < 0.0)
    return NULL;

  I  = L + (U * r);
  uu = u.dot(u);
  uv = u.dot(v);
  vv = v.dot(v);
  w = I - THIS[0];
  wu = w.dot(u);
  wv = w.dot(v);
  D = uv * uv - uu * vv;

  gama = (uv * wv - vv * wu) / D;
  if(gama < 0.0 || gama > 1.0)
    return NULL;
  beta = (uv * wu - uu * wv) / D;
  if(beta < 0.0 || (gama + beta) > 1.0)
    return NULL;

  no = normal(I);
  return this;
}
