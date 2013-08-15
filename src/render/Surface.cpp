/*
 * Surface.cpp
 *
 *  Created on: Dec 31, 2012
 *      Author: norton
 */

/* local includes */
#include <Debug.hpp>
#include <Model.hpp>
#include <Surface.hpp>
#include <Ray.hpp>

/* std includes */
#include <algorithm>
#include <limits>

namespace ray {

  uint32_t Surface::idgen = 0;

  /**
   * Determines if a Ray and a Surface intersect. This also calculates the
   * location of the intersection.
   *
   * @param ray    The Ray that needs to be tested
   * @param skip   The Surface that should be skipped
   * @param inter  Returns location of the intersection
   * @return       True if the Ray Intersected the Surface
   */
  bool Surface::intersect(const Ray& ray, Intersection& inter) const {
    if(!getBounds().intersect(ray))
      return false;
    return getIntersection(ray, inter);
  }

  Surface::operator render::d_Surface() const {
    return getDevice();
  }

  /* ************************************************************************ */
  /* *** Box **************************************************************** */
  /* ************************************************************************ */

  /**
   * Constructs a Box that contains both of the given Boxes.
   *
   * @param a  A Box
   * @param b  A Box
   */
  Box::Box(const Box& a, const Box& b) :
    _min(ray::min(a.min(), b.min())),
    _len(ray::max(a.min() + a.len(), b.min() + b.len()) - _min) { }

  /**
   * Tests if this Box completely conatins another Box.
   *
   * @param box  the other Box
   * @return     true if the other Box is contained
   */
  bool Box::contains(const Box& box) const {
    if(box.min().x() < min().x() ||
       box.min().y() < min().y() ||
       box.min().z() < min().z())
      return false;

    if(box.len().x() > len().x() ||
       box.len().y() > len().y() ||
       box.len().z() > len().z())
      return false;

    return true;
  }

  /**
   * Intersects a Box with a Ray. Box are a simple optimization to check if a
   * Ray passes through a particular area of the world. This function tries to
   * exit as soon as possible to limit the overhead of checking a Box
   * intersection.
   *
   * @param ray  The Ray to test
   * @return     if the Ray passes through this region.
   */
  bool Box::intersect(const Ray& ray) const {
    double tmin, tmax;
    double dmin, dmax;

    if(ray.zero(0)) {
      if(ray.posi(0)) {
        dmin = (min().x() - ray.L().x()) * ray.iU().x();
        dmax = dmin + (len().x() * ray.iU().x());
        if(dmax < EPSILON)
          return false;
      } else {
        dmax = (min().x() - ray.L().x()) * ray.iU().x();
        if(dmax < EPSILON)
          return false;
        dmin = dmax + (len().x() * ray.iU().x());
      }

      if(dmin > dmax)
        return false;
    } else {
      if((ray.L().x() < min().x()) || (ray.L().x() > len().x() + min().x())) {
        return false;
      }

      dmin = std::numeric_limits<double>::min();
      dmax = std::numeric_limits<double>::max();
    }

    if(ray.zero(1)) {
      if(ray.posi(1)) {
        tmin = (min().y() - ray.L().y()) * ray.iU().y();
        tmax = tmin + (len().y() * ray.iU().y());
      } else {
        tmax = (min().y() - ray.L().y()) * ray.iU().y();
        tmin = tmax + (len().y() * ray.iU().y());
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
      if((ray.L().y() < min().y()) || (ray.L().y() > len().y() + min().y())) {
        return false;
      }
    }

    if(ray.zero(2)) {
      if(ray.posi(2)) {
        tmin = (min().z() - ray.L().z()) * ray.iU().z();
        tmax = tmin + (len().z() * ray.iU().z());
      } else {
        tmax = (min().z() - ray.L().z()) * ray.iU().z();
        tmin = tmax + (len().z() * ray.iU().z());
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
      if((ray.L().z() < min().z()) || (ray.L().z() > len().z() + min().z())) {
        return false;
      }
    }

    return true;
  }

  /* ************************************************************************ */
  /* *** Surface Tree ******************************************************* */
  /* ************************************************************************ */

  /**
   * Get the Bounding Box for this SurfaceTree.
   *
   * @return  the Box that is the region this Surface occupies.
   */
  Box SurfaceTree::getBounds() const {
    return bounds;
  }

  /**
   * Gets the Intersection of a Ray and a SurfaceTree. The return of this
   * function indicates if the Ray intersected the surface tree and the inter
   * parameter returns the location of the intersection.
   *
   * @param ray    the Ray to intersect
   * @param skip   the Surface that the ray originated on
   * @param inter  return for the location of the intersection
   * @return       true if an intersection was found.
   */
  bool SurfaceTree::getIntersection(const Ray& ray, Intersection& inter) const {
    Intersection best;
    Intersection curr;
    bool found = false;

    for(Surface::ptr surf : children) {
      if(surf->intersect(ray, curr)) {
        best  = Intersection::best(best, curr);
        found = true;
      }
    }

    inter = best;
    return found;
  }

  render::d_Surface SurfaceTree::getDevice() const {
    render::d_Surface ret;

    ret.id  = id;
    ret.mat = 0;
    ret.min = bounds.min();
    ret.len = bounds.len();

    ret.which = render::d_Surface::tree;

    ret.d_axis = -1;
    ret.v_axis = -1;

    if(children.size() == 1) {
      ret.d_axis = children[0]->id;
    } else if(children.size() == 2) {
      ret.d_axis = children[0]->id;
      ret.v_axis = children[1]->id;
    }

    return ret;
  }

  /* ************************************************************************ */
  /* *** Triangle *********************************************************** */
  /* ************************************************************************ */

#define max3_coordinate(x, y, z) ((x > y) ? ((x > z) ? 0 : 2) : ((y > z) ? 1 : 2))
#define max3(x, y, z)            ((x > y) ? ((x > z) ? x : z) : ((y > z) ? y : z))
#define min3(x, y, z)            ((x < y) ? ((x < z) ? x : z) : ((y < z) ? y : z))

  /**
   * Constructor the a Triangle. This needs to be passed the 3 Vectors that at
   * the vertices for the Triangle and the normals for the Vectors.
   *
   * @param va  first Vector
   * @param vb  second Vector
   * @param vc  third Vector
   * @param na  first normal
   * @param nb  second normal
   * @param nc  third normal
   */
  Triangle::Triangle(RefVector _va, RefVector _vb, RefVector _vc,
                     RefVector _na, RefVector _nb, RefVector _nc,
                     uint16_t material) :
      Surface(material),
      va(_va), vb(_vb), vc(_vc),
      na(_na), nb(_nb), nc(_nc),
      _norm(0, 0, 0),
      _perp(0, 0, 0),
      d_axis(2),
      v_axis(2),
      bounds()
  {
    double x, y, z;
    bool swap = false;

    auto a = va - vb;
    auto b = vc - vb;
    _norm = cross(a, b).normalize();

    x = fabs(_norm.x());
    y = fabs(_norm.y());
    z = fabs(_norm.z());
    switch((d_axis = max3_coordinate(x, y, z))) {
      case 0:
        if((vb.y() - vc.y()) * (vb.z() - va.z()) <
           (vb.z() - vc.z()) * (vb.y() - va.y()))
          swap = true;
        break;

      case 1:
        if((vb.x() - vc.x()) * (vb.z() - va.z()) <
           (vb.z() - vc.z()) * (vb.x() - va.x()))
          swap = true;
        break;

      case 2:
        if((vb.x() - vc.x()) * (vb.y() - va.y()) <
           (vb.y() - vc.y()) * (vb.x() - va.x()))
          swap = true;
        break;
    }

    if(swap) {
      std::swap(va, vb);
      std::swap(na, nb);
    }

    x = fabs(b.x());
    y = fabs(b.y());
    z = fabs(b.z());
    v_axis = max3_coordinate(x, y, z);

    a = vb - vc;
    b = va - vc;
    _perp = cross(cross(b, a), a).normalize();

    a = min(min(va, vb), vc);
    b = max(max(va, vb), vc);
    bounds = Box(a, b - a);

  }

  /**
   * Calculates the normal for a particular location on the Triangle.
   *
   * @param inter  the location of the intersection
   * @return       the normal for the location of intersection
   */
  Vector Triangle::normalAt(const Vector& inter) const {
    double u, v;
    Vector diff, nt1, nt2;

    u    = dot((inter - va), _perp) / dot(vc - va, _perp);
    diff = va + ((inter - va) / u);
    v    = ((diff[v_axis]     - vb[v_axis]) /
            (vc[v_axis] - vb[v_axis]));

    nt1 = na + ((nb - na) * u);
    nt2 = na + ((nc - na) * u);

    return (nt1 + ((nt2 - nt1) * v)).normalize();
  }

  /**
   * Get the bounding region for the Triangle.
   *
   * @return  the Bounding region as a Box
   */
  Box Triangle::getBounds() const {
    return bounds;
  }

  /**
   * Calculates the Intersection of a Ray and a Triangle.
   *
   * @param ray    the Ray to find an intersection for.
   * @param skip   the Source surface for the ray.
   * @param inter  the location of the intersection.
   * @return       if the Ray intersected the Surface.
   */
  bool Triangle::getIntersection(const Ray& ray, Intersection& inter) const {
    double a, b, uu, uv, vv, wu, wv, D, gamma, beta, r;
    Vector w, I;
    Vector u, v, n;

    if(this == ray.source())
      return false;

    u = vb - va;
    v = vc - va;
    n = cross(u, v);

    a = -dot(n, ray.L() - va);
    b =  dot(n, ray.U());
    if(fabs(b) < EPSILON) {
      return false;
    }

    if((r = a / b) < 0.0) {
      return false;
    }

    I  = ray.L() + (ray.U() * r);
    uu = dot(u, u);
    uv = dot(u, v);
    vv = dot(v, v);
    w  = I - va;
    wu = dot(w, u);
    wv = dot(w, v);
    D  = uv * uv - uu * vv;

    gamma = (uv * wv - vv * wu) / D;
    if(gamma < 0.0 || gamma > 1.0) {
      return false;
    }

    beta = (uv * wu - uu * wv) / D;
    if(beta < 0.0 || (gamma + beta) > 1.0) {
      return false;
    }

    inter = Intersection(this, I, normalAt(I), ray.U().normalize(), r);
    return true;
  }

  render::d_Surface Triangle::getDevice() const {
    render::d_Surface ret;

    ret.id = id;
    ret.mat = _material;
    ret.min = bounds.min();
    ret.len = bounds.len();

    ret.which = render::d_Surface::triangle;

    ret.va = va; ret.vb = vb; ret.vc = vc;
    ret.na = na; ret.nb = nb; ret.nc = nc;

    ret._norm = _norm;
    ret._perp = _perp;

    ret.d_axis = d_axis;
    ret.v_axis = v_axis;

    return ret;
  }

}
