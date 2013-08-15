/*
 * Surface.h
 *
 *  Created on: Dec 31, 2012
 *      Author: norton
 */

#pragma once

/* local includes */
#include <RefVector.hpp>
#include <Vector.hpp>

#include <render.hpp>

/* std includes */
#include <algorithm>
#include <memory>
#include <vector>

namespace ray {

  class Ray;
  class Intersection;
  class Model;

  class Box {
    public:

      Box() : _min(), _len() { }
      Box(Vector min, Vector len) : _min(min), _len(len) { }
      Box(const Box& a, const Box& b);

      template<typename iter_t>
      Box(iter_t begin, iter_t end);

      inline const Vector& min() const { return _min; }
      inline const Vector& len() const { return _len; }

      bool contains(const Box& box) const;
      bool intersect(const Ray& ray) const;

    private:

      Vector _min;
      Vector _len;
  };

  class Surface {
    public:

      typedef std::shared_ptr<Surface> ptr;

      Surface(uint16_t material) : id(idgen++), _material(material) { }
      virtual ~Surface() { }

      inline uint16_t material() const { return _material; }

      bool intersect(const Ray& ray, Intersection& inter) const;

      virtual Box  getBounds() const = 0;
      virtual bool getIntersection(const Ray& ray, Intersection& inter) const = 0;

      virtual void place(std::vector<render::d_Surface>& out) const = 0;

      static uint32_t idgen;
      uint32_t id;

      operator render::d_Surface() const;

    protected:

      virtual render::d_Surface getDevice() const = 0;

      uint16_t _material;
  };

  class SurfaceTree : public Surface {
    public:

      template<typename iter_t>
      SurfaceTree(iter_t begin, iter_t end);

      virtual ~SurfaceTree() { }

      virtual Box getBounds() const;
      virtual bool getIntersection(const Ray& ray, Intersection& inter) const;

      virtual void place(std::vector<render::d_Surface>& out) const
      { out[id] = render::d_Surface(*this); for(const Surface::ptr s : children) s->place(out); }

    private:

      virtual render::d_Surface getDevice() const;

      std::vector<Surface::ptr> children;
      Box                       bounds;
  };

  class Triangle : public Surface {
    public:

      Triangle(RefVector va, RefVector vb, RefVector vc,
               RefVector na, RefVector nb, RefVector nc,
               uint16_t material);

      virtual ~Triangle() { }

      virtual Box getBounds() const;
      virtual bool getIntersection(const Ray& ray, Intersection& inter) const;

      inline virtual void place(std::vector<render::d_Surface>& out) const
      { out[id] = render::d_Surface(*this); }

    private:

      virtual render::d_Surface getDevice() const;

      Vector normalAt(const Vector& v) const;

      RefVector va, vb, vc;
      RefVector na, nb, nc;
      Vector    _norm;
      Vector    _perp;
      uint8_t   d_axis;
      uint8_t   v_axis;
      Box       bounds;
  };

  /* ************************************************************************ */
  /* *** template function declarations ************************************* */
  /* ************************************************************************ */

  /**
   * Creates a Bounding Box for a collection of surfaces.
   *
   * @param begin  the beginning iterator of the collection
   * @param end    the ending iterator of the collection
   */
  template<typename iter_t>
  Box::Box(iter_t begin, iter_t end) :
      _min(0, 0, 0), _len(0, 0, 0)
  {
    auto base = (*begin)->getBounds();

    for(auto iter = begin + 1; iter != end; iter++)
      base = Box(base, (*iter)->getBounds());

    _min = base.min();
    _len = base.len();
  }

#define COMP_LAMBDA(axis)                                \
  [](const Surface::ptr& l, const Surface::ptr& r) {     \
    auto lb = l->getBounds(), rb = r->getBounds();       \
    return (lb.min().axis() + (lb.len().axis() / 2.0) <  \
            rb.min().axis() + (rb.len().axis() / 2.0));  \
  }

  /**
   * Constructs the SurfaceTree using a collection of surfaces. Basically this
   * will divide the list up to create sub-trees and then create the current
   * tree with the two sub-trees.
   *
   * @param begin  the beginning iterator of the collection
   * @param end    the ending iterator of the collection
   */
  template<typename iter_t>
  SurfaceTree::SurfaceTree(iter_t begin, iter_t end) :
      Surface(0), children(), bounds(begin, end)
  {
    if((end - begin) < BRANCHING_FACTOR) {
      children = std::vector<Surface::ptr>(begin, end);
    } else {

      auto seperator = begin + ((end - begin) / 2);
      auto compfunc = bounds.len().x() > bounds.len().y() ?
          bounds.len().x() > bounds.len().z() ? COMP_LAMBDA(x) : COMP_LAMBDA(z) :
          bounds.len().y() > bounds.len().z() ? COMP_LAMBDA(y) : COMP_LAMBDA(z);

      std::sort(begin, end, compfunc);

      auto a = std::make_shared<SurfaceTree>(begin, seperator);
      auto b = std::make_shared<SurfaceTree>(seperator, end);

      children.push_back(a);
      children.push_back(b);
    }
  }

#undef COMP_LAMBDA

}
