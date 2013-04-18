/*
 * render.cu
 *
 *  Created on: Mar 6, 2013
 *      Author: norton
 */

/* local includes */
#include <render.hpp>
#include <Debug.hpp>

/* boost includes */
#include <boost/variant.hpp>

/* std includes */
#include <float.h>
#include <vector>

/* include the vector code */
#include <Vector.cui>
//#include <thrust/functional.h>

#define MAXIMUM_ITERATIONS   512
#define MINIMUM_CONTRIBUTION 0.0039

namespace ray {

  namespace render {

    /* ********************************************************************** */
    /* *** Forward Function declarations ************************************ */
    /* ********************************************************************** */

    template<typename T>
    struct d_Stack {
        __device__ d_Stack() :
            stack(), top(0) { }

        __device__ inline void push(T elem) { stack[top++] = elem;   }
        __device__ inline T&   peek()       { return stack[top - 1]; }
        __device__ inline T     pop()       { return stack[--top];   }
        __device__ inline uint size() const { return top;            }

        T    stack[128];
        uint top;
    };

    struct selem {
        enum state { left = 0, right = 1, done = 2};

        __device__ selem() :
          surf(-1), lr(done) { }
        __device__ selem(int32_t surf) :
          surf(surf), lr(left) { }

        int32_t surf;
        state lr;
    };

    struct {
        uint x;
    } threadIdx;

    __host__ std::ostream& operator<<(std::ostream& ostr, d_Intersection& inter) {
      return (ostr << "INTER[ l:" << inter.location
                        << ", n:" << inter.normal
                        << ", v:" << inter.viewing << "]");
    }

    __host__ std::ostream& operator<<(std::ostream& ostr, d_Ray& ray) {
      return (ostr << "RAY[ L:" << ray.L << ", U:" << ray.U << "]");
    }

    /**
     * Structure used to simplify passing of arguments for the device functions.
     * All of the pointers to relevant arrays are found here instead of passing
     * each individually.
     */
    struct d_Model {
        uint32_t root;
        uint32_t n_lights;

        d_Surface*  surfaces;
        d_Material* materials;
        d_Light*    lights;
    };

    __device__ d_Intersection best_of(
        const d_Intersection& a,
        const d_Intersection& b);

    __device__ bool intersect(
        Vector m,
        Vector l,
        const d_Ray& ray);

    __device__ bool intersect(
        d_Model* surfs,
        uint32_t curr,
        const d_Ray& ray,
        d_Intersection& inter);

    __device__ bool intersect(
        d_Surface& curr,
        const d_Ray& ray,
        d_Intersection& inter);

    __device__ Vector normalAt(
        d_Surface& surf, const
        Vector& inter);

    __device__ Vector diffuse(
        const Vector* lhs,
        const Vector& rhs);

    __device__ bool shadowed(
        d_Model* model,
        const d_Ray& ray,
        const d_Light& light);

    __device__ Vector reflectance(
        d_Model* model,
        const d_Intersection& inter);

    __device__ Vector getColor(
        d_Model* model,
        const d_Ray& ray);

    /* ********************************************************************** */
    /* *** Intersection Code ************************************************ */
    /* ********************************************************************** */

    /**
     * Find the best of two different intersections. This compares the distance
     * of the intersections and picks the closer one.
     *
     * @param a  the first intersection
     * @param b  the second intersection
     * @return   the best of the two intersections
     */
    __device__ d_Intersection best_of(
        const d_Intersection& a,
        const d_Intersection& b)
    {
      return a.distance < b.distance ?
          a.distance > 0 ? a : b :
          b.distance > 0 ? b : a;
    }

    /**
     * Determines if a ray and a bounding box intersect. This is used by every
     * surface before the actual intersection is run. This allows us to short
     * circuit and ignore large sections of the model on every test.
     *
     * @param m    the minimum coordinates of the bounding box.
     * @param l    the length of the sides of a bounding box.
     * @param ray  the ray that we are performing the intersection for
     * @return     if the ray passes through the region represented by the box
     */
    __device__ bool intersect(
        Vector m,
        Vector l,
        const d_Ray& ray)
    {
      double tmin, tmax;
      double dmin, dmax;

      if(ray.zero(0)) {
        if(ray.posi(0)) {
          dmin = (m.x() - ray.L.x()) * ray.iU.x();
          dmax = dmin + (l.x() * ray.iU.x());
          if(dmax < EPSILON)
            return false;
        } else {
          dmax = (m.x() - ray.L.x()) * ray.iU.x();
          if(dmax < EPSILON)
            return false;
          dmin = dmax + (l.x() * ray.iU.x());
        }

        if(dmin > dmax)
          return false;
      } else {
        if((ray.L.x() < m.x()) || (ray.L.x() > l.x() + m.x())) {
          return false;
        }

        dmin = FLT_MIN;
        dmax = FLT_MAX;
      }

      if(ray.zero(1)) {
        if(ray.posi(1)) {
          tmin = (m.y() - ray.L.y()) * ray.iU.y();
          tmax = tmin + (l.y() * ray.iU.y());
        } else {
          tmax = (m.y() - ray.L.y()) * ray.iU.y();
          tmin = tmax + (l.y() * ray.iU.y());
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
        if((ray.L.y() < m.y()) || (ray.L.y() > l.y() + m.y())) {
          return false;
        }
      }

      if(ray.zero(2)) {
        if(ray.posi(2)) {
          tmin = (m.z() - ray.L.z()) * ray.iU.z();
          tmax = tmin + (l.z() * ray.iU.z());
        } else {
          tmax = (m.z() - ray.L.z()) * ray.iU.z();
          tmin = tmax + (l.z() * ray.iU.z());
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
        if((ray.L.z() < m.z()) || (ray.L.z() > l.z() + m.z())) {
          return false;
        }
      }

      return true;
    }

    /**
     * Intersects a ray with a generic surface. This will check if the ray
     * passes through the region that contains the surface and then calls the
     * appropriate function to determine if it intersects the actual surface.
     *
     * @param surfs  The array of all surfaces in the model
     * @param curr   The index of the current surface.
     * @param ray    The ray to find the intersection for.
     * @param inter  Return for the location of intersection
     * @return       If the ray intersected the surface
     */
    __device__ bool intersect(
        d_Model* model,
        uint32_t root,
        const d_Ray& ray,
        d_Intersection& inter)
    {
      d_Intersection best;
      d_Intersection curr;

      d_Stack<selem> stack;

      bool found = false;

      stack.push(selem(root));

      while(stack.size() != 0) {
        if(stack.peek().surf >= 0) {
          d_Surface& surf = model->surfaces[stack.peek().surf];

          if(!intersect(surf.min, surf.len, ray) ||
              ray.src == surf.id) {
            stack.pop();
          } else if(surf.which == d_Surface::triangle) {
            if(intersect(surf, ray, curr)) {
              best = best_of(best, curr);
              found = true;
            }
            stack.pop();

          } else if(surf.which == d_Surface::tree) {
            switch(stack.peek().lr) {
              case selem::left:
                stack.peek().lr = selem::right;
                stack.push(selem(surf.d_axis));
                break;
              case selem::right:
                stack.peek().lr = selem::done;
                stack.push(selem(surf.v_axis));
                break;
              case selem::done:
                stack.pop();
                break;
            }
          }
        } else {
          stack.pop();
        }
      }

      if(found) {
        inter = best;
        return true;
      }
      return false;
    }

    /**
     * Intersections a ray with a Triangle. This is the base case for the
     * recursive intersection check.
     *
     * @param curr   The triangle that will be checked for intersection
     * @param ray    The ray that will be intersected
     * @param inter  Return for the location of intersection
     * @return       If the ray intersected the surface
     */
    __device__ bool intersect(
        d_Surface& curr,
        const d_Ray& ray,
        d_Intersection& inter)
    {
      double  a, b, uu, uv, vv, wu, wv, D, gamma, beta, r;
      Vector w, I;
      Vector u, v, n;

      u = curr.vb - curr.va;
      v = curr.vc - curr.va;
      n = cross(u, v);

      a = -dot(n, ray.L - curr.va);
      b =  dot(n, ray.U);
      if(fabs(b) < EPSILON) {
        return false;
      }

      if((r = a / b) < 0.0) {
        return false;
      }

      I  = ray.L + (ray.U * r);
      uu = dot(u, u);
      uv = dot(u, v);
      vv = dot(v, v);
      w  = I - curr.va;
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

      inter = d_Intersection(curr.id, I, normalAt(curr, I), ray.U.normalize(), r);
      return true;
    }

    /**
     * Finds the normal for the location intersection of a ray and a Triangle.
     *
     * @param surf   The triangle that the ray intersected
     * @param inter  The location of the intersection
     * @return       The normal for the location of intersection
     */
    __device__ Vector normalAt(
        d_Surface& surf,
        const Vector& inter)
    {
      double  u, v;
      Vector diff, nt1, nt2;

      u    = dot((inter - surf.va), surf._perp) / dot(surf.vc - surf.va, surf._perp);
      diff = surf.va + ((inter - surf.va) / u);
      v    = ((diff[surf.v_axis] - surf.vb[surf.v_axis]) /
              (surf.vc[surf.v_axis] - surf.vb[surf.v_axis]));

      nt1 = surf.na + ((surf.nb - surf.na) * u);
      nt2 = surf.na + ((surf.nc - surf.na) * u);

      return (nt1 + ((nt2 - nt1) * v)).normalize();
    }

    /* ********************************************************************** */
    /* *** Reflectance Code ************************************************* */
    /* ********************************************************************** */

    /**
     * Operator used for the calculation of the diffuse reflectance off of a
     * material.
     *
     * @param lhs  the matrix that contains the diffuse reflectance
     * @param rhs  the color of the light that is being reflected
     * @return     the color that reflects off the surface
     */
    __device__ Vector diffuse(
        const Vector* lhs,
        const Vector& rhs)
    {
      return Vector(
          lhs[0][0] * rhs[0] + lhs[0][1] * rhs[0] + lhs[0][2] * rhs[0],
          lhs[1][0] * rhs[1] + lhs[1][1] * rhs[1] + lhs[1][2] * rhs[1],
          lhs[2][0] * rhs[2] + lhs[2][1] * rhs[2] + lhs[2][2] * rhs[2]);
    }

    /**
     * Test if a particular location is shadowed.
     *
     * @param model  the Model that needs to be checks for shadows
     * @param ray    the ray from the location to the light
     * @param light  the light source that is being checked
     * @return       true if the location is shadowed.
     */
    __device__ bool shadowed(
        d_Model* model,
        const d_Ray& ray,
        const d_Light& light)
    {
      d_Intersection inter;

      double maxDistance = light.local.distance(ray.L);
      return(
          intersect(model, model->root, ray, inter) &&
          inter.distance < maxDistance);
    }

    /**
     * Get color of the light at an intersection.
     *
     * @param model  the model to get the color for
     * @param inter  the location to get the color for
     * @return       the color at that location
     */
    __device__ Vector reflectance(
        d_Model* model,
        const d_Intersection& inter)
    {
      d_Material& m = model->materials[model->surfaces[inter.src].mat];

      Vector p = inter.location;
      Vector v = inter.viewing.negate();
      Vector n = inter.normal;

      Vector Lp;
      Vector Rl;

      Vector ret;

      if(dot(v, n) < 0)
        n = n.negate();

      for(int i = 0; i < model->n_lights; i++) {
        const d_Light& light = model->lights[i];

        Lp = (light.local - p).normalize();

        if(dot(Lp, n) < 0 && shadowed(model, d_Ray(Lp, p, inter.src), light))
          continue;

        Rl = (n * (dot(Lp, n) * 2) - Lp).normalize();

        ret = ret +
            (diffuse(m.diffuse, light.illum) * dot(Lp, n)) +
            (light.illum * m.ks * pow(max(0.0, dot(v, Rl)), m.alpha));
      }

      return ret;
    }

    /**
     * Given a source Ray and a model, this gets the color that we should use
     * when filling in the pixel.
     *
     * @param model  the model to render
     * @param ray    the source ray from the camera
     * @return       the color that the pixel should be filled with
     */
    __device__ Vector getColor(
        d_Model* model,
        const d_Ray& ray)
    {
      d_Intersection inter;
      Vector         color, newdir;
      Vector         n, v;
      d_Ray          curr_ray = ray;
      double          cont = 1.0;

      for(int i = 0; i < MAXIMUM_ITERATIONS && cont > MINIMUM_CONTRIBUTION; i++) {
        if(!intersect(model, model->root, curr_ray, inter))
          break;

        v = inter.viewing.negate();
        n = inter.normal;

        color = color + (reflectance(model, inter) * cont);
        cont  = cont * (model->materials[model->surfaces[inter.src].mat].ks);

        newdir = n * (dot(v, n) * 2) - v;
        curr_ray = d_Ray(inter.location, newdir.normalize(), inter.src);
      }

      return ray::max(ray::min(color, 255), 0);
    }

    /* ********************************************************************** */
    /* *** External Interface *********************************************** */
    /* ********************************************************************** */

    __global__ void kernel(
        d_Surface*  surfaces,
        d_Material* materials,
        d_Light*    lights,
        uint32_t    root,
        uint32_t    n_lights,
        d_Ray*      rays,
        Vector*     dest)
    {
      d_Model model;

      model.root      = root;
      model.n_lights  = n_lights;
      model.surfaces  = surfaces;
      model.materials = materials;
      model.lights    = lights;

      uint idx = threadIdx.x;

      dest[idx] = getColor(&model, rays[idx]);
    }

    d_Surface*  surfaces  = NULL;
    d_Material* materials = NULL;
    d_Light*    lights    = NULL;

    uint32_t rootSurface;

    int32_t n_surface;
    int32_t n_material;
    int32_t n_light;

    __host__ void setSurfaces(d_Surface* surs, size_t size, uint32_t root) {
      size_t total = size * sizeof(d_Surface);

      if(surfaces)
        cudaFree(surfaces);

      cudaMalloc((void**)&surfaces, total);
      cudaMemcpy(surfaces, surs, total, cudaMemcpyHostToDevice);

      rootSurface = root;
      n_surface = size;
    }

    __host__ void setMaterials(d_Material* mats, size_t size) {
      size_t total = size * sizeof(d_Material);

      if(materials)
        cudaFree(materials);

      cudaMalloc((void**)&materials, total);
      cudaMemcpy(materials, mats, total, cudaMemcpyHostToDevice);

      n_material = size;
    }

    __host__ void setLights(d_Light* ligs, size_t size) {
      size_t total = size * sizeof(d_Light);

      if(lights)
        cudaFree(lights);

      cudaMalloc((void**)&lights, total);
      cudaMemcpy(lights, ligs, total, cudaMemcpyHostToDevice);

      n_light = size;
    }

    __host__ void Trace(Vector* out, d_Ray* in, size_t size) {
      Vector* device_out;
      d_Ray*  device_in;

      cudaMalloc((void**)&device_out, size * sizeof(Vector));
      cudaMalloc((void**)&device_in,  size * sizeof(d_Ray));

      cudaMemcpy(device_in, in, size * sizeof(d_Ray), cudaMemcpyHostToDevice);

#ifdef __CUDACC__
      kernel<<<1, 256>>>(
          surfaces,
          materials,
          lights,
          rootSurface,
          n_light,
          device_in,
          device_out);
#else
      for(int i = 0; i < size; i++) {
        threadIdx.x = i;
        kernel(
            surfaces,
            materials,
            lights,
            rootSurface,
            n_light,
            device_in,
            device_out);
      }
#endif

      cudaMemcpy(out, device_out, size * sizeof(Vector), cudaMemcpyDeviceToHost);

      cudaFree(device_out);
      cudaFree(device_in);
    }

    __host__ std::ostream& operator<<(std::ostream& ostr, const d_Surface& surf) {
      ostr << surf.id << " => ";

      switch(surf.which) {
        case d_Surface::triangle:
          ostr << "Triangle";
          break;

        case d_Surface::tree:
          ostr << "Tree: " << surf.v_axis << " :: " << surf.d_axis;
          break;
      }

      return ostr;
    }

  }

}
