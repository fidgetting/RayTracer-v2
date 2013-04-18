/*
 * render.hpp
 *
 *  Created on: Mar 6, 2013
 *      Author: norton
 */

#ifndef RENDER_HPP_
#define RENDER_HPP_

#pragma once

/* local includes */
#include <Vector.hpp>

/* std includes */
#include <stdint.h>

namespace ray {

  namespace render {

    struct d_Material {
        double ks;
        double kt;
        double alpha;
        Vector diffuse[3];
    };

    struct d_Light {
        __host__ d_Light(Vector loc, Vector ill) :
          local(loc), illum(ill) { }

        Vector local;
        Vector illum;
    };

    struct d_Surface {
        enum Type { triangle = 0, tree = 1 };

        Vector min;
        Vector len;
        uint32_t id;
        uint32_t mat;

        Type which;

        int32_t d_axis;
        int32_t v_axis;
        Vector va, vb, vc;
        Vector na, nb, nc;
        Vector _norm;
        Vector _perp;
    };

    struct d_Ray {

        __host__ d_Ray() :
            L(), U(), iL(), iU(), positive(), nonzero(), src() { }

        __device__ __host__ d_Ray(Vector L, Vector U, int32_t src) :
            L(L), U(U),
            iL(1.0 / L.x(), 1.0 / L.y(), 1.0 / L.z()),
            iU(1.0 / U.x(), 1.0 / U.y(), 1.0 / U.z()),
            positive(), nonzero(), src(src) {
          positive[0] = U.x() > 0;
          positive[1] = U.y() > 0;
          positive[2] = U.z() > 0;
          nonzero [0] = L.x() != 0;
          nonzero [1] = L.y() != 0;
          nonzero [2] = L.z() != 0;
        }

        __device__ __host__ d_Ray(Vector L, Vector U, Vector iL, Vector iU, bool posi[3],
            bool nonz[3], int32_t src) :
                      L(L), U(U), iL(iL), iU(iU), positive(), nonzero(),
                      src(src) {
          positive[0] = posi[0];
          positive[1] = posi[1];
          positive[2] = posi[2];
          nonzero [0] = nonz[0];
          nonzero [1] = nonz[1];
          nonzero [2] = nonz[2];
        }

        __device__ inline bool posi(uint8_t idx) const { return positive[idx]; }
        __device__ inline bool zero(uint8_t idx) const { return nonzero [idx]; }

        Vector L, U, iL, iU;

        bool positive[3];
        bool nonzero [3];

        int32_t src;
    };

    struct d_Intersection {

        __device__ __host__ d_Intersection() :
                  src(-1), location(), normal(), viewing(), distance(-1) { }
        __device__ __host__ d_Intersection(int32_t src, Vector location, Vector normal,
            Vector viewing, double distance) :
                      src(src), location(location), normal(normal), viewing(viewing),
                      distance(distance) { }

        int32_t src;

        Vector location;
        Vector normal;
        Vector viewing;

        double distance;
    };

    __host__ void setSurfaces (d_Surface*  surs, size_t size, uint32_t root);
    __host__ void setMaterials(d_Material* mats, size_t size);
    __host__ void setLights   (d_Light*    ligs, size_t size);

    __host__ void Trace(Vector* out, d_Ray* in, size_t size);

    __host__ std::ostream& operator<<(std::ostream& ostr, const d_Surface& surf);
  }

}

#endif /* RENDER_HPP_ */
