/*
 * Model.hpp
 *
 *  Created on: Jan 8, 2013
 *      Author: norton
 */

#pragma once

/* local includes */
#include <Camera.hpp>
#include <Matrix.tpp>
#include <Surface.hpp>
#include <Vector.hpp>

#include <render.hpp>

/* std includes */
#include <iostream>
#include <map>
#include <vector>

namespace ray {

#define MAXIMUM_ITERATIONS   512
#define MINIMUM_CONTRIBUTION 0.0039

  class ObjectStream;

  class Material {
    public:

      Material() : _ks(0), _kt(0), _alpha(0), _diffuse(4, 4) { }
      Material(double ks, double kt, double alpha, const Matrix<double>& diffuse) :
        _ks(ks),
        _kt(kt),
        _alpha(alpha),
        _diffuse(diffuse) { }
      virtual ~Material() { }

      inline double    ks() const { return _ks;    }
      inline double    kt() const { return _kt;    }
      inline double alpha() const { return _alpha; }

      inline const Matrix<double> diffuse() const { return _diffuse; }

      operator render::d_Material() const;

    private:

      double         _ks;
      double         _kt;
      double         _alpha;
      Matrix<double> _diffuse;
  };

  class Light {
    public:

      Light(Vector local, Vector illum) :
        _local(local),
        _illum(illum)  { }
      virtual ~Light() { }

      inline Vector local() const { return _local; }
      inline Vector illum() const { return _illum; }

      inline operator render::d_Light() const
      { return render::d_Light(_local, _illum); }

    private:

      Vector _local;
      Vector _illum;
  };

  class Model {
    public:

      Model() :
        lights(),
        materials(),
        surfaces(),
        vertices(),
        normals() { }

      Model(std::vector<Light> lights,
            std::vector<Material> materials,
            std::vector<Surface::ptr> surfs,
            ray::Matrix<double>& vertices,
            ray::Matrix<double>& normals);

      virtual ~Model()   { }

      Matrix<Pixel> click(const Camera& cam, int row, int cols) const;

      Box getBounds() const;

      static void fromObjectStream(
          const std::shared_ptr<ObjectStream> objstream,
          Model& mreturn, Camera& creturn);

    private:

      Vector calculateColor(const Ray& ray) const;

      Vector reflectance(const Intersection& inter) const;
      bool   shadowed(const Ray& ray, const Light& light) const;

      bool renderSection(
          const Matrix<Ray>& rays,
          Matrix<Pixel>& out,
          uint32_t minRow,
          uint32_t maxRow) const;

      /** all of the lights for the model */
      std::vector<Light> lights;

      /** all of the materials for the model */
      std::vector<Material> materials;

      /** all of the Surfaces in the model */
      Surface::ptr surfaces;

      /** the vertices for the model */
      ray::Matrix<double> vertices;

      /** the normals for the model */
      ray::Matrix<double> normals;

  };

}




