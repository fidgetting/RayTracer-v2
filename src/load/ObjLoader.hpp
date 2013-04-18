/*
 * ObjLoader.hpp
 *
 *  Created on: Jan 9, 2013
 *      Author: norton
 */

#pragma once

/* local includes */
#include <Model.hpp>
#include <ObjectStream.hpp>

/* std includes */
#include <map>
#include <string>

namespace ray {
  namespace obj {

    class ObjLoader : public ObjectStream {
      public:

        static const std::string suffix;

        struct Mat {
          Mat() : ka(), kd(), ks(), phong(0), illum(0) { }

          Vector  ka;
          Vector  kd;
          Vector  ks;
          double   phong;
          uint8_t illum;
        };

        ObjLoader(std::string fileName);

        ObjLoader(const ObjLoader& obj) = delete;
        const ObjLoader& operator =(const ObjLoader& obj) = delete;

        virtual ~ObjLoader();

        virtual std::vector<Light>       lights() const;
        virtual std::vector<Material> materials() const;
        virtual std::vector<Polygon>   polygons() const;
        virtual std::vector<Vector>    vertices() const;
        virtual std::vector<Vector>    textures() const;
        virtual std::vector<Vector>     normals() const;

        inline void pushLight   (const Light&    l)
          { _lights.push_back(l); }
        inline void pushPolygon (const Polygon&  p)
          { _polygons.push_back(p); }
        inline void makeMaterial(const std::string& name)
          { _materials[name] = std::pair<Mat, uint16_t>(Mat(), 0); }
        inline Mat& getMaterial(const std::string& name)
          { return _materials[name].first; }

      private:

        Material
            toMaterial(
                const Mat& mat) const;

        std::vector<Vector>  _vertices;
        std::vector<Vector>  _textures;
        std::vector<Vector>  _normals;
        std::vector<Light>   _lights;
        std::vector<Polygon> _polygons;
        std::map<std::string, std::pair<Mat, uint16_t> >
            _materials;

    };

  }

}


