/*
 * ObjectStream.hpp
 *
 *  Created on: Jan 9, 2013
 *      Author: norton
 */

#pragma once

/* std includes */
#include <memory>
#include <stdint.h>
#include <string>
#include <vector>

namespace ray {

  class Light;
  class Material;
  class Vector;
  class Surface;

  class ObjectStream {
    public:

      struct Polygon {
        Polygon(std::vector<int> verts,
            std::vector<int> texts,
            std::vector<int> norms,
            std::string mat) :
              vertices(verts),
              textures(texts),
              normals (norms),
              material(mat),
              matidx  (0) { }

        std::vector<int> vertices;
        std::vector<int> textures;
        std::vector<int> normals;
        std::string material;
        uint16_t    matidx;
      };

      typedef std::shared_ptr<ObjectStream> ptr;

      ObjectStream() { }
      virtual ~ObjectStream() { }

      virtual std::vector<Light>       lights() const = 0;
      virtual std::vector<Material> materials() const = 0;
      virtual std::vector<Polygon>   polygons() const = 0;
      virtual std::vector<Vector>    vertices() const = 0;
      virtual std::vector<Vector>    textures() const = 0;
      virtual std::vector<Vector>     normals() const = 0;

      static ObjectStream::ptr loadObject(std::string fname);
  };

}
