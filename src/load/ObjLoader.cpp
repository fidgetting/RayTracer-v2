/*
 * ObjLoader.cpp
 *
 *  Created on: Jan 9, 2013
 *      Author: norton
 */

/* local includes */
#include <ObjLoader.hpp>
#include <Vector.hpp>
#include <Model.hpp>

/* std includes */
#include <stdio.h>

/* boost includes */
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

extern int yyparse(void);
extern int yylex_destroy(void);
extern FILE* yyin;

extern std::vector<ray::Vector> verts;
extern std::vector<ray::Vector> texts;
extern std::vector<ray::Vector> norms;

std::vector<std::string> objLibs;
ray::obj::ObjLoader*     objDest;

namespace ray {
  namespace obj {

    const std::string ObjLoader::suffix = ".obj";

    ObjLoader::ObjLoader(std::string fileName) {
      fs::path directory = fs::path(fileName).parent_path();

      verts.clear();
      texts.clear();
      norms.clear();

      objDest = this;

      if(!(yyin = fopen(fileName.c_str(), "r")))
        throw std::exception();
      yyparse();
      fclose(yyin);

      for(std::string& str : objLibs) {
        if(!(yyin = fopen((directory / str).c_str(), "r")))
          throw std::exception();
        yyparse();
        fclose(yyin);
      }

      _vertices = verts;
      _textures = texts;
      _normals  = norms;

      uint16_t gen = 0;
      for(auto& curr : _materials)
        curr.second.second = gen++;
      for(Polygon& curr : _polygons)
        curr.matidx = _materials[curr.material].second;

      yyin = NULL;
    }

    ObjLoader::~ObjLoader() {
      yylex_destroy();
    }

    /**
     * Turns a Mat into a Material for use by the render code.
     *
     * @param mat  the Mat to be translated
     * @return     the Material
     */
    Material ObjLoader::toMaterial(const Mat& mat) const {
      Matrix<double> diffuse = ray::eye<double>(4);

      diffuse[0][0] = mat.kd[0];
      diffuse[1][1] = mat.kd[1];
      diffuse[2][2] = mat.kd[2];

      return Material(mat.ks[0], 0, mat.phong, diffuse);
    }

    /**
     * Get the Lights for the object
     *
     * @return  the vector of Lights
     */
    std::vector<Light> ObjLoader::lights() const {
      return _lights;
    }

    /**
     * Get the Materials for the object
     *
     * @return  the vector of Materials
     */
    std::vector<Material> ObjLoader::materials() const {
      std::vector<Material> retval;

      for(auto curr : _materials) {
        retval.push_back(toMaterial(curr.second.first));
      }

      return retval;
    }

    /**
     * Get the polygons for the object
     *
     * @return  the vector of Polygons
     */
    std::vector<ObjectStream::Polygon> ObjLoader::polygons() const {
      return _polygons;
    }

    /**
     * Get the vertices for the object
     *
     * @return  the vector of vertices
     */
    std::vector<Vector> ObjLoader::vertices() const {
      return _vertices;
    }

    /**
     * Get the the vector texture coordinates for the object
     *
     * @return  the vector of texture coordinates
     */
    std::vector<Vector> ObjLoader::textures() const {
      return _textures;
    }

    /**
     * Get the vector of normals for the object
     *
     * @return  the vector of normals
     */
    std::vector<Vector> ObjLoader::normals() const {
      return _normals;
    }

  }

}

