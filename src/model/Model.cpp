/*
 * Model.cpp
 *
 *  Created on: Jan 8, 2013
 *      Author: norton
 */

/* local includes */
#include <ObjectStream.hpp>
#include <Model.hpp>
#include <Ray.hpp>
#include <Debug.hpp>

/* boost includes */
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

namespace ray {

#ifdef DEBUG
#define ROW_DEBUG 400
#define COL_DEBUG 512
#endif

  Material::operator ray::render::d_Material() const {
    render::d_Material ret;

    ret.ks    = _ks;
    ret.kt    = _kt;
    ret.alpha = _alpha;

    ret.diffuse[0] = Vector(_diffuse[0][0], _diffuse[0][1], _diffuse[0][2]);
    ret.diffuse[1] = Vector(_diffuse[1][0], _diffuse[1][1], _diffuse[1][2]);
    ret.diffuse[2] = Vector(_diffuse[2][0], _diffuse[2][1], _diffuse[2][2]);

    return ret;
  }

  /**
   * Gets a bounding box for the entire model. This will find a bounding box
   * that contains all of the surfaces contained within the model.
   *
   * @return  The bounding box
   */
  Box Model::getBounds() const {
    return surfaces->getBounds();
  }

  bool Model::renderSection(
      const Matrix<Ray>& rays,
      Matrix<Pixel>& out,
      uint32_t minRow,
      uint32_t maxRow) const
  {
    for(int i = minRow; i < maxRow; i++) {
      for(int j = 0; j < rays.cols(); j++) {
        DEBUG_SECTION(sect, i == ROW_DEBUG && j == COL_DEBUG);

        out[i][j] = Pixel(calculateColor(rays[i][j]));
      }
    }

#ifdef DEBUG

    out[ROW_DEBUG + 1][COL_DEBUG + 1] = Pixel(255, 255, 255);
    out[ROW_DEBUG + 1][COL_DEBUG    ] = Pixel(255, 255, 255);
    out[ROW_DEBUG + 1][COL_DEBUG - 1] = Pixel(255, 255, 255);
    out[ROW_DEBUG    ][COL_DEBUG + 1] = Pixel(255, 255, 255);
    out[ROW_DEBUG    ][COL_DEBUG - 1] = Pixel(255, 255, 255);
    out[ROW_DEBUG - 1][COL_DEBUG + 1] = Pixel(255, 255, 255);
    out[ROW_DEBUG - 1][COL_DEBUG    ] = Pixel(255, 255, 255);
    out[ROW_DEBUG - 1][COL_DEBUG - 1] = Pixel(255, 255, 255);

#endif

    return true;
  }

  Model::Model(
      std::vector<Light> lights,
      std::vector<Material> materials,
      std::vector<Surface::ptr> surfs,
      ray::Matrix<double>& vertices,
      ray::Matrix<double>& normals) :
        lights(lights),
        materials(materials),
        surfaces(nullptr),
        vertices(vertices),
        normals(normals)
  {
    surfaces = std::make_shared<ray::SurfaceTree>(
        surfs.begin(), surfs.end());

    std::vector<render::d_Surface>  s_transfer;
    std::vector<render::d_Material> m_transfer;
    std::vector<render::d_Light>    l_transfer;

    s_transfer.resize(Surface::idgen);
    surfaces->place(s_transfer);

    for(const Material& mat : materials)
      m_transfer.push_back(render::d_Material(mat));
    for(const Light& light: lights)
      l_transfer.push_back(render::d_Light(light));

    setSurfaces (s_transfer.data(), s_transfer.size(), surfaces->id);
    setMaterials(m_transfer.data(), m_transfer.size());
    setLights   (l_transfer.data(), l_transfer.size());
  }

  /**
   * Takes a picture of the model with a Camera. This is the ray tracer's
   * rendering step.
   *
   * @param cam   The Camera to use for the picture
   * @param rows  The number of rows in the image
   * @param cols  The number of columns in the image
   * @return      The resulting image.
   */
  Matrix<Pixel> Model::click(const Camera& cam, int rows, int cols) const {
    Matrix<Ray>   rays = cam.getRays(rows, cols);
    Matrix<Pixel> image(rays.rows(), rays.cols());

    uint8_t  nthreads = boost::thread::hardware_concurrency() + 1;
    uint32_t rowRange = rows / nthreads;

    boost::thread_group threads;

    for(int i = 0; i < nthreads - 1; i++) {
      uint32_t rowStart = i * rowRange;
      threads.create_thread(
          boost::bind(&Model::renderSection, this, rays, image,
              rowStart, rowStart + rowRange));
    }

    renderSection(rays, image, rowRange * (nthreads - 1), rows);

    threads.join_all();

    return image;
  }

  /**
   * Calculate the color that a particular ray will have. This will do the
   * recursive step for the Ray.
   *
   * @param ray  the Ray to check against the Model
   * @return     the Color that the ray is reflecting
   */
  Vector Model::calculateColor(const Ray& ray) const {
    Intersection   best;
    Vector         color(0, 0, 0), newdir;
    Vector         n, v;
    Ray            curr_ray = ray;
    double          cont     = 1.0;

    for(int i = 0; i < MAXIMUM_ITERATIONS && cont > MINIMUM_CONTRIBUTION; i++) {

      /* get the closest intersection */
      if(!surfaces->intersect(curr_ray, best))
        break;

      v = best.v().negate();
      n = best.n();

      /* calculate color of intersection */
      color = color + (reflectance(best) * cont);
      cont  = cont * (materials[best.source()->material()]).ks();

      newdir   = n * (dot(v, n) * 2) - v;
      curr_ray = Ray(best.i(), newdir.normalize(), best.source());
    }

    return ray::max(ray::min(color, 255), 0);
  }

  /**
   * Calculates the color of the reflection for a particular Intersection. The
   * Intersection has the location of the intersection, the reflecting surface,
   * and the normal and viewing Vectors.
   *
   * @param inter  the location of the Intersection
   * @return       the color of the reflection off the surface
   */
  Vector Model::reflectance(const Intersection& inter) const {
    Material m = materials[inter.source()->material()];
    Vector p = inter.i();
    Vector v = inter.v().negate();
    Vector n = inter.n();

    Vector Lp;
    Vector Rl;

    Vector ret;

    if(dot(v, n) < 0)
      n = n.negate();

    for(const Light& light : lights) {
      Lp = (light.local() - p).normalize();

      if(dot(Lp, n) < 0 && shadowed(Ray(Lp, p, inter.source()), light))
        continue;

      Rl = (n * (dot(Lp, n) * 2) - Lp).normalize();

      ret = ret +
          (m.diffuse() * light.illum() * dot(Lp, n)) +
          (light.illum() * m.ks() * std::pow(std::max(double(0.0), dot(v, Rl)), m.alpha()));
    }

    return ret;
  }

  /**
   * Determines if a particular Light is shadowed by one of the Surfaces in the
   * Model.
   *
   * @param ray    the Ray from the Intersection to the Light
   * @param Light  the Light to check if it is Shadowed
   * @return       true if the Intersection is shadowed for the Light
   */
  bool Model::shadowed(const Ray& ray, const Light& light) const {
    Intersection inter;

    return
        surfaces->intersect(ray, inter) &&
        inter.distance() < light.local().distance(ray.L());
  }

  /**
   * Creates a Model and a Camera based on an ObjectStream
   *
   * @param stream   the object stream to create everything with
   * @param mreturn  return location for the model
   * @param creturn  return location for teh camera
   */
  void Model::fromObjectStream(
      const std::shared_ptr<ObjectStream> stream,
      Model& mreturn, Camera& creturn)
  {
    /* build everything for the model */
    auto vertices = stream->vertices();
    auto normals  = stream->normals();
    auto polygons = stream->polygons();

    auto vertmat  = Matrix<double>(vertices.size(), 4);
    auto normmat  = Matrix<double>(normals.size(),  4);

    std::vector<Surface::ptr> surfaces;

    for(int i = 0; i < vertices.size(); i++) {
      vertmat[i][0] = vertices[i].x();
      vertmat[i][1] = vertices[i].y();
      vertmat[i][2] = vertices[i].z();
      vertmat[i][3] = 1.0;
    }

    for(int i = 0; i < normals.size(); i++) {
      normmat[i][0] = normals[i].x();
      normmat[i][1] = normals[i].y();
      normmat[i][2] = normals[i].z();
      normmat[i][3] = 1.0;
    }

    for(int i = 0; i < polygons.size(); i++) {
      ObjectStream::Polygon& p = polygons[i];

      for(int j = 1; j < p.vertices.size() - 1; j++) {
        surfaces.push_back(std::make_shared<Triangle>(
            RefVector(vertmat, p.vertices[0]),
            RefVector(vertmat, p.vertices[j]),
            RefVector(vertmat, p.vertices[j + 1]),
            RefVector(normmat, p.normals[0]),
            RefVector(normmat, p.normals[j]),
            RefVector(normmat, p.normals[j + 1]),
            p.matidx));
      }
    }

    /* build everything for the camera */
    auto box = Box(surfaces.begin(), surfaces.end());
    auto zdiff = sqrt(pow(box.len().y(), 2) * pow(box.len().x(), 2)) + box.len().z();

    auto fl  = -1.0;
    auto up  = Vector(0, 1, 0);
    auto at  = Vector(0, 0, 1);
    auto fp  = (box.min() + (box.len() / 2.0)) + Vector(0, 0, 0.25 * zdiff);
    auto vrp = fp + (at * fl);

    /* create the camera */
    creturn = Camera(fp, vrp, up);

    /* add a couple lights for the camera */
    auto lights = stream->lights();
    lights.push_back(ray::Light(creturn._fp(), ray::Vector(255)));
    lights.push_back(ray::Light(creturn._fp(), ray::Vector(255)));

    /* create the model */
    mreturn = Model(
        lights,
        stream->materials(),
        surfaces,
        vertmat,
        normmat);
  }
}
