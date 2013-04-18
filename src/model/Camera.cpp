/*
 * Camera.cpp
 *
 *  Created on: Jan 3, 2013
 *      Author: norton
 */

/* local includes */
#include <Camera.hpp>
#include <Model.hpp>

namespace ray {

  Camera::Camera() :
      fl(),
      umin(-1), umax(1),
      vmin(-1), vmax(1) { }

  Camera::Camera(Vector fp, Vector vrp, Vector up) :
      fp(fp),
      vrp(vrp),
      n((fp - vrp).normalize()),
      u(cross(up, fp - vrp).normalize()),
      v(cross(n, u)),
      fl(-(vrp - fp).length()),
      umin(-1), umax(1),
      vmin(-1), vmax(1) { }

  /**
   * Changes the location of the camera in the world. This moves the
   *
   * @param amount  the distance to move the Camera
   * @param which   the direction to move the Camera in
   */
  void Camera::move(double amount, axis which) {
    switch(which) {
      case x_axis: fp = fp + (u * amount); break;
      case y_axis: fp = fp + (v * amount); break;
      case z_axis: fp = fp + (n * amount); break;
    }

    vrp = fp + (n * fl);
  }

  /**
   * Rotates the camera around a point and an axis.
   *
   * @param amount  the amount (in radians) to rotate the camera by
   * @param which   the axis to rotate the camera around
   * @param around  the point to rotate the camera around
   */
  void Camera::rotate(double amount, axis which, Vector around) {
    Matrix<double> rota(4, 4);
    Vector vecs[3];

    switch(which) {
      case x_axis: vecs[2] = v; break;
      case y_axis: vecs[2] = u; break;
      case z_axis: return;      break;
    }

    vecs[0] = vecs[2].getPerpendicular();
    vecs[1] = cross(vecs[0], vecs[2]).normalize();

    auto R = ray::eye<double>(4);
    for(int i = 0; i < 3; i++)
      for(int j = 0; j < 3; j++)
        R[i][j] = vecs[i][j];

    /* create z */
    auto z = ray::eye<double>(4);
    z[0][0] =  std::cos(amount);
    z[0][1] =  std::sin(amount);
    z[1][0] = -std::sin(amount);
    z[1][1] =  std::cos(amount);

    /* create rotation matrix */
    rota = R.t() * z * R;

    /* move the actual camera elements */
    fp  = fp - around;
    fp  = rota * fp;
    fp  = fp + around;

    n   = rota * n;
    u   = rota * u;
    v   = rota * v;
    vrp = fp + (n * fl);
  }

  /**
   * Get the Rays that are cast by this Camera. The size of the resulting Matrix
   * should match the size of the picture that is being rendered.
   *
   * @param rows  the number of rows in the resulting Matrix
   * @param cols  the number of columns in the resulting Matrix
   * @return      A Matrix of Rays that will work for the Camera
   */
  Matrix<Ray> Camera::getRays(int rows, int cols) const {
    Vector U, L;
    double xv, yv;
    double xinc = (umax - umin) / double(cols);
    double yinc = (vmax - vmin) / double(rows);

    Matrix<Ray> ret(rows, cols);

    xv = umin;
    for(int x = 0; x < cols; x++, xv += xinc) {
      yv = vmin;
      for(int y = 0; y < rows; y++, yv += yinc) {
        L = vrp + (u * xv) - (v * yv);
        U = (L - fp).normalize();

        ret[y][x] = Ray(L, U, nullptr);
      }
    }

    return ret;
  }

  /**
   * Gets the projection Matrix for the Camera
   *
   * @return  the project Matrix
   */
  Matrix<double> Camera::getProjection() const {
    Matrix<double> ret = ray::eye<double>(4);

    ret[3][2] = 1.0 / fl;
    ret[3][3] = 0;

    return ret;
  }

  /**
   * Gets the rotation Matrix for the Camera
   *
   * @param rows  the number of rows, used to scale the model
   * @param cols  the number of columns, used to scale the model
   * @return      the rotation Matrix for the Camera
   */
  Matrix<double> Camera::getRotation(int rows, int cols) const {
    Matrix<double> scal = ray::eye<double>(4);
    Matrix<double> rota = ray::eye<double>(4);
    Matrix<double> tran = ray::eye<double>(4);

    scal[0][0] = scal[1][1] = scal[2][2] =
        sqrt(pow(rows, 2) + pow(cols, 2));

    rota[0][0] = u[0]; rota[0][1] = u[1]; rota[0][2] = u[2];
    rota[1][0] = v[0]; rota[1][1] = v[1]; rota[1][2] = v[2];
    rota[2][0] = n[0]; rota[2][1] = n[1]; rota[2][2] = n[2];

    tran[0][3] = -fp[0];
    tran[1][3] = -fp[1];
    tran[2][3] = -fp[2];

    return scal * rota * tran;
  }

  /**
   * Create a new Camera based on a model.
   *
   * @param m  the model to base the camera on
   * @return   a new Camera that looks at the model
   */
  Camera Camera::fromModel(const Model& m) {
    auto box   = m.getBounds();
    auto zdiff = sqrt(
        pow(box.len().y(), 2) * pow(box.len().x(), 2)) + box.len().z();

    auto fl  = -1.0;
    auto up  = Vector(0, 1, 0);
    auto at  = Vector(0, 0, 1);
    auto fp  = (box.min() + (box.len() / 2.0)) + Vector(0, 0, 0.25 * zdiff);
    auto vrp = fp + (at * fl);

    return Camera(fp, vrp, up);
  }

}
