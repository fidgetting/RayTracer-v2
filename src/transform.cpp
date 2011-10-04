/*
 * Transform.cpp
 *
 *  Created on: Aug 30, 2010
 *      Author: norton
 */

#include <transform.h>

#include <cmath>

/**
 * Used to create the scale transformation matrix
 *
 * @return a new scale matrix
 */
ray::matrix<4, 4> ray::scale::operator()() {
  matrix<4, 4> ret;

  ret[0][0] = _x;
  ret[1][1] = _y;
  ret[2][2] = _z;
  ret[3][3] = 1.0;

  return ret;
}

/**
 * Used to create the translation transformation matrix
 *
 * @return a new translation matrix
 */
ray::matrix<4, 4> ray::translate::operator()() {
  matrix<4, 4> ret = identity<4>();

  ret[0][3] = _x;
  ret[1][3] = _y;
  ret[2][3] = _z;
  ret[3][3] = 1.0;

  return ret;
}

/**
 * Used to create the rotation transformation matrix.
 *
 * @return a new rotation matrix
 */
ray::matrix<4, 4> ray::rotate::operator()() {
  matrix<4, 4> R;
  matrix<4, 4> z;
  vector vecs[3];
  vector m;

  /* create the rotation matrix */
  /* create w */
  vecs[2] = _w;
  vecs[2].normalize();

  /* create u */
  m = _w;
  if(     m[0] <= m[1] && m[0] <= m[2]) m[0] = 1;
  else if(m[1] <= m[2])                 m[1] = 1;
  else                                  m[2] = 1;
  m.normalize();
  vecs[0] = vecs[2].cross(m);
  vecs[0].normalize();

  /* create v */
  vecs[1] = vecs[0].cross(vecs[2]);

  R[3][3] = 1;
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++) {
      R[i][j] = vecs[i][j];
    }
  }

  /* create the Z axis matrix */
  z[0][0] =  std::cos(_r);
  z[0][1] = -std::sin(_r);
  z[1][0] =  std::sin(_r);
  z[1][1] =  std::cos(_r);
  z[2][2] = 1;
  z[3][3] = 1;

  return R.t() * z * R;
}
