/*
 * RefVector.cpp
 *
 *  Created on: Jan 24, 2013
 *      Author: norton
 */

/* local includes */
#include <RefVector.hpp>

/* std includes */
#include <cmath>

namespace ray {

  /**
   * Create the Reference Vector based on
   *
   * @param d  the pointer to set the data to
   */
  RefVector::RefVector(const double* d) :
    data(d) { }

  /**
   * Create a Reference Vector based on the row of a matrix.
   *
   * @param mat
   * @param idx
   */
  RefVector::RefVector(const Matrix<double>& mat, uint16_t idx) :
    data(mat[idx]) { }

  /**
   * Calculate the Vector in the opposite direction.
   *
   * @return  a new Vector in exactly the opposite direction
   */
  Vector RefVector::negate() const {
    return Vector(-x(), -y(), -z());
  }

  /**
   * Calculate a new Vector going the same direction with length of 1.
   *
   * @return  the new Vector of length 1
   */
  Vector RefVector::normalize() const {
    double len = length();
    return Vector(
        x() / len,
        y() / len,
        z() / len);
  }

  /**
   * Calculate the distance between a RefVector and a Vector
   *
   * @param rhs  The other Vector
   * @return     The distance between the two Vectors
   */
  double RefVector::distance(const Vector& rhs) const {
    return ((*this) - rhs).length();
  }

  /**
   * Calculate the length of the RefVector.
   *
   * @return  The double length of the Vector
   */
  double RefVector::length() const {
    return std::sqrt(
        x() * x() +
        y() * y() +
        z() * z());
  }

  /**
   * Construct a Vector using a reference Vector. This will be used to do
   * implicit casts from RefVector to Vector for most of the Vector operations.
   *
   * @param vec  the RefVector to copy
   */
  Vector::Vector(const RefVector& vec) :
    data({vec.x(), vec.y(), vec.z(), 0.0}) { }

  /**
   * Muliply a Matrix by a column Vector.
   *
   * @param lhs  the Matrix to multiply
   * @param rhs  the Column Vector
   * @return     a new Vector that is the multiplication
   */
  Vector operator *(const Matrix<double>& lhs, const Vector& rhs) {
    double ret[4] = {0.0, 0.0, 0.0, 0.0};

    for(int i = 0; i < lhs.rows(); i++) {
      for(int j = 0; j < lhs.cols(); j++) {
        ret[i] += lhs[i][j] * rhs[j];
      }
    }

    return Vector(ret[0], ret[1], ret[2]);
  }

}

