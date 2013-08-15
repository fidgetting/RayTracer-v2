/*
 * RefVector.hpp
 *
 *  Created on: Jan 24, 2013
 *      Author: norton
 */

#pragma once

/* local includes */
#include <Matrix.tpp>
#include <Vector.hpp>

namespace ray {

  class RefVector {
    public:

      RefVector(const double* data);
      RefVector(const Matrix<double>& mat, uint16_t idx);

      /* getters */
      inline double x() const { return data[0]; }
      inline double y() const { return data[1]; }
      inline double z() const { return data[2]; }
      inline double w() const { return data[3]; }

      inline double operator[](int i) const { return data[i]; }

      /* operations */
      Vector negate()    const;
      Vector normalize() const;

      double distance(const Vector& rhs) const;
      double length  ()                  const;

    private:

      const double* data;
  };

  Vector operator *(const Matrix<double>& lhs, const Vector& rhs);

}





