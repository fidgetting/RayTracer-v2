/*
 * Vector.tpp
 *
 *  Created on: Aug 27, 2010
 *      Author: norton
 */

#ifndef VECTOR_TPP_INCLUDE
#define VECTOR_TPP_INCLUDE

#include <iostream>
#include <string>
#include <cstring>
#include <cmath>

#define EPSILON 1.0e-10
#define DBL_MAX std::numeric_limits<double>::max()
#define DBL_MIN std::numeric_limits<double>::min()

#define V_SIZE 4

namespace ray {

  const int X = 0;
  const int Y = 1;
  const int Z = 2;
  const int W = 3;

  class vector {
    public:

      typedef       double*       iterator;
      typedef const double* const_iterator;

      vector(double d = 0);
      vector(double x, double y, double z);
      vector(double* d);

      /* ********** getters and setters ********** */
      inline double& operator[](int i)       { return data[i]; }
      inline double  operator[](int i) const { return data[i]; }
      inline int     size()            const { return V_SIZE;  }
      inline void    clear() { std::memset(data, 0, V_SIZE * sizeof(double)); }
      inline const double* ptr() const { return data; }

      inline       iterator begin()       { return data;          }
      inline       iterator end()         { return data + V_SIZE; }
      inline const_iterator begin() const { return data;          }
      inline const_iterator end()   const { return data + V_SIZE; }

      /* ********** Math Operators ********** */
      vector cross(const vector& rhs) const;
      vector normal() const;
      double distance(const vector& rhs) const;
      double dot(const vector& rhs) const;
      double length() const;
      void negate();
      void normalize(bool to_w = false);
      vector& operator+=(const vector& v);
      vector& operator+=(const double& d);
      vector& operator/=(const double& d);

      vector  operator*(const vector& rhs) const;
      vector  operator*(double d)          const;
      vector  operator/(double d)          const;

      std::string str() const;

    protected:
      double  data[V_SIZE];
  };
}

bool operator==(const ray::vector& lhs, const ray::vector& rhs);
bool operator!=(const ray::vector& lhs, const ray::vector& rhs);

ray::vector operator-(const ray::vector& lhs, const ray::vector& rhs);
ray::vector operator+(const ray::vector& lhs, const ray::vector& rhs);

#endif /* VECTOR_TPP_INCLUDE */
