/*
 * vector.cpp
 *
 *  Created on: Sep 15, 2011
 *      Author: norton
 */

#include <vector.h>

ray::vector::vector(double d) : data(NULL), del(true) {
  data = new double[V_SIZE];
  memset(data, 0, V_SIZE * sizeof(double));
  data[0] = d;
  data[1] = d;
  data[2] = d;
  data[3] = 1.0;
}

ray::vector::vector(double x, double y, double z) : data(NULL), del(true) {
  data = new double[V_SIZE];
  data[0] = x;
  data[1] = y;
  data[2] = z;
  data[3] = 1.0;
}

ray::vector::vector(const vector& cpy) : data(cpy.data), del(cpy.del) {
  if(cpy.del) {
    data = new double[V_SIZE];
    del = cpy.del;
    memcpy(data, cpy.data, V_SIZE * sizeof(double));
  }
}

ray::vector::~vector() {
  if(del) delete[] data;
}

/**
 * assignment operator for the vector class
 *
 * @param asn
 * @return
 */
const ray::vector& ray::vector::operator=(const vector& asn) {
  if(del) delete[] data;

  del  = asn.del;
  data = asn.data;
  if(del) {
    data = new double[V_SIZE];
    memcpy(data, asn.data, V_SIZE * sizeof(double));
  }

  return asn;
}

/**
 * Calculates the cross product of this vector and another
 *
 * @param rhs the vector to compare to
 * @return a new vector perpendicular to both
 */
ray::vector ray::vector::cross(const vector& rhs) const {
  vector ret;

  ret[0] = data[1]*rhs[2] - data[2]*rhs[1];
  ret[1] = data[2]*rhs[0] - data[0]*rhs[2];
  ret[2] = data[0]*rhs[1] - data[1]*rhs[0];

  return ret;
}

/**
 * Returns a vector that is perpendicular to the current vector. The returned
 * vector will be of length 1.
 *
 * @return a vector normal to the current one
 */
ray::vector ray::vector::normal() const {
  ray::vector ret = *this;

  ret[0] += 10;
  ret = this->cross(ret);
  ret.normalize();

  return ret;
}

/**
 * Calculates the distance between two vector, treating them as points
 *
 * @param rhs the vector to calculate the distance to
 * @return the distance
 */
double ray::vector::distance(const vector& rhs) const {
  double ret = 0;

  for(int i = 0; i < V_SIZE - 1; i++) {
    ret += std::pow(data[i] - rhs[i], 2);
  }

  return std::sqrt(ret);
}

/**
 * Calculates the dot product of two vectors
 *
 * @param rhs
 * @return
 */
double ray::vector::dot(const vector& rhs) const {
  double ret = 0;

  for(int i = 0; i < V_SIZE - 1; i++) {
    ret += data[i] * rhs[i];
  }

  return ret;
}

/**
 * Calculates the length of the vector
 *
 * @return the length
 */
double ray::vector::length() const {
  double ret = 0;

  for(int i = 0; i < V_SIZE - 1; i++) {
    ret += std::pow(data[i], 2);
  }

  return std::sqrt(ret);
}

/**
 * reverse the direction of the Vector
 */
void ray::vector::negate() {
  for(int i = 0; i < V_SIZE - 1; i++) {
    data[i] = -data[i];
  }
}

/**
 * make the length of the vector 1
 */
void ray::vector::normalize(bool to_w) {
  if(to_w) {
    for(int i = 0; i < V_SIZE; i++) {
      data[i] /= data[V_SIZE - 1];
    }
  } else {
    double mag = length();
    for(int i = 0; i < V_SIZE - 1; i++) {
      data[i] /= mag;
    }
  }
}

/**
 * Adds another vector to this one
 *
 * @param rhs the other vector
 * @return this
 */
ray::vector& ray::vector::operator+=(const vector& rhs) {
  for(int i = 0; i < V_SIZE - 1; i++) {
    data[i] += rhs[i];
  }
  return *this;
}

/**
 * Adds a scalr to a vector
 *
 * @param rhs the scalar
 * @return this
 */
ray::vector& ray::vector::operator+=(const double& rhs) {
  for(int i = 0; i < V_SIZE - 1; i++) {
    data[i] += rhs;
  }
  return *this;
}

ray::vector& ray::vector::operator/=(const double& rhs) {
  for(int i = 0; i < V_SIZE - 1; i++) {
    data[i] /= rhs;
  }
  return *this;
}

/**
 * multiplies two vectors
 *
 * @param lhs
 * @param rhs
 * @return
 */
ray::vector ray::vector::operator*(const ray::vector& rhs) const {
  ray::vector ret;

  for(int i = 0; i < V_SIZE - 1; i++) {
    ret[i] = data[i] * rhs[i];
  }

  return ret;
}

/**
 * multiplies a vector by a scalar
 *
 * @param lhs
 * @param rhs
 * @return
 */
ray::vector ray::vector::operator*(double rhs) const {
  ray::vector ret;

  for(int i = 0; i < V_SIZE - 1; i++) {
    ret[i] = data[i] * rhs;
  }

  return ret;
}

/**
 * checks if two vectors are equal
 *
 * @param lhs left hand side of operator
 * @param rhs right hand side of operator
 * @return equality
 */
bool operator==(const ray::vector& lhs, const ray::vector& rhs) {
  for(int i = 0; i < V_SIZE - 1; i++)
    if(lhs[i] != rhs[i])
      return false;
  return true;
}

/**
 * checks if two vectors are not equal
 *
 * @param lhs the left hand side fo the operator
 * @param rhs the right hand side of the operator
 * @return non-equality
 */
bool operator!=(const ray::vector& lhs, const ray::vector& rhs) {
  return !(lhs == rhs);
}

/**
 * subtracts one vector from another
 *
 * @param lhs left hand side of operator
 * @param rhs right hand side of operator
 * @return a new vector
 */
ray::vector operator-(const ray::vector& lhs, const ray::vector& rhs) {
  ray::vector ret;

  for(int i = 0; i < V_SIZE - 1; i++) {
    ret[i] = lhs[i] - rhs[i];
  }

  return ret;
}

/**
 * add two vectors
 *
 * @param lhs
 * @param rhs
 * @return
 */
ray::vector operator+(const ray::vector& lhs, const ray::vector& rhs) {
  ray::vector ret;

  for(int i = 0; i < V_SIZE - 1; i++) {
    ret[i] = lhs[i] + rhs[i];
  }

  return ret;
}

/**
 * output operator for the vector class
 *
 * @param ostr
 * @param v
 * @return
 */
std::ostream& operator<<(std::ostream& ostr, const ray::vector& v) {
  ostr << "v ";
  for(int i = 0; i < V_SIZE; i++)
    ostr << v[i] << " ";
  return ostr;
}





