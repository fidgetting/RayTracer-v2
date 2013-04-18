/*
 * Matrix.tpp
 *
 *  Created on: Dec 25, 2012
 *      Author: norton
 */

#pragma once

/* std includes */
#include <iostream>
#include <iomanip>
#include <stdint.h>
#include <memory>

namespace ray {

  template<typename Type>
  class Matrix {
    public:

      Matrix();
      Matrix(int rows, int cols, Type t = Type());

      /* getters */
      inline uint32_t rows() const { return _rows; }
      inline uint32_t cols() const { return _cols; }

      inline       Type* get()       { return data.get(); }
      inline const Type* get() const { return data.get(); }

      inline       Type* operator [](int row)
      { return &(data.get()[row * _cols]); }
      inline const Type* operator [](int row) const
      { return &(data.get()[row * _cols]); }

      /* operations */
      Matrix<Type> t() const;

    private:

      struct deleter { void operator()(Type* t) { delete[] t; } };

      uint32_t _rows;
      uint32_t _cols;

      std::shared_ptr<Type> data;
  };

  template<typename Type>
  Matrix<Type>::Matrix() :
      _rows(0),
      _cols(0),
      data(nullptr) { }

  /**
   * Constructs a new Matrix
   *
   * @param rows  the number of rows in the Matrix
   * @param cols  the number of columns in the Matrix
   * @param t     the starting value of the Matrix
   */
  template<typename Type>
  Matrix<Type>::Matrix(int rows, int cols, Type t) :
      _rows(rows),
      _cols(cols),
      data (new Type[rows * cols], deleter())
  {
    for(int i = 0; i < rows * cols; i++) {
      data.get()[i] = t;
    }
  }

  /**
   * Create the transpose of the Matrix
   *
   * @return  a new Matrix
   */
  template<typename Type>
  Matrix<Type> Matrix<Type>::t() const {
    Matrix<Type> ret(_cols, _rows);

    for(int i = 0; i < _rows; i++) {
      for(int j = 0; j < _cols; j++) {
        ret[j][i] = data.get()[i * _cols + j];
      }
    }

    return ret;
  }

  /**
   * Create a certain sized identity Matrix
   *
   * @return  a new Matrix
   */
  template<typename Type>
  Matrix<Type> eye(int size) {
    Matrix<Type> ret(size, size, 0);

    for(int i = 0; i < size; i++) {
      ret[i][i] = 1;
    }

    return ret;
  }

  /**
   * Multiply two Matrices. This a naive implementation of Matrix multiplication
   * and as a result should be used for small Matrices. Calling this funcation
   * also requires providing the template parameters. As a result of this
   * restriction it is much easier to call this function through the * operator.
   *
   * @param lhs  Matrix on the left side of the multiplication
   * @param rhs  Matrix on the right side of the multiplication
   * @return     a new Matrix that is the result of the multiplication
   */
  template<typename Type>
  Matrix<Type> MatrixMult(const Matrix<Type>& lhs, const Matrix<Type>& rhs) {
    Matrix<Type> ret(lhs.rows(), rhs.cols());

    if(lhs.cols() != rhs.rows()) {
      throw std::exception();
    }

    for(int i = 0; i < lhs.rows(); i++) {
      for(int j = 0;j < rhs.cols(); j++) {
        for(int k = 0; k < lhs.cols(); k++) {
          ret[i][j] += lhs[i][k] * rhs[k][j];
        }
      }
    }

    return ret;
  }

  /**
   * Operator overload for two Matrices. This is mostly an easy to use wrapper
   * around the MatrixMult function.
   *
   * @param lhs  Matrix to the left of the operator
   * @param rhs  Matrix to the right of the operator
   * @return     A new Matrix
   */
  template<typename T1, typename T2>
  Matrix<T1> operator *(const Matrix<T1>& lhs, const Matrix<T2>& rhs) {
    return MatrixMult<T1>(lhs, rhs);
  }

  /**
   * Operator overload for a Matrix and a scalar.
   *
   * @param lhs  The Matrix to the left of the operator
   * @param rhs  the Scalar to the right of the operator
   * @return     A new Matrix
   */
  template<typename Type>
  Matrix<Type> operator *(const Matrix<Type>& lhs, Type rhs) {
    Matrix<Type> ret;

    for(int i = 0; i < lhs.rows(); i++) {
      for(int j = 0; j < lhs.cols(); j++) {
        ret[i][j] = lhs[i][j] * rhs;
      }
    }

    return ret;
  }

  template<typename Type>
  std::ostream& operator<<(std::ostream& ostr, const Matrix<Type>& mat) {
    for(int i = 0; i < mat.rows(); i++) {
      ostr << "[ ";
      for(int j = 0; j < mat.cols(); j++) {
        ostr << std::setprecision(5) << std::setfill(' ') << std::setw(7)
             << mat[i][j] << " ";
      }
      ostr << "]\n";
    }

    return ostr;
  }

}
