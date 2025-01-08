/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MATRIX_COMPUTE_H
#define MATRIX_COMPUTE_H

#include "kalman_filter.h"
#include "helper_files.h"

/* User defined types */
/** \ingroup     KF_algo
    \brief       Matrix type definitions, 2D array variant */
typedef struct {
    float32_t matrix[NUMOFROWS][NUMOFELE]; // safer to define rows as well
	uint8_t rows;
	uint8_t cols;
    bool status;
} MatrixT;

/* Only for matrix * vector */
//
Matrix_T mw_matrix_add(const Matrix_T *first_mat, const Matrix_T *second_mat);
//
VectorT mw_matrix_mult(const Matrix_T *mat_inp, const VectorT *vec_inp);

/* For matrix * matrix */
//
MatrixT mw_matrix_mult_full_classic(const MatrixT *mat_inp1, const MatrixT *mat_inp2);
// 
Matrix_T mw_matrix_mult_full(const Matrix_T *mat_inp1, const Matrix_T *mat_inp2);

#endif /* MATRIX_COMPUTE_H */
