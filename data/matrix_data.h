/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MATRIX_DATA_H
#define MATRIX_DATA_H

/* User includes */
#include "kalman_filter.h"

/* User defines */
/* 0 - default system matrices, 
 * 1 - transposed system matrices, 
 * 2 - general 2 state system */
#define SYSTEM_SWITCH                            (1U)

/* Declaration of externs */

/* State-vector definition
 * x = [ omega_spindle, T_mot, T_rider, T_load ]';
 * 
 */

/* System matrix */
extern const Matrix_T d_matrix_A_DU;

/* Input matrix + KF gain, concat */
extern const Matrix_T d_matrix_B_plus_K_DU;

/* Output matrix */
extern const Matrix_T d_matrix_C_DU;

/* Feedforward matrix */
extern const Matrix_T d_matrix_D_DU;

/* Initial condition of the state vector */
extern const float32_t x_ini[NUMOFELE];

/* Initial condition of the state vector delay */
extern const float32_t x_k_1_ini[NUMOFELE];

#endif // MATRIX_DATA_H

