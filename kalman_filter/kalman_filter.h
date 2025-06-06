/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

/* Docstring
 * Namespace is: kf_
 *
 * Created on: 15.06.2023
 * Author: Barna Temesi
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

/* Due to compatibility reasons */
#ifdef CPU_IS_ARM
	#include <arm_math.h>
#else
	typedef float float32_t;
#endif

#include "helper_files.h"

/* User defines */
/* Default system matrices */
#if SYSTEM_SWITCH == 0 
    #define NUMOFELE		        	    (4U)    /**< A matrix, number of columns */
    #define NUMOFCOLS_B                     (1U)    /**< B matrix, number of columns */
    #define NUMOFCOLS_B_PLUS_KF             (4U)    /**< [ B matrix KF gain ] */
    #define NUMOFROWS_U                     (1U)    /**< Number of control signals, U */
    #define NUMOFROWS		    	        (4U)    /**< Number of states */
    #define LEN_OF_MATRIX                   (NUMOFELE * NUMOFROWS)
    #define NUMOFROWS_SENSOR_MEAS	   	    (3U)    /**< Number of signals measured by sensors */
    #define INI_VAL			    	        ((float32_t)0.0F)  /**< General INIT value */
#endif 

/* Transposed system matrices */
#if SYSTEM_SWITCH == 1 
    #define NUMOFELE		        	    (4U)    /**< A matrix, number of columns */
    #define NUMOFCOLS_B                     (1U)    /**< B matrix, number of columns */
    #define NUMOFCOLS_B_PLUS_KF             (4U)    /**< [ B matrix KF gain ] */
    #define NUMOFROWS_U                     (1U)    /**< Number of control signals, U */
    #define NUMOFROWS		    	        (4U)    /**< Number of states */
    #define LEN_OF_MATRIX                   (NUMOFELE * NUMOFROWS)
    #define NUMOFROWS_SENSOR_MEAS	   	    (3U)    /**< Number of signals measured by sensors */
    #define INI_VAL			    	        ((float32_t)0.0F)  /**< General INIT value */
#endif 

/* General 2 state system */
#if SYSTEM_SWITCH == 2

#endif

/** \ingroup     KF_algo
    \brief       General return enum */
typedef enum {
    NOTVALID,
    VALID
} Ret_T;

/** \ingroup     KF_algo
    \brief       Vector type definitions */
typedef struct {
    float32_t vector[NUMOFROWS];
	uint8_t rows;
    uint8_t arr_cap;
    bool status;
} VectorT;

/** \ingroup     KF_algo
    \brief       Matrix type definitions */
typedef struct {
    float32_t matrix[LEN_OF_MATRIX];
	uint8_t rows;
	uint8_t cols;
    bool status;
} Matrix_T;

/** \ingroup     KF_algo
    \brief       KF type definition */
typedef struct {
    VectorT control_signal_inp;             /*<< Control signal `u` */
    VectorT y_meas_inp;						/*<< Measurement signal `y` */
    bool general_status;					/*<< General status indicator */
} Kalman_Filter_T;

/* User function prototypes */
/**
 * \brief Properly initializes the needed vectors / matrices
 *
 * \details Initializes the delayed state vector of x
 *
 * \param[in]	vector_x_ini: Desired array of values to be used in the first time step
 * \param[in]	vector_x_k_1_inp: Destination of desired vector
 *
 * \return  Status
 */
Ret_T init_kf_matrices(const float32_t kf_vector_ini_inp[], VectorT *kf_vector_x_k_1_inp);

/**
 * \brief Steady-state Kalman-filter computation
 *
 * \details Streamlined computation using non-generic functions
 * 
 * \param[in]   vectors_inp: control signal + measured output vector of y - data structure
 * \param[in]   vector_x_k_1_inp: Delayed state vector, used for proper instancing
 *
 * \return  returns y_est (estimated / filtered signals) and status
 */
VectorT kalman_filter_computation(const Kalman_Filter_T *kf_vectors_inp, VectorT *kf_vector_x_k_1_inp);

#ifdef __cplusplus
}
#endif

#endif /* KALMAN_FILTER_H */
