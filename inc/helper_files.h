#ifndef HELPER_F_H__
#define HELPER_F_H__

#include "main.h"
#include <stdint.h>
#include <math.h>

/* Type definitions        */
/** \ingroup     SpGtor
    \brief       Struct of filtered signals,
 * 				 source: Filtering_component */
typedef struct
{
	float32_t signal_out_k;							/**< Output signal */
	float32_t drl_upper_limit;						/**< DRL upper limit value */
	float32_t drl_lower_limit;						/**< DRL lower limit value */
	bool status_active;								/**< Active status */
} RC_Dynamic_RateLimT;

/* User defined prototypes */
//
void mw_print_array(const float32_t *arr_inp, const uint32_t len);
//
void mw_print_matrix(const float32_t *matrix_inp, const uint32_t rows, const uint32_t cols);

/* Obsolete for now
void print_array_2d(const float32_t (*arr_inp)[], const uint32_t rows);
 * */

/**
 * \brief Array initialization
 *
 * \details Generic array init with given value
 * 
 * \param[out]	ptr_inp: float32_t pointer to array
 * \param[in]   ini_val: Desired initial value
 * \param[in]   num_of_ele: Number of elements in the array
 *
 * \return  void
 */
void mw_init_array(float32_t *ptr_inp, float32_t ini_val, uint32_t num_of_ele);
//
void mw_init_matrix_by_rows(float32_t *matrix_inp, float32_t *each_row_inp, uint32_t rows, uint32_t cols);

/**
 * \brief Saturation for float32_t type
 *
 * \details X
 *
 * \return Return value of float32_t
*/
float32_t mw_sat_float(const float32_t inp_val, const float32_t upper_limit, const float32_t lower_limit);

/**
 * \brief Saturation for uint32_t type
 *
 * \details X
 *
 * \return Return value of uint32_t
*/
uint32_t mw_sat_uint(const uint32_t inp_val, const uint32_t upper_limit, const uint32_t lower_limit);

/**
 * \brief Saturation for uint8_t type
 *
 * \details X
 *
 * \return Return value of uint8_t
*/
uint8_t mw_sat_uint8(const uint8_t inp_val, const uint8_t upper_limit, const uint8_t lower_limit);

/**
 * \brief Comparison of two floating point signals
 *
 * \details X
 *
 * \return Return value of bool
*/
bool mw_float_comparison(const float32_t inp_val1, const float32_t inp_val2, const float32_t eps_in);

/**
 * \brief Zero guard used for float32_t type
 *
 * \details X
 *
 * \return Return value of float32_t
*/
float32_t mw_zero_guard(const float32_t inp_val, const float32_t desired_val);

/**
 * \brief Initialization of dynamic rate limiter
 *
 * \details X
 *
 * \return Return void
*/
void mw_drl_init(RC_Dynamic_RateLimT *drl_params_in);

/**
 * \brief Update function used for dynamic rate limiters
 *
 * \details X
 *
 * \return Return void
*/
void mw_drl_params_update(const float32_t new_upper_limit, const float32_t new_lower_limit, RC_Dynamic_RateLimT *drl_params_in);

/**
 * \brief Dynamic rate limiter
 *
 * \details Using defined sampling frequency!
 * 			Output value is stored in the passed in _params_in
 *
 * \return Return void
*/
void mw_dynamic_rate_limiter(const float32_t inp_val, RC_Dynamic_RateLimT *drl_params_in, const float32_t F_TS_system);

/**
 * \brief Linear algorithm
 *
 * \details X
 *
 * \return Return of value of float32_t
*/
float32_t mw_linear_algo(const float32_t inp_val, const float32_t start_val, const float32_t stop_val, const float32_t remaining_val);

/**
 * \brief Getter for filtered rider torque
 *
 * \details X
 *
 * \return Return value of float32_t
*/
float32_t mw_linear_scheduling(const float32_t scheduling_val, const float32_t A_param, const float32_t B_param,
								const float32_t high_val, const float32_t low_val);
                                
/**
 * \brief Getter for filtered rider torque
 *
 * \details X
 *
 * \return Return value of float32_t
*/
bool mw_compare_vectors_f32(const float32_t *first_vec_inp, const float32_t *second_vec_inp, uint32_t vec_len);                  

#endif /* HELPER_F_H__ */
