#include "kalman_filter.h"
#include "matrix_data.h"
#include "helper_files.h"

#undef  DEBUG_KF
// #define DEBUG_KF

/* User static inline function prototypes */
// Adding up two vectors
static inline VectorT kf_vector_add(const VectorT *kf_first_vector, const VectorT *kf_second_vector);
// x[k+1] = A * x[k] + B * u[k]
static inline VectorT kf_first_line_of_state_space(const VectorT *kf_state_vec_inp, const VectorT *kf_u_signal_inp);
// y[k] = C * x[k] + D * u[k]
static inline VectorT kf_second_line_of_state_space(const VectorT *kf_state_vec_inp, const VectorT *kf_u_signal_inp);
// Initialization and vector - to - vector copy
static inline Ret_T kf_init_vect_2_vect_copy(VectorT *kf_vect_result, const float32_t kf_vector_inp[],
												const uint32_t kf_len_of_vec);

static inline Ret_T kf_init_vect_2_vect_copy(VectorT *kf_vect_result, const float32_t kf_vector_inp[],
												const uint32_t kf_len_of_vec)
{
    Ret_T kf_init_return_val = NOTVALID;
    
    if (kf_len_of_vec > kf_vect_result->arr_cap)
    {
        kf_vect_result->status = false;
        kf_init_return_val = NOTVALID;
    } else
    {
        kf_vect_result->status = true;
        kf_init_return_val = VALID;
    }
    
    for (size_t i=0; i<kf_len_of_vec; ++i)
    {
        kf_vect_result->vector[i] = kf_vector_inp[i];
    }
    kf_vect_result->rows = kf_len_of_vec;
    
    return kf_init_return_val;
}

Ret_T init_kf_matrices(const float32_t kf_vector_ini_inp[], VectorT *kf_vector_x_k_1_inp)
{
    /* Delayed state vector 
     * aka - Initial condition 
     * */
    (void)kf_init_vect_2_vect_copy(kf_vector_x_k_1_inp, kf_vector_ini_inp, NUMOFROWS);
    /* Handle error here */
    
    return (Ret_T)VALID;
}

VectorT kalman_filter_computation(const Kalman_Filter_T *kf_vectors_inp, VectorT *kf_vector_x_k_1_inp)
{
    /* INIT */
    VectorT kf_y_est_vector_obj = {
    		.arr_cap = NUMOFROWS,
			.rows = NUMOFROWS,
			.status = false,
    };
    mw_init_array(kf_y_est_vector_obj.vector, 0U, NUMOFROWS);
    
    VectorT kf_vector_u_used = {
            .arr_cap = NUMOFROWS,
            .rows = NUMOFROWS_U + NUMOFROWS_SENSOR_MEAS,
			.status = false,
    };
    mw_init_array(kf_vector_u_used.vector, 0.0F, NUMOFROWS);
    
    /* Control signal input + sensor measured signals have to fit in vector_u_used */
    if ((kf_vectors_inp->control_signal_inp.rows + kf_vectors_inp->y_meas_inp.rows) > NUMOFROWS)
    {
    	/* Return it with false status */
        return kf_y_est_vector_obj;
    }
    
    /* ELSE construct vector_u_used from the inputs */
    for (size_t i=0; i<(kf_vectors_inp->control_signal_inp.rows); ++i)
    {
        kf_vector_u_used.vector[i] = kf_vectors_inp->control_signal_inp.vector[i];
    }

    /* Appending y_meas_inp to the end of the u_used vector */
    for (size_t i=(kf_vectors_inp->control_signal_inp.rows);
         i<(kf_vectors_inp->control_signal_inp.rows + kf_vectors_inp->y_meas_inp.rows); ++i)
    {
        kf_vector_u_used.vector[i] = kf_vectors_inp->y_meas_inp.vector[i - kf_vectors_inp->control_signal_inp.rows];
    }
    
    /* Body of the algorithm */
    /* x[k+1] = A * x[k] + B * u[k] */
    VectorT vector_x_state = kf_first_line_of_state_space(kf_vector_x_k_1_inp, &kf_vector_u_used);
    
    /* y[k] = C * x[k] + D * u[k] */
    kf_y_est_vector_obj = kf_second_line_of_state_space(kf_vector_x_k_1_inp, &kf_vector_u_used);
    
    /* Delay the vector by one-step */
    for (size_t i=0; i<NUMOFROWS; ++i)
    {
    	kf_vector_x_k_1_inp->vector[i] = vector_x_state.vector[i];
    }
    
    return kf_y_est_vector_obj;
}

static inline VectorT kf_vector_add(const VectorT *kf_first_vector, const VectorT *kf_second_vector)
{
    VectorT kf_temp_vector_object = {
    		.rows = NUMOFROWS,
			.arr_cap = NUMOFROWS,
			.status = false,
    };
    mw_init_array(kf_temp_vector_object.vector, 0.0F, NUMOFROWS);
    
    /* If rows are not equal, return with false status */
    if (kf_first_vector->rows != kf_second_vector->rows)
    {
        return kf_temp_vector_object;
    } else 
    {
    	kf_temp_vector_object.status = true;
    }
    
    for(size_t i=0; i<(kf_first_vector->rows); ++i)
    {
    	kf_temp_vector_object.vector[i] = kf_first_vector->vector[i] + kf_second_vector->vector[i];
    }
    
    return kf_temp_vector_object;
}

/*
 * kf_first_line_of_state_space() and
 * kf_second_line_of_state_space()
 * are conditionally compiled based on the order (rank) of the system
 */
#if NUMOFROWS == 2
    static inline VectorT kf_first_line_of_state_space(const VectorT *kf_state_vec_inp, const VectorT *kf_u_signal_inp)
    {
        /* Init */
        VectorT kf_temp_vector_object = {
        		.rows = NUMOFROWS,
    			.arr_cap = NUMOFROWS,
    			.status = false,
        };
        mw_init_array(kf_temp_vector_object.vector, 0.0F, NUMOFROWS);
        
        VectorT kf_matrix_mult_temp1 = {
                        .arr_cap = NUMOFROWS,
                        .rows = NUMOFROWS,
        };
        VectorT kf_matrix_mult_temp2 = {
                        .arr_cap = NUMOFROWS,
                        .rows = NUMOFROWS,
        };
        
        /* Manually unrolling the for-loop */
        kf_matrix_mult_temp1.vector[0] =
                ( d_matrix_A_DU.matrix[0] * kf_state_vec_inp->vector[0] ) +
                ( d_matrix_A_DU.matrix[2] * kf_state_vec_inp->vector[1] )
        ;
        
        kf_matrix_mult_temp2.vector[1] =
                ( d_matrix_B_plus_K_DU.matrix[1] * kf_u_signal_inp->vector[0] ) +
                ( d_matrix_B_plus_K_DU.matrix[3] * kf_u_signal_inp->vector[1] )
        ;
        
        kf_temp_vector_object = kf_vector_add(&kf_matrix_mult_temp1, &kf_matrix_mult_temp2);
        
        kf_temp_vector_object.status = true;
        
        return kf_temp_vector_object;
    }

    static inline VectorT kf_second_line_of_state_space(const VectorT *kf_state_vec_inp, const VectorT *kf_u_signal_inp)
    {
        /* Init */
        VectorT kf_temp_vector_object = {
        		.rows = NUMOFROWS,
    			.arr_cap = NUMOFROWS,
    			.status = false,
        };
        mw_init_array(kf_temp_vector_object.vector, 0.0F, NUMOFROWS);
        
        VectorT kf_matrix_mult_temp1 = {
                        .arr_cap = NUMOFROWS,
                        .rows = NUMOFROWS,
        };
        VectorT kf_matrix_mult_temp2 = {
                        .arr_cap = NUMOFROWS,
                        .rows = NUMOFROWS,
        };
        
        /* Manually unrolling the for-loop */
        kf_matrix_mult_temp1.vector[0] =
                ( d_matrix_C_DU.matrix[0] * kf_state_vec_inp->vector[0] ) +
                ( d_matrix_C_DU.matrix[2] * kf_state_vec_inp->vector[1] )
        ;
        
        kf_matrix_mult_temp2.vector[1] =
                ( d_matrix_D_DU.matrix[1] * kf_u_signal_inp->vector[0] ) +
                ( d_matrix_D_DU.matrix[3] * kf_u_signal_inp->vector[1] )
        ;
        
        kf_temp_vector_object = kf_vector_add(&kf_matrix_mult_temp1, &kf_matrix_mult_temp2);
        
        kf_temp_vector_object.status = true;
        
        return kf_temp_vector_object;
    }
#endif

#if NUMOFROWS == 3
    static inline VectorT kf_first_line_of_state_space(const VectorT *kf_state_vec_inp, const VectorT *kf_u_signal_inp)
    {
        /* Init */
        VectorT kf_temp_vector_object = {
        		.rows = NUMOFROWS,
    			.arr_cap = NUMOFROWS,
    			.status = false,
        };
        mw_init_array(kf_temp_vector_object.vector, 0.0F, NUMOFROWS);
        
        VectorT kf_matrix_mult_temp1 = {
                        .arr_cap = NUMOFROWS,
                        .rows = NUMOFROWS,
        };
        VectorT kf_matrix_mult_temp2 = {
                        .arr_cap = NUMOFROWS,
                        .rows = NUMOFROWS,
        };
        
        for(size_t i = 0; i < 3; ++i) {
            kf_matrix_mult_temp1.vector[i] =
                ( d_matrix_A_DU.matrix[i] * kf_state_vec_inp->vector[0] ) +
                ( d_matrix_A_DU.matrix[i + 3] * kf_state_vec_inp->vector[1] ) +
                ( d_matrix_A_DU.matrix[i + 6] * kf_state_vec_inp->vector[2] )
                ;
            kf_matrix_mult_temp2.vector[i] =
                ( d_matrix_B_plus_K_DU.matrix[i] * kf_u_signal_inp->vector[0] ) +
                ( d_matrix_B_plus_K_DU.matrix[i + 3] * kf_u_signal_inp->vector[1] ) +
                ( d_matrix_B_plus_K_DU.matrix[i + 6] * kf_u_signal_inp->vector[2] )
                ;
        }
        
        kf_temp_vector_object = kf_vector_add(&kf_matrix_mult_temp1, &kf_matrix_mult_temp2);
        
        kf_temp_vector_object.status = true;
        
        return kf_temp_vector_object;
    }

    static inline VectorT kf_second_line_of_state_space(const VectorT *kf_state_vec_inp, const VectorT *kf_u_signal_inp)
    {
        /* Init */
        VectorT kf_temp_vector_object = {
        		.rows = NUMOFROWS,
    			.arr_cap = NUMOFROWS,
    			.status = false,
        };
        mw_init_array(kf_temp_vector_object.vector, 0.0F, NUMOFROWS);
        
        VectorT kf_matrix_mult_temp1 = {
                        .arr_cap = NUMOFROWS,
                        .rows = NUMOFROWS,
        };
        VectorT kf_matrix_mult_temp2 = {
                        .arr_cap = NUMOFROWS,
                        .rows = NUMOFROWS,
        };
        
        for(size_t i = 0; i < 3; ++i) {
            kf_matrix_mult_temp1.vector[i] =
                ( d_matrix_C_DU.matrix[i] * kf_state_vec_inp->vector[0] ) +
                ( d_matrix_C_DU.matrix[i + 3] * kf_state_vec_inp->vector[1] ) +
                ( d_matrix_C_DU.matrix[i + 6] * kf_state_vec_inp->vector[2] )
                ;
            kf_matrix_mult_temp2.vector[i] =
                ( d_matrix_D_DU.matrix[i] * kf_u_signal_inp->vector[0] ) +
                ( d_matrix_D_DU.matrix[i + 3] * kf_u_signal_inp->vector[1] ) +
                ( d_matrix_D_DU.matrix[i + 6] * kf_u_signal_inp->vector[2] )
                ;
        }
        
        kf_temp_vector_object = kf_vector_add(&kf_matrix_mult_temp1, &kf_matrix_mult_temp2);
        
        kf_temp_vector_object.status = true;
        
        return kf_temp_vector_object;
    }
#endif

#if NUMOFROWS == 4
    static inline VectorT kf_first_line_of_state_space(const VectorT *kf_state_vec_inp, const VectorT *kf_u_signal_inp)
    {
        /* Init */
        VectorT kf_temp_vector_object = {
        		.rows = NUMOFROWS,
    			.arr_cap = NUMOFROWS,
    			.status = false,
        };
        mw_init_array(kf_temp_vector_object.vector, 0.0F, NUMOFROWS);
        
        VectorT kf_matrix_mult_temp1 = {
                        .arr_cap = NUMOFROWS,
                        .rows = NUMOFROWS,
        };
        VectorT kf_matrix_mult_temp2 = {
                        .arr_cap = NUMOFROWS,
                        .rows = NUMOFROWS,
        };
        
        for(size_t i = 0; i < 4; ++i) {
            kf_matrix_mult_temp1.vector[i] =
                ( d_matrix_A_DU.matrix[i] * kf_state_vec_inp->vector[0] ) +
                ( d_matrix_A_DU.matrix[i + 4] * kf_state_vec_inp->vector[1] ) +
                ( d_matrix_A_DU.matrix[i + 8] * kf_state_vec_inp->vector[2] ) +
                ( d_matrix_A_DU.matrix[i + 12] * kf_state_vec_inp->vector[3] )
                ;
            kf_matrix_mult_temp2.vector[i] =
                ( d_matrix_B_plus_K_DU.matrix[i] * kf_u_signal_inp->vector[0] ) +
                ( d_matrix_B_plus_K_DU.matrix[i + 4] * kf_u_signal_inp->vector[1] ) +
                ( d_matrix_B_plus_K_DU.matrix[i + 8] * kf_u_signal_inp->vector[2] ) +
                ( d_matrix_B_plus_K_DU.matrix[i + 12] * kf_u_signal_inp->vector[3] )
                ;
        }
        
        kf_temp_vector_object = kf_vector_add(&kf_matrix_mult_temp1, &kf_matrix_mult_temp2);
        
        kf_temp_vector_object.status = true;
        
        return kf_temp_vector_object;
    }

    static inline VectorT kf_second_line_of_state_space(const VectorT *kf_state_vec_inp, const VectorT *kf_u_signal_inp)
    {
        /* Init */
        VectorT kf_temp_vector_object = {
        		.rows = NUMOFROWS,
    			.arr_cap = NUMOFROWS,
    			.status = false,
        };
        mw_init_array(kf_temp_vector_object.vector, 0.0F, NUMOFROWS);
        
        VectorT kf_matrix_mult_temp1 = {
                        .arr_cap = NUMOFROWS,
                        .rows = NUMOFROWS,
        };
        VectorT kf_matrix_mult_temp2 = {
                        .arr_cap = NUMOFROWS,
                        .rows = NUMOFROWS,
        };
        
        for(size_t i = 0; i < 4; ++i) {
            kf_matrix_mult_temp1.vector[i] =
                ( d_matrix_C_DU.matrix[i] * kf_state_vec_inp->vector[0] ) +
                ( d_matrix_C_DU.matrix[i + 4] * kf_state_vec_inp->vector[1] ) +
                ( d_matrix_C_DU.matrix[i + 8] * kf_state_vec_inp->vector[2] ) +
                ( d_matrix_C_DU.matrix[i + 12] * kf_state_vec_inp->vector[3] )
                ;
            kf_matrix_mult_temp2.vector[i] =
                ( d_matrix_D_DU.matrix[i] * kf_u_signal_inp->vector[0] ) +
                ( d_matrix_D_DU.matrix[i + 4] * kf_u_signal_inp->vector[1] ) +
                ( d_matrix_D_DU.matrix[i + 8] * kf_u_signal_inp->vector[2] ) +
                ( d_matrix_D_DU.matrix[i + 12] * kf_u_signal_inp->vector[3] )
                ;
        }
        
        kf_temp_vector_object = kf_vector_add(&kf_matrix_mult_temp1, &kf_matrix_mult_temp2);
        
        kf_temp_vector_object.status = true;
        
        return kf_temp_vector_object;
    }
#endif

#if NUMOFROWS == 5
    static inline VectorT kf_first_line_of_state_space(const VectorT *kf_state_vec_inp, const VectorT *kf_u_signal_inp)
    {
        /* Init */
        VectorT kf_temp_vector_object = {
        		.rows = NUMOFROWS,
    			.arr_cap = NUMOFROWS,
    			.status = false,
        };
        mw_init_array(kf_temp_vector_object.vector, 0.0F, NUMOFROWS);
        
        VectorT kf_matrix_mult_temp1 = {
                        .arr_cap = NUMOFROWS,
                        .rows = NUMOFROWS,
        };
        VectorT kf_matrix_mult_temp2 = {
                        .arr_cap = NUMOFROWS,
                        .rows = NUMOFROWS,
        };
        
        for(size_t i = 0; i < 5; ++i) {
            kf_matrix_mult_temp1.vector[i] =
                ( d_matrix_A_DU.matrix[i] * kf_state_vec_inp->vector[0] ) +
                ( d_matrix_A_DU.matrix[i + 5] * kf_state_vec_inp->vector[1] ) +
                ( d_matrix_A_DU.matrix[i + 10] * kf_state_vec_inp->vector[2] ) +
                ( d_matrix_A_DU.matrix[i + 15] * kf_state_vec_inp->vector[3] ) +
                ( d_matrix_A_DU.matrix[i + 20] * kf_state_vec_inp->vector[4] )
                ;
            kf_matrix_mult_temp2.vector[i] =
                ( d_matrix_B_plus_K_DU.matrix[i] * kf_u_signal_inp->vector[0] ) +
                ( d_matrix_B_plus_K_DU.matrix[i + 5] * kf_u_signal_inp->vector[1] ) +
                ( d_matrix_B_plus_K_DU.matrix[i + 10] * kf_u_signal_inp->vector[2] ) +
                ( d_matrix_B_plus_K_DU.matrix[i + 15] * kf_u_signal_inp->vector[3] ) +
                ( d_matrix_B_plus_K_DU.matrix[i + 20] * kf_u_signal_inp->vector[4] )
                ;
        }
        
        kf_temp_vector_object = kf_vector_add(&kf_matrix_mult_temp1, &kf_matrix_mult_temp2);
        
        kf_temp_vector_object.status = true;
        
        return kf_temp_vector_object;
    }

    static inline VectorT kf_second_line_of_state_space(const VectorT *kf_state_vec_inp, const VectorT *kf_u_signal_inp)
    {
        /* Init */
        VectorT kf_temp_vector_object = {
        		.rows = NUMOFROWS,
    			.arr_cap = NUMOFROWS,
    			.status = false,
        };
        mw_init_array(kf_temp_vector_object.vector, 0.0F, NUMOFROWS);
        
        VectorT kf_matrix_mult_temp1 = {
                        .arr_cap = NUMOFROWS,
                        .rows = NUMOFROWS,
        };
        VectorT kf_matrix_mult_temp2 = {
                        .arr_cap = NUMOFROWS,
                        .rows = NUMOFROWS,
        };
        
        for(size_t i = 0; i < 5; ++i) {
            kf_matrix_mult_temp1.vector[i] =
                ( d_matrix_C_DU.matrix[i] * kf_state_vec_inp->vector[0] ) +
                ( d_matrix_C_DU.matrix[i + 5] * kf_state_vec_inp->vector[1] ) +
                ( d_matrix_C_DU.matrix[i + 10] * kf_state_vec_inp->vector[2] ) +
                ( d_matrix_C_DU.matrix[i + 15] * kf_state_vec_inp->vector[3] ) +
                ( d_matrix_C_DU.matrix[i + 20] * kf_state_vec_inp->vector[4] )
                ;
            kf_matrix_mult_temp2.vector[i] =
                ( d_matrix_D_DU.matrix[i] * kf_u_signal_inp->vector[0] ) +
                ( d_matrix_D_DU.matrix[i + 5] * kf_u_signal_inp->vector[1] ) +
                ( d_matrix_D_DU.matrix[i + 10] * kf_u_signal_inp->vector[2] ) +
                ( d_matrix_D_DU.matrix[i + 15] * kf_u_signal_inp->vector[3] ) +
                ( d_matrix_D_DU.matrix[i + 20] * kf_u_signal_inp->vector[4] )
                ;
        }
        
        kf_temp_vector_object = kf_vector_add(&kf_matrix_mult_temp1, &kf_matrix_mult_temp2);
        
        kf_temp_vector_object.status = true;
        
        return kf_temp_vector_object;
    }
#endif
