#include "kalman_filter.h"
#include "matrix_data.h"
#include "helper_files.h"

#undef  DEBUG_KF
// #define DEBUG_KF

/* User static inline function prototypes */
//
static inline VectorT vector_add(const VectorT *first_vector, const VectorT *second_vector);
//
static inline VectorT first_line_of_state_space(const VectorT *state_vec_inp, const VectorT *u_signal_inp);
//
static inline VectorT second_line_of_state_space(const VectorT *state_vec_inp, const VectorT *u_signal_inp);
//
static inline Ret_T init_vect_2_vect_copy(VectorT *vect_result, const float32_t vector_inp[], const uint32_t len_of_vec);

/* Global variables */
static VectorT vector_x_k_1_state = {
    .rows = NUMOFROWS,
    .arr_cap = NUMOFROWS,
};

static inline Ret_T init_vect_2_vect_copy(VectorT *vect_result, const float32_t vector_inp[], const uint32_t len_of_vec)
{
    Ret_T return_val = NOTVALID;
    
    if (len_of_vec > vect_result->arr_cap)
    {
        vect_result->status = false;
        return_val = NOTVALID;
    } else
    {
        vect_result->status = true;
        return_val = VALID;
    }
    
    for (size_t i=0; i<len_of_vec; ++i)
    {
        vect_result->vector[i] = vector_inp[i];
    }
    vect_result->rows = len_of_vec;
    
    return return_val;
}

Ret_T init_kf_matrices(void)
{
    /* Delayed state vector 
     * aka - Initial condition 
     * */
    (void)init_vect_2_vect_copy(&vector_x_k_1_state, x_k_1_ini, NUMOFROWS);
    /* Handle error here */
    
    return (Ret_T)VALID;
}

VectorT kalman_filter_computation(const Kalman_Filter_T *vectors_inp)
{
    /* INIT */
    VectorT y_est_vector_obj = {
    		.arr_cap = NUMOFROWS,
			.rows = NUMOFROWS,
			.status = false,
    };
    mw_init_array(y_est_vector_obj.vector, 0U, NUMOFROWS);
    
    VectorT vector_u_used = {
            .rows = NUMOFROWS_U + NUMOFROWS_SENSOR_MEAS,
            .arr_cap = NUMOFROWS,
    };
    mw_init_array(vector_u_used.vector, 0.0F, NUMOFROWS); // this is not the best
    
    /* Control signal input + sensor measured signals have to fit in vector_u_used */
    if (vectors_inp->control_signal_inp.rows + vectors_inp->y_meas_inp.rows > NUMOFROWS)
    {
    	/* Return it with false status */
        return y_est_vector_obj;
    }
    
    /* ELSE construct vector_u_used from the inputs */
    for (size_t i=0; i<(vectors_inp->control_signal_inp.rows); ++i)
    {
        vector_u_used.vector[i] = vectors_inp->control_signal_inp.vector[i];
    }
    /* Appending y_meas_inp to the end of the u_used vector */
    for (size_t i=(vectors_inp->control_signal_inp.rows);
         i<(vectors_inp->control_signal_inp.rows + vectors_inp->y_meas_inp.rows); ++i)
    {
        vector_u_used.vector[i] = vectors_inp->y_meas_inp.vector[i - vectors_inp->control_signal_inp.rows];
    }
    
    /* Body of the algorithm */
    /* State vector */
    VectorT vector_x_state = first_line_of_state_space(&vector_x_k_1_state, &vector_u_used);
    
    y_est_vector_obj = second_line_of_state_space(&vector_x_k_1_state, &vector_u_used);
    
    /* Delay the vector by one-step */
    vector_x_k_1_state = vector_x_state;
    
    return y_est_vector_obj;
}

static inline VectorT vector_add(const VectorT *first_vector, const VectorT *second_vector)
{
    VectorT temp_vector_object = {
    		.rows = NUMOFROWS,
			.arr_cap = NUMOFROWS,
			.status = false,
    };
    mw_init_array(temp_vector_object.vector, 0.0F, NUMOFROWS);
    
    /* If rows are not equal, return with false status */
    if (first_vector->rows != second_vector->rows)
    {
        return temp_vector_object;
    } else 
    {
    	temp_vector_object.status = true;
    }
    
    for(size_t i=0; i<(first_vector->rows); ++i)
    {
    	temp_vector_object.vector[i] = first_vector->vector[i] + second_vector->vector[i];
    }
    
    return temp_vector_object;
}

/*
 * first_line_of_state_space() and
 * second_line_of_state_space() 
 * are conditionally compiled based on the order (rank) of the system
 */
#if NUMOFROWS == 2
    static inline VectorT first_line_of_state_space(const VectorT *state_vec_inp, const VectorT *u_signal_inp)
    {
        /* Init */
        VectorT temp_vector_object = {
        		.rows = NUMOFROWS,
    			.arr_cap = NUMOFROWS,
    			.status = false,
        };
        mw_init_array(temp_vector_object.vector, 0.0F, NUMOFROWS);
        
        VectorT matrix_mult_temp1 = {
                        .arr_cap = NUMOFROWS,
                        .rows = NUMOFROWS,
        };
        VectorT matrix_mult_temp2 = {
                        .arr_cap = NUMOFROWS,
                        .rows = NUMOFROWS,
        };
        
        /* Manually unrolling the for-loop */
        matrix_mult_temp1.vector[0] = 
                ( d_matrix_A_DU.matrix[0] * state_vec_inp->vector[0] ) +
                ( d_matrix_A_DU.matrix[2] * state_vec_inp->vector[1] )
        ;
        
        matrix_mult_temp2.vector[1] = 
                ( d_matrix_B_plus_K_DU.matrix[1] * u_signal_inp->vector[0] ) +
                ( d_matrix_B_plus_K_DU.matrix[3] * u_signal_inp->vector[1] )
        ;
        
        temp_vector_object = vector_add(&matrix_mult_temp1, &matrix_mult_temp2);
        
        temp_vector_object.status = true;
        
        return temp_vector_object;
    }

    static inline VectorT second_line_of_state_space(const VectorT *state_vec_inp, const VectorT *u_signal_inp)
    {
        /* Init */
        VectorT temp_vector_object = {
        		.rows = NUMOFROWS,
    			.arr_cap = NUMOFROWS,
    			.status = false,
        };
        mw_init_array(temp_vector_object.vector, 0.0F, NUMOFROWS);
        
        VectorT matrix_mult_temp1 = {
                        .arr_cap = NUMOFROWS,
                        .rows = NUMOFROWS,
        };
        VectorT matrix_mult_temp2 = {
                        .arr_cap = NUMOFROWS,
                        .rows = NUMOFROWS,
        };
        
        /* Manually unrolling the for-loop */
        matrix_mult_temp1.vector[0] = 
                ( d_matrix_C_DU.matrix[0] * state_vec_inp->vector[0] ) +
                ( d_matrix_C_DU.matrix[2] * state_vec_inp->vector[1] )
        ;
        
        matrix_mult_temp2.vector[1] = 
                ( d_matrix_D_DU.matrix[1] * u_signal_inp->vector[0] ) +
                ( d_matrix_D_DU.matrix[3] * u_signal_inp->vector[1] )
        ;
        
        temp_vector_object = vector_add(&matrix_mult_temp1, &matrix_mult_temp2);
        
        temp_vector_object.status = true;
        
        return temp_vector_object;
    }
#endif

#if NUMOFROWS == 3
    static inline VectorT first_line_of_state_space(const VectorT *state_vec_inp, const VectorT *u_signal_inp)
    {
        /* Init */
        VectorT temp_vector_object = {
        		.rows = NUMOFROWS,
    			.arr_cap = NUMOFROWS,
    			.status = false,
        };
        mw_init_array(temp_vector_object.vector, 0.0F, NUMOFROWS);
        
        VectorT matrix_mult_temp1 = {
                        .arr_cap = NUMOFROWS,
                        .rows = NUMOFROWS,
        };
        VectorT matrix_mult_temp2 = {
                        .arr_cap = NUMOFROWS,
                        .rows = NUMOFROWS,
        };
        
        for(size_t i = 0; i < 3; ++i) {
            matrix_mult_temp1.vector[i] = 
                ( d_matrix_A_DU.matrix[i] * state_vec_inp->vector[0] ) +
                ( d_matrix_A_DU.matrix[i + 3] * state_vec_inp->vector[1] ) +
                ( d_matrix_A_DU.matrix[i + 6] * state_vec_inp->vector[2] )
                ;
            matrix_mult_temp2.vector[i] = 
                ( d_matrix_B_plus_K_DU.matrix[i] * u_signal_inp->vector[0] ) +
                ( d_matrix_B_plus_K_DU.matrix[i + 3] * u_signal_inp->vector[1] ) +
                ( d_matrix_B_plus_K_DU.matrix[i + 6] * u_signal_inp->vector[2] )
                ;
        }
        
        temp_vector_object = vector_add(&matrix_mult_temp1, &matrix_mult_temp2);
        
        temp_vector_object.status = true;
        
        return temp_vector_object;
    }

    static inline VectorT second_line_of_state_space(const VectorT *state_vec_inp, const VectorT *u_signal_inp)
    {
        /* Init */
        VectorT temp_vector_object = {
        		.rows = NUMOFROWS,
    			.arr_cap = NUMOFROWS,
    			.status = false,
        };
        mw_init_array(temp_vector_object.vector, 0.0F, NUMOFROWS);
        
        VectorT matrix_mult_temp1 = {
                        .arr_cap = NUMOFROWS,
                        .rows = NUMOFROWS,
        };
        VectorT matrix_mult_temp2 = {
                        .arr_cap = NUMOFROWS,
                        .rows = NUMOFROWS,
        };
        
        for(size_t i = 0; i < 3; ++i) {
            matrix_mult_temp1.vector[i] = 
                ( d_matrix_C_DU.matrix[i] * state_vec_inp->vector[0] ) +
                ( d_matrix_C_DU.matrix[i + 3] * state_vec_inp->vector[1] ) +
                ( d_matrix_C_DU.matrix[i + 6] * state_vec_inp->vector[2] )
                ;
            matrix_mult_temp2.vector[i] = 
                ( d_matrix_D_DU.matrix[i] * u_signal_inp->vector[0] ) +
                ( d_matrix_D_DU.matrix[i + 3] * u_signal_inp->vector[1] ) +
                ( d_matrix_D_DU.matrix[i + 6] * u_signal_inp->vector[2] )
                ;
        }
        
        temp_vector_object = vector_add(&matrix_mult_temp1, &matrix_mult_temp2);
        
        temp_vector_object.status = true;
        
        return temp_vector_object;
    }
#endif

#if NUMOFROWS == 4
    static inline VectorT first_line_of_state_space(const VectorT *state_vec_inp, const VectorT *u_signal_inp)
    {
        /* Init */
        VectorT temp_vector_object = {
        		.rows = NUMOFROWS,
    			.arr_cap = NUMOFROWS,
    			.status = false,
        };
        mw_init_array(temp_vector_object.vector, 0.0F, NUMOFROWS);
        
        VectorT matrix_mult_temp1 = {
                        .arr_cap = NUMOFROWS,
                        .rows = NUMOFROWS,
        };
        VectorT matrix_mult_temp2 = {
                        .arr_cap = NUMOFROWS,
                        .rows = NUMOFROWS,
        };
        
        for(size_t i = 0; i < 4; ++i) {
            matrix_mult_temp1.vector[i] = 
                ( d_matrix_A_DU.matrix[i] * state_vec_inp->vector[0] ) +
                ( d_matrix_A_DU.matrix[i + 4] * state_vec_inp->vector[1] ) +
                ( d_matrix_A_DU.matrix[i + 8] * state_vec_inp->vector[2] ) +
                ( d_matrix_A_DU.matrix[i + 12] * state_vec_inp->vector[3] )
                ;
            matrix_mult_temp2.vector[i] = 
                ( d_matrix_B_plus_K_DU.matrix[i] * u_signal_inp->vector[0] ) +
                ( d_matrix_B_plus_K_DU.matrix[i + 4] * u_signal_inp->vector[1] ) +
                ( d_matrix_B_plus_K_DU.matrix[i + 8] * u_signal_inp->vector[2] ) +
                ( d_matrix_B_plus_K_DU.matrix[i + 12] * u_signal_inp->vector[3] )
                ;
        }
        
        temp_vector_object = vector_add(&matrix_mult_temp1, &matrix_mult_temp2);
        
        temp_vector_object.status = true;
        
        return temp_vector_object;
    }

    static inline VectorT second_line_of_state_space(const VectorT *state_vec_inp, const VectorT *u_signal_inp)
    {
        /* Init */
        VectorT temp_vector_object = {
        		.rows = NUMOFROWS,
    			.arr_cap = NUMOFROWS,
    			.status = false,
        };
        mw_init_array(temp_vector_object.vector, 0.0F, NUMOFROWS);
        
        VectorT matrix_mult_temp1 = {
                        .arr_cap = NUMOFROWS,
                        .rows = NUMOFROWS,
        };
        VectorT matrix_mult_temp2 = {
                        .arr_cap = NUMOFROWS,
                        .rows = NUMOFROWS,
        };
        
        for(size_t i = 0; i < 4; ++i) {
            matrix_mult_temp1.vector[i] = 
                ( d_matrix_C_DU.matrix[i] * state_vec_inp->vector[0] ) +
                ( d_matrix_C_DU.matrix[i + 4] * state_vec_inp->vector[1] ) +
                ( d_matrix_C_DU.matrix[i + 8] * state_vec_inp->vector[2] ) +
                ( d_matrix_C_DU.matrix[i + 12] * state_vec_inp->vector[3] )
                ;
            matrix_mult_temp2.vector[i] = 
                ( d_matrix_D_DU.matrix[i] * u_signal_inp->vector[0] ) +
                ( d_matrix_D_DU.matrix[i + 4] * u_signal_inp->vector[1] ) +
                ( d_matrix_D_DU.matrix[i + 8] * u_signal_inp->vector[2] ) +
                ( d_matrix_D_DU.matrix[i + 12] * u_signal_inp->vector[3] )
                ;
        }
        
        temp_vector_object = vector_add(&matrix_mult_temp1, &matrix_mult_temp2);
        
        temp_vector_object.status = true;
        
        return temp_vector_object;
    }
#endif

#if NUMOFROWS == 5
    static inline VectorT first_line_of_state_space(const VectorT *state_vec_inp, const VectorT *u_signal_inp)
    {
        /* Init */
        VectorT temp_vector_object = {
        		.rows = NUMOFROWS,
    			.arr_cap = NUMOFROWS,
    			.status = false,
        };
        mw_init_array(temp_vector_object.vector, 0.0F, NUMOFROWS);
        
        VectorT matrix_mult_temp1 = {
                        .arr_cap = NUMOFROWS,
                        .rows = NUMOFROWS,
        };
        VectorT matrix_mult_temp2 = {
                        .arr_cap = NUMOFROWS,
                        .rows = NUMOFROWS,
        };
        
        for(size_t i = 0; i < 5; ++i) {
            matrix_mult_temp1.vector[i] = 
                ( d_matrix_A_DU.matrix[i] * state_vec_inp->vector[0] ) +
                ( d_matrix_A_DU.matrix[i + 5] * state_vec_inp->vector[1] ) +
                ( d_matrix_A_DU.matrix[i + 10] * state_vec_inp->vector[2] ) +
                ( d_matrix_A_DU.matrix[i + 15] * state_vec_inp->vector[3] ) +
                ( d_matrix_A_DU.matrix[i + 20] * state_vec_inp->vector[4] )
                ;
            matrix_mult_temp2.vector[i] = 
                ( d_matrix_B_plus_K_DU.matrix[i] * u_signal_inp->vector[0] ) +
                ( d_matrix_B_plus_K_DU.matrix[i + 5] * u_signal_inp->vector[1] ) +
                ( d_matrix_B_plus_K_DU.matrix[i + 10] * u_signal_inp->vector[2] ) +
                ( d_matrix_B_plus_K_DU.matrix[i + 15] * u_signal_inp->vector[3] ) +
                ( d_matrix_B_plus_K_DU.matrix[i + 20] * u_signal_inp->vector[4] )
                ;
        }
        
        temp_vector_object = vector_add(&matrix_mult_temp1, &matrix_mult_temp2);
        
        temp_vector_object.status = true;
        
        return temp_vector_object;
    }

    static inline VectorT second_line_of_state_space(const VectorT *state_vec_inp, const VectorT *u_signal_inp)
    {
        /* Init */
        VectorT temp_vector_object = {
        		.rows = NUMOFROWS,
    			.arr_cap = NUMOFROWS,
    			.status = false,
        };
        mw_init_array(temp_vector_object.vector, 0.0F, NUMOFROWS);
        
        VectorT matrix_mult_temp1 = {
                        .arr_cap = NUMOFROWS,
                        .rows = NUMOFROWS,
        };
        VectorT matrix_mult_temp2 = {
                        .arr_cap = NUMOFROWS,
                        .rows = NUMOFROWS,
        };
        
        for(size_t i = 0; i < 5; ++i) {
            matrix_mult_temp1.vector[i] = 
                ( d_matrix_C_DU.matrix[i] * state_vec_inp->vector[0] ) +
                ( d_matrix_C_DU.matrix[i + 5] * state_vec_inp->vector[1] ) +
                ( d_matrix_C_DU.matrix[i + 10] * state_vec_inp->vector[2] ) +
                ( d_matrix_C_DU.matrix[i + 15] * state_vec_inp->vector[3] ) +
                ( d_matrix_C_DU.matrix[i + 20] * state_vec_inp->vector[4] )
                ;
            matrix_mult_temp2.vector[i] = 
                ( d_matrix_D_DU.matrix[i] * u_signal_inp->vector[0] ) +
                ( d_matrix_D_DU.matrix[i + 5] * u_signal_inp->vector[1] ) +
                ( d_matrix_D_DU.matrix[i + 10] * u_signal_inp->vector[2] ) +
                ( d_matrix_D_DU.matrix[i + 15] * u_signal_inp->vector[3] ) +
                ( d_matrix_D_DU.matrix[i + 20] * u_signal_inp->vector[4] )
                ;
        }
        
        temp_vector_object = vector_add(&matrix_mult_temp1, &matrix_mult_temp2);
        
        temp_vector_object.status = true;
        
        return temp_vector_object;
    }
#endif
