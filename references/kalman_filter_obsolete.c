#include "main.h"
#include "kalman_filter.h"
#include "matrix_data.h"

#undef  DEBUG_KF
// #define DEBUG_KF

/* Every state is returned, states maybe omitted afterwards */
VectorT vector_u_used = {
    .rows = NUMOFROWS,
    .arr_cap = NUMOFROWS,
};
VectorT vector_x_state = {
    .rows = NUMOFROWS,
    .arr_cap = NUMOFROWS,
};
VectorT vector_x_k_1_state = {
    .rows = NUMOFROWS,
    .arr_cap = NUMOFROWS,
};

/**
 * \brief Properly initializes the needed vectors / matrices
 *
 * \details Works on global data
 *
 * \return  Status
 */
Ret_T init_kf_matrices(void);

/**
 * \brief Steady-state Kalman-filter computation
 *
 * \details See detailed documentation - more verbose debug information
 * 
 * \param[in]   control_signal_inp: Control signal as VectorT
 * \param[in]   y_meas_inp: Measurement signals as VectorT
 *
 * \return  returns y_est (estimated / filtered signals) and status
 */
VectorT kalman_filter_computation(const VectorT *control_signal_inp, const VectorT *y_meas_inp);

Ret_T init_kf_matrices(void)
{
    Ret_T stack_status_var = NOTVALID;
    
    /* State vector 
     * aka - Initial condition 
     */
    stack_status_var = init_vect_2_vect_copy(&vector_x_state, x_ini, NUMOFROWS);
    /* Handle error here */
    
    /* Delayed state vector */
    stack_status_var = init_vect_2_vect_copy(&vector_x_k_1_state, x_k_1_ini, NUMOFROWS);
    /* Handle error here */
    
    /* Control signal input */
    init_array(vector_u_used.vector, 0.0F, NUMOFROWS); // this is not the best
    vector_u_used.rows = NUMOFROWS_U + NUMOFROWS_SENSOR_MEAS;
    vector_u_used.arr_cap = NUMOFROWS;
    
    return stack_status_var;
}

VectorT kalman_filter_computation(const VectorT *control_signal_inp, const VectorT *y_meas_inp)
{
    /* INIT */
    VectorT y_est_vector_obj;
    init_array(y_est_vector_obj.vector, 0.0F, NUMOFROWS);
    y_est_vector_obj.arr_cap = NUMOFROWS;
    y_est_vector_obj.rows = NUMOFROWS;
    y_est_vector_obj.status = false;
    
    /* Control signal input + sensor measured signals have to fit in vector_u_used */
    if (control_signal_inp->rows + y_meas_inp->rows > NUMOFROWS)
        {
        return y_est_vector_obj;
    }
    
    /* ELSE construct vector_u_used from the inputs */
    for (size_t i=0; i<(control_signal_inp->rows); ++i)
    {
        vector_u_used.vector[i] = control_signal_inp->vector[i];
    }
    /* Appending y_meas_inp to the end of the vector */
    for (size_t i=(control_signal_inp->rows);
         i<(control_signal_inp->rows + y_meas_inp->rows); ++i)
    {
        vector_u_used.vector[i] = y_meas_inp->vector[i - control_signal_inp->rows];
    }
    
#ifdef DEBUG_KF
    printf("vector_u_used is: \n");
    print_array(vector_u_used.vector, vector_u_used.rows);
#endif
    
/* System equation */
    /* A times x */
    VectorT vector_temp1 = matrix_mult(&d_matrix_A_DU, &vector_x_k_1_state);
    
#ifdef DEBUG_KF
    if (vector_temp1.status){
        printf("Vector_temp1 is: \n");
        print_array(vector_temp1.vector, vector_temp1.rows);
    }
#endif
    
    /* B times u */
    VectorT vector_temp2 = matrix_mult(&d_matrix_B_plus_K_DU, &vector_u_used);
    
#ifdef DEBUG_KF
    if (vector_temp2.status)
    {
        printf("Vector_temp2 is: \n");
        print_array(vector_temp2.vector, vector_temp2.rows);
    }
#endif

    /* Update of x_k_1 = vector_temp1 + vector_temp2 */
    vector_x_state = vector_add(&vector_temp1, &vector_temp2);
    
#ifdef DEBUG_KF
    if (vector_x_state.status)
    {
        printf("vector_x_state is: \n");
        print_array(vector_x_state.vector, vector_x_state.rows);
    }
#endif

/* Output equation */
    /* C times x */
    vector_temp1 = matrix_mult(&d_matrix_C_DU, &vector_x_k_1_state);
    
#ifdef DEBUG_KF
    if (vector_temp1.status)
    {
        printf("Vector_temp3 is: \n");
        print_array(vector_temp1.vector, vector_temp1.rows);
    }
#endif
    
    /* D times u */
    vector_temp2 = matrix_mult(&d_matrix_D_DU, &vector_u_used);
    
#ifdef DEBUG_KF
    if (vector_temp2.status)
    {
        printf("Vector_temp4 is: \n");
        print_array(vector_temp2.vector, vector_temp1.rows);
    }
#endif
    
    y_est_vector_obj = vector_add(&vector_temp1, &vector_temp2);
    
#ifdef DEBUG_KF
    if (y_est_vector_obj.status)
    {
        printf("y_est_vector_obj is: \n");
        print_array(y_est_vector_obj.vector, y_est_vector_obj.rows);
    }
#endif

    /* Delay the vector by one-step */
    vector_x_k_1_state = vector_x_state;
    
    return y_est_vector_obj;
}

VectorT kalman_filter_computation_v2(const VectorT *control_signal_inp, const VectorT *y_meas_inp)
{
    /* INIT */
    VectorT y_est_vector_obj;
    init_array(y_est_vector_obj.vector, 0U, NUMOFROWS);
    y_est_vector_obj.arr_cap = NUMOFROWS;
    y_est_vector_obj.rows = NUMOFROWS;
    y_est_vector_obj.status = false;
    
    /* Control signal input + sensor measured signals have to fit in vector_u_used */
    if (control_signal_inp->rows + y_meas_inp->rows > NUMOFROWS)
    {
        return y_est_vector_obj;
    }
    
    /* ELSE construct vector_u_used from the inputs */
    for (size_t i=0; i<(control_signal_inp->rows); i++)
    {
        vector_u_used.vector[i] = control_signal_inp->vector[i];
    }
    /* Appending y_meas_inp to the end of the u_used vector */
    for (size_t i=(control_signal_inp->rows);
         i<(control_signal_inp->rows + y_meas_inp->rows); i++)
    {
        vector_u_used.vector[i] = y_meas_inp->vector[i - control_signal_inp->rows];
    }
    
    /* Body of the algorithm */
    vector_x_state = first_line_of_state_space(&vector_x_k_1_state, &vector_u_used);
    
    y_est_vector_obj = second_line_of_state_space(&vector_x_k_1_state, &vector_u_used);
    
    /* Delay the vector by one-step */
    vector_x_k_1_state = vector_x_state;
    
    return y_est_vector_obj;
}
