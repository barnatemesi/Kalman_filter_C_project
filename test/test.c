#include <stdio.h>
#include "kalman_filter.h"
#include "matrix_data.h"
#include "helper_files.h"

// User defines
#undef DEBUG
// #define DEBUG

#define LEN_OF_DATA						(512U)

/* Declaration of global variables */

/* Ini of variables for file handling */
const char file_name[] =      "kalman_filter_validation.csv";

const char ref_file_name[]  = "../data/SIM_KF_validation.csv";

FILE *fpt = NULL;
FILE *fpt_ref = NULL;

char header[100]; // first row buffer

int main(void)
{
	printf("Start of unit testing. Hold tight!\n");

    /* Init */
    Ret_T ret_check = NOTVALID;
    uint32_t test_fail_counter = 0U;

    // The system input is a step signal
    Kalman_Filter_T kf_signals_vector = {
        .control_signal_inp = {
            .vector = {5.0F, 0.0F, 0.0F, 0.0F}, // {5.0F, 0.0F, 0.0F, 0.0F}
            .rows = NUMOFROWS_U,
            .arr_cap = NUMOFROWS,
        },
        .y_meas_inp = {
            .vector = {1.0F, 2.0F, 3.0F, 0.0F}, // {1.0F, 2.0F, 3.0F, 0.0F}
            .rows = NUMOFROWS_SENSOR_MEAS,
            .arr_cap = NUMOFROWS,
        },
        .general_status = true,
    };

    /* Global variables */
    static VectorT vector_x_k_1_state = {
        .rows = NUMOFROWS,
        .arr_cap = NUMOFROWS,
    };
    mw_init_array(vector_x_k_1_state.vector, 0U, NUMOFROWS);

    /* Vectors to store the results and variables */
    float32_t signal_no0[LEN_OF_DATA];
    float32_t signal_no1[LEN_OF_DATA];
    float32_t signal_no2[LEN_OF_DATA];
    float32_t signal_no3[LEN_OF_DATA];
    mw_init_array(signal_no0, 0U, LEN_OF_DATA);
    mw_init_array(signal_no1, 0U, LEN_OF_DATA);
    mw_init_array(signal_no2, 0U, LEN_OF_DATA);
    mw_init_array(signal_no3, 0U, LEN_OF_DATA);

    float32_t ref_signal_no0[LEN_OF_DATA];
    float32_t ref_signal_no1[LEN_OF_DATA];
    float32_t ref_signal_no2[LEN_OF_DATA];
    float32_t ref_signal_no3[LEN_OF_DATA];
    mw_init_array(ref_signal_no0, 0U, LEN_OF_DATA);
    mw_init_array(ref_signal_no1, 0U, LEN_OF_DATA);
    mw_init_array(ref_signal_no2, 0U, LEN_OF_DATA);
    mw_init_array(ref_signal_no3, 0U, LEN_OF_DATA);

    /* File handling */
    fpt = fopen(file_name, "w+");
    if (fpt==NULL)  {
        printf("File creation has failed!, %s\n", file_name);
        perror("1 - Error");
        return -1;
    }
    
    fpt_ref = fopen(ref_file_name, "r");
    if (fpt_ref==NULL)  {
    	printf("File not found, %s\n", ref_file_name);
    	perror("1 - Error");
    	return -1;
    }

    // Writing the first line in the .csv save-file
    fprintf(fpt, "Idx, omega_spindle, T_mot, T_rider, T_load \n");

    /* Skipping the header */
    fscanf(fpt_ref, "%[^\n]\n", header);

    // Init
    ret_check = init_kf_matrices(&x_k_1_ini[0], &vector_x_k_1_state);
    if (!ret_check){
    	printf("Unexpected error!\n");
    	return -1;
    }
    
    /* Body of the algorithm */
    uint32_t row_ID = 0U;
    size_t i_iter = 0U;
    float32_t w_spindle_ref, T_motor_ref, T_rider_ref, T_load_ref = 0.0F;

    /* Read reference data in from file */
    while (fscanf(fpt_ref, "%u, %f, %f, %f, %f", &row_ID, &w_spindle_ref, &T_motor_ref,
    											 &T_rider_ref, &T_load_ref) == (NUMOFROWS + 1))
    {
    	(void)row_ID;

    	/* Collect data in arrays */
    	ref_signal_no0[i_iter] = w_spindle_ref;
    	ref_signal_no1[i_iter] = T_motor_ref;
    	ref_signal_no2[i_iter] = T_rider_ref;
    	ref_signal_no3[i_iter] = T_load_ref;

    	VectorT ret_of_kf = kalman_filter_computation(&kf_signals_vector, &vector_x_k_1_state);

    	if (!ret_of_kf.status)
    	{
    		printf("An error has occurred during the KF computation!\n");
    		return -1;
    	}

		/* Saving data to file */
		fprintf(fpt, "%d, %f, %f, %f, %f\n", (int)i_iter,                  /**< number of row */
											 (double)ret_of_kf.vector[0],  /**< omega_spindle */
											 (double)ret_of_kf.vector[1],  /**< T_mot */
											 (double)ret_of_kf.vector[2],  /**< T_rider */
											 (double)ret_of_kf.vector[3]); /**< T_load */

		/* Collect data in arrays */
		signal_no0[i_iter] = ret_of_kf.vector[0];		/**< omega_spindle */
		signal_no1[i_iter] = ret_of_kf.vector[1];		/**< T_mot */
		signal_no2[i_iter] = ret_of_kf.vector[2];		/**< T_rider */
		signal_no3[i_iter] = ret_of_kf.vector[3];		/**< T_load */

    	++i_iter;
    }

    /* Comparison of reference with generated data */
    if (!mw_compare_vectors_f32(signal_no0, ref_signal_no0, i_iter))
    {
    	++test_fail_counter;
    	printf("Omega spindle test has failed!\n");
    }
    if (!mw_compare_vectors_f32(signal_no1, ref_signal_no1, i_iter))
    {
    	++test_fail_counter;
    	printf("Motor torque test has failed!\n");
    }
    if (!mw_compare_vectors_f32(signal_no2, ref_signal_no2, i_iter))
    {
    	++test_fail_counter;
    	printf("Rider torque test has failed!\n");
    }
    if (!mw_compare_vectors_f32(signal_no3, ref_signal_no3, i_iter))
    {
    	++test_fail_counter;
    	printf("Load torque test has failed!\n");
    }
    
    /* Post-processing */
    fclose(fpt);
	fclose(fpt_ref);
        
	printf("***********************************************************\n");
	printf("A Kalman filter validation file was successfully created in the Workspace folder under the name of: %s \n", file_name);
    
	printf("***********************************************************\n");
	
	if (test_fail_counter == 0U)
	{
		printf("Test has succeeded!\n");
		printf("Termination of program ...\n");
		return 0;
	} else
	{
		printf("Test has failed!\n");
		printf("Termination of program ...\n");
		return -1;
	}
}
