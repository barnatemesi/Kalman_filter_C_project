#include "main.h"
#include "kalman_filter.h"
#include "helper_files.h"

#undef DEBUG
// #define DEBUG

#undef DEBUG_PRINT
// #define DEBUG_PRINT

#ifdef DEBUG_PRINT
    #define TIMESTEPS                       (5U)
#else
    #define TIMESTEPS                       (438U) // 1251U
#endif

#define LEN_OF_DATA							(512U)

/* Ini of variables for file handling */
const char file_name[] =      "kalman_filter_validation_v2.csv";
const char file_name1[] =     "data/kalman_filter_validation_v2.csv";

const char ref_file_name[]  = "data/SIM_KF_validation.csv";
const char ref_file_name1[] = "ref_profile_08_09_2023_v00.csv";

FILE *fpt = NULL;
FILE *fpt_ref = NULL;

char header[100]; // first row buffer

/* Declaration of global variables */
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

/* Global variables storing the test vectors */
float32_t signal_no0[LEN_OF_DATA];
float32_t signal_no1[LEN_OF_DATA];
float32_t signal_no2[LEN_OF_DATA];
float32_t signal_no3[LEN_OF_DATA];

float32_t ref_signal_no0[LEN_OF_DATA];
float32_t ref_signal_no1[LEN_OF_DATA];
float32_t ref_signal_no2[LEN_OF_DATA];
float32_t ref_signal_no3[LEN_OF_DATA];

/* Declaration of input scalar and measurement vector */
const float32_t u_vect[NUMOFROWS_U] = {5.0F};

const float32_t y_meas_vect[NUMOFROWS_SENSOR_MEAS] = {1.0F, 2.0F, 3.0F};

int main(void)
{
    /* Init */
    Ret_T ret_check = NOTVALID;


#ifndef DEBUG_PRINT
    fpt = fopen(file_name, "w+");
    if (fpt==NULL)  {
        printf("File not found, %s\n", file_name);
        perror("1 - Error");
        return errno;
    }
    
    fpt_ref = fopen(ref_file_name, "r");
    if (fpt_ref==NULL)  {
           printf("File not found, %s\n", ref_file_name);
           perror("1 - Error");
           return errno;
    }

    // Writing the first line in the .csv save-file
    fprintf(fpt, "Idx, omega_spindle, T_mot, T_rider, T_load \n");

    /* Skipping the header */
    fscanf(fpt_ref, "%[^\n]\n", header);
#endif

    // Init
    ret_check = init_kf_matrices();
    if (!ret_check){
            printf("Unexpected error!\n");
    }
    
    // Body of the algorithm
    for (size_t i=0; i<TIMESTEPS; ++i)
    {
    	VectorT ret_of_kf = kalman_filter_computation(&kf_signals_vector);

    	if (!ret_of_kf.status)
    	{
    		printf("An error has occurred during the KF computation!\n");
    	}
        
    #ifdef DEBUG_PRINT
        printf("idx is: %d\n", (int)i);
        printf("ret_of_kf output is:\n");
        
        print_array(ret_of_kf.vector, ret_of_kf.rows);
        
        printf("***********************************************************\n");
    #else
        /* Saving data to file */
        fprintf(fpt, "%d, %f, %f, %f, %f\n", (int)i,                       /**< number of row */
                                             (double)ret_of_kf.vector[0],  /**< omega_spindle */
                                             (double)ret_of_kf.vector[1],  /**< T_mot */
                                             (double)ret_of_kf.vector[2],  /**< T_rider */
                                             (double)ret_of_kf.vector[3]); /**< T_load */
    #endif

    /* Collect data in arrays */
    signal_no0[i] = ret_of_kf.vector[0];
    signal_no1[i] = ret_of_kf.vector[1];
    signal_no2[i] = ret_of_kf.vector[2];
    signal_no3[i] = ret_of_kf.vector[3];
    }
    
    uint32_t row_ID = 0U;
    size_t i_iter = 0U;
    float32_t w_spindle_ref, T_motor_ref, T_rider_ref, T_load_ref = 0.0F;

    /* Read reference data in from file */
    while (fscanf(fpt_ref, "%u, %f, %f, %f, %f", &row_ID, &w_spindle_ref, &T_motor_ref,
    											 &T_rider_ref, &T_load_ref) == 5)
    {
    	(void)row_ID;

    	/* Collect data in arrays */
    	ref_signal_no0[i_iter] = w_spindle_ref;
    	ref_signal_no1[i_iter] = T_motor_ref;
    	ref_signal_no2[i_iter] = T_rider_ref;
    	ref_signal_no3[i_iter] = T_load_ref;

    	++i_iter;
    }

    #ifdef DEBUG
        printf("***********************************************************\n");
    #endif

    /* Comparison of reference with generated data */
	mw_compare_vectors_f32(&kf_signals_vector.control_signal_inp.vector[0], &kf_signals_vector.control_signal_inp.vector[0], 4U) ? printf("Test vectors are the same!\n") : printf("Test vectors are not the same!\n");

    mw_compare_vectors_f32(signal_no0, ref_signal_no0, TIMESTEPS) ? printf("Omega_spindle vectors are the same!\n") : printf("Omega_spindle vectors are not the same!\n");
    mw_compare_vectors_f32(signal_no1, ref_signal_no1, TIMESTEPS) ? printf("T_motor vectors are the same!\n") : printf("T_motor vectors are not the same!\n");
    mw_compare_vectors_f32(signal_no2, ref_signal_no2, TIMESTEPS) ? printf("T_rider vectors are the same!\n") : printf("T_rider vectors are not the same!\n");
    mw_compare_vectors_f32(signal_no3, ref_signal_no3, TIMESTEPS) ? printf("T_estimated_load vectors are the same!\n") : printf("T_estimated_load vectors are not the same!\n");
    
    #ifndef DEBUG_PRINT
        fclose(fpt);
        fclose(fpt_ref);
        
        printf("***********************************************************\n");
        printf("A Kalman filter validation file was successfully created in the Workspace folder under the name of: %s \n", file_name);
    #endif
    
	printf("***********************************************************\n");
	
	return 0;
}
