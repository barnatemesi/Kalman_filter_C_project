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
const char file_name[] =      "kalman_filter_validation.csv";

const char ref_file_name[]  = "data/SIM_KF_validation.csv";

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
#endif

    // Init
    ret_check = init_kf_matrices();
    if (!ret_check){
    	printf("Unexpected error!\n");
    	return -1;
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

    }

    #ifdef DEBUG
        printf("***********************************************************\n");
    #endif
    
    #ifndef DEBUG_PRINT
        fclose(fpt);
        fclose(fpt_ref);
        
        printf("***********************************************************\n");
        printf("A Kalman filter validation file was successfully created in the Workspace folder under the name of: %s \n", file_name);
    #endif
    
	printf("***********************************************************\n");
	
	return 0;
}
