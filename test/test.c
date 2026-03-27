#include <stdio.h>
#include "unity.h"
#include "kalman_filter.h"
#include "matrix_data.h"
#include "helper_files.h"

#define LEN_OF_DATA (512U)
#define FLOAT_TOLERANCE (0.001F)

/* File paths */
static const char output_file_name[] = "kalman_filter_validation.csv";
static const char ref_file_name[] = "../data/SIM_KF_validation.csv";

/* Shared test state populated by setUp */
static Kalman_Filter_T kf_signals_vector;
static VectorT vector_x_k_1_state;

static float32_t signal_no0[LEN_OF_DATA];
static float32_t signal_no1[LEN_OF_DATA];
static float32_t signal_no2[LEN_OF_DATA];
static float32_t signal_no3[LEN_OF_DATA];

static float32_t ref_signal_no0[LEN_OF_DATA];
static float32_t ref_signal_no1[LEN_OF_DATA];
static float32_t ref_signal_no2[LEN_OF_DATA];
static float32_t ref_signal_no3[LEN_OF_DATA];

static size_t num_samples;
static bool kf_data_ready = false;

/**
 * \brief Run the KF over the reference dataset and collect output signals.
 * \details Called once; results are reused by individual signal tests.
 */
static void run_kf_and_load_reference(void)
{
    if (kf_data_ready)
    {
        return;
    }

    /* KF input: step signal */
    kf_signals_vector = (Kalman_Filter_T){
        .control_signal_inp =
            {
                .vector = {5.0F, 0.0F, 0.0F, 0.0F},
                .rows = NUMOFROWS_U,
                .arr_cap = NUMOFROWS,
            },
        .y_meas_inp =
            {
                .vector = {1.0F, 2.0F, 3.0F, 0.0F},
                .rows = NUMOFROWS_SENSOR_MEAS,
                .arr_cap = NUMOFROWS,
            },
        .general_status = true,
    };

    vector_x_k_1_state = (VectorT){
        .rows = NUMOFROWS,
        .arr_cap = NUMOFROWS,
    };
    mw_init_array(vector_x_k_1_state.vector, 0.0F, NUMOFROWS);

    mw_init_array(signal_no0, 0.0F, LEN_OF_DATA);
    mw_init_array(signal_no1, 0.0F, LEN_OF_DATA);
    mw_init_array(signal_no2, 0.0F, LEN_OF_DATA);
    mw_init_array(signal_no3, 0.0F, LEN_OF_DATA);
    mw_init_array(ref_signal_no0, 0.0F, LEN_OF_DATA);
    mw_init_array(ref_signal_no1, 0.0F, LEN_OF_DATA);
    mw_init_array(ref_signal_no2, 0.0F, LEN_OF_DATA);
    mw_init_array(ref_signal_no3, 0.0F, LEN_OF_DATA);

    Ret_T ret_check = init_kf_matrices(&x_k_1_ini[0], &vector_x_k_1_state);
    TEST_ASSERT_TRUE_MESSAGE(ret_check, "KF matrix initialisation failed");

    /* Open reference file */
    FILE *fpt_ref = fopen(ref_file_name, "r");
    TEST_ASSERT_NOT_NULL_MESSAGE(fpt_ref, "Could not open reference CSV");

    /* Open output file */
    FILE *fpt = fopen(output_file_name, "w+");
    TEST_ASSERT_NOT_NULL_MESSAGE(fpt, "Could not create output CSV");
    fprintf(fpt, "Idx, omega_spindle, T_mot, T_rider, T_load \n");

    /* Skip header */
    char header[100];
    fscanf(fpt_ref, "%99[^\n]\n", header);

    uint32_t row_ID = 0U;
    num_samples = 0U;
    float32_t w_spindle_ref = 0.0F, T_motor_ref = 0.0F, T_rider_ref = 0.0F, T_load_ref = 0.0F;

    while ((num_samples < LEN_OF_DATA) &&
           fscanf(fpt_ref, "%u, %f, %f, %f, %f",
                  &row_ID, &w_spindle_ref, &T_motor_ref, &T_rider_ref, &T_load_ref) == (NUMOFROWS + 1))
    {
        (void)row_ID;

        ref_signal_no0[num_samples] = w_spindle_ref;
        ref_signal_no1[num_samples] = T_motor_ref;
        ref_signal_no2[num_samples] = T_rider_ref;
        ref_signal_no3[num_samples] = T_load_ref;

        VectorT ret_of_kf = kalman_filter_computation(&kf_signals_vector, &vector_x_k_1_state);
        TEST_ASSERT_TRUE_MESSAGE(ret_of_kf.status, "KF computation returned error status");

        fprintf(fpt, "%d, %f, %f, %f, %f\n",
                (int)num_samples,
                (double)ret_of_kf.vector[0],
                (double)ret_of_kf.vector[1],
                (double)ret_of_kf.vector[2],
                (double)ret_of_kf.vector[3]);

        signal_no0[num_samples] = ret_of_kf.vector[0];
        signal_no1[num_samples] = ret_of_kf.vector[1];
        signal_no2[num_samples] = ret_of_kf.vector[2];
        signal_no3[num_samples] = ret_of_kf.vector[3];

        ++num_samples;
    }

    fclose(fpt);
    fclose(fpt_ref);

    TEST_ASSERT_GREATER_THAN_UINT32_MESSAGE(0U, (uint32_t)num_samples,
                                            "No reference data was read");
    kf_data_ready = true;
}

/* Unity required hooks */
void setUp(void)
{
    run_kf_and_load_reference();
}

void tearDown(void) {}

/* ---- Individual signal validation tests ---- */

static void assert_signal_matches_reference(const float32_t *actual,
                                            const float32_t *expected,
                                            size_t len,
                                            const char *signal_name)
{
    for (size_t i = 0; i < len; ++i)
    {
        char msg[128];
        snprintf(msg, sizeof(msg), "%s mismatch at sample %zu", signal_name, i);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(FLOAT_TOLERANCE, expected[i], actual[i], msg);
    }
}

void test_omega_spindle(void)
{
    assert_signal_matches_reference(signal_no0, ref_signal_no0, num_samples, "omega_spindle");
}

void test_motor_torque(void)
{
    assert_signal_matches_reference(signal_no1, ref_signal_no1, num_samples, "T_motor");
}

void test_rider_torque(void)
{
    assert_signal_matches_reference(signal_no2, ref_signal_no2, num_samples, "T_rider");
}

void test_load_torque(void)
{
    assert_signal_matches_reference(signal_no3, ref_signal_no3, num_samples, "T_load");
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_omega_spindle);
    RUN_TEST(test_motor_torque);
    RUN_TEST(test_rider_torque);
    RUN_TEST(test_load_torque);
    return UNITY_END();
}
