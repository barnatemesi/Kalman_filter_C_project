#include "helper_files.h"

void mw_init_array(float32_t *ptr_inp, float32_t ini_val, uint32_t num_of_ele)
{
	for (size_t i=0; i<num_of_ele; ++i)
	{
		*(ptr_inp+i) = ini_val;
	}
}

void mw_init_matrix_by_rows(float32_t *matrix_inp, float32_t *each_row_inp, uint32_t rows, uint32_t cols)
{
    /* Copies row-by-row */
	for (size_t i=0; i<rows; ++i)
        {
            memcpy((matrix_inp+i), each_row_inp, sizeof(float32_t) * cols);
        }
}

void mw_print_array(const float32_t * arr_inp, const uint32_t len)
{
	for(size_t i=0; i<len; ++i)
	{
		printf("Array value [%d] is: %f\n", (uint32_t)i, *(arr_inp+i));
	}
    printf("\n");
}

void mw_print_matrix(const float32_t *matrix_inp, const uint32_t rows, const uint32_t cols)
{
	printf("Printing matrix!\n");
	for(size_t i=0; i<rows; ++i)
	{
		for(size_t j=0; j<cols; ++j)
		{
			printf("%f ", *(matrix_inp + j + i * cols));
		}
		printf("\n");
	}
}

/*
void print_array_2d(const float32_t (*arr_inp)[NUMOFELE], const uint32_t rows)
{
	printf("Printing matrix!\n");
	for(size_t i=0; i<rows; ++i)
	{
		for(size_t j=0; j<NUMOFELE; ++j)
		{
			printf("%f ", arr_inp[i][j]);
		}
		printf("\n");
	}
}
 */

float32_t mw_sat_float(const float32_t inp_val, const float32_t upper_limit, const float32_t lower_limit)
{
	float32_t stack_val = inp_val;

	if ( stack_val > upper_limit )
	{
		stack_val = upper_limit;
	}

	if ( stack_val < lower_limit)
	{
		stack_val = lower_limit;
	}

	return stack_val;
}

uint32_t mw_sat_uint(const uint32_t inp_val, const uint32_t upper_limit, const uint32_t lower_limit)
{
	uint32_t stack_val = inp_val;

	if ( stack_val > upper_limit )
	{
		stack_val = upper_limit;
	}

	if ( stack_val < lower_limit)
	{
		stack_val = lower_limit;
	}

	return stack_val;
}

uint8_t mw_sat_uint8(const uint8_t inp_val, const uint8_t upper_limit, const uint8_t lower_limit)
{
	uint8_t stack_val = inp_val;

	if ( stack_val > upper_limit )
	{
		stack_val = upper_limit;
	}

	if ( stack_val < lower_limit)
	{
		stack_val = lower_limit;
	}

	return stack_val;
}

bool mw_float_comparison(const float32_t inp_val1, const float32_t inp_val2, const float32_t eps_in)
{
	return (fabs(inp_val1 - inp_val2) < eps_in);
}

float32_t mw_zero_guard(const float32_t inp_val, const float32_t desired_val)
{
	if (inp_val < desired_val)
	{
		return desired_val;
	} else
	{
		return inp_val;
	}
}

void mw_drl_init(RC_Dynamic_RateLimT *drl_params_in)
{
	drl_params_in->signal_out_k = 0.0F;
	drl_params_in->drl_upper_limit = 1000.0F;
	drl_params_in->drl_lower_limit = 1000.0F;
	drl_params_in->status_active = false;
}

void mw_drl_params_update(const float32_t new_upper_limit, const float32_t new_lower_limit, RC_Dynamic_RateLimT *drl_params_in)
{
	drl_params_in->drl_upper_limit = new_upper_limit;
	drl_params_in->drl_lower_limit = new_lower_limit;
}

void mw_dynamic_rate_limiter(const float32_t inp_val, RC_Dynamic_RateLimT *drl_params_in, const float32_t F_TS_system)
{
	float32_t mw_drl_diff_signal = (inp_val - drl_params_in->signal_out_k);
	float32_t mw_Y_k_after_sat = mw_sat_float(mw_drl_diff_signal,
											(F_TS_system * drl_params_in->drl_upper_limit),
											(F_TS_system * drl_params_in->drl_lower_limit));

	drl_params_in->signal_out_k = drl_params_in->signal_out_k + mw_Y_k_after_sat;

	if (!mw_float_comparison(mw_drl_diff_signal, mw_Y_k_after_sat, 0.005F))
	{
		drl_params_in->status_active = true;
	} else
	{
		drl_params_in->status_active = false;
	}
}

float32_t mw_linear_algo(const float32_t inp_val, const float32_t start_val, const float32_t stop_val, const float32_t remaining_val)
{
	float32_t factor_out = 0.0F;

	// a_temp
	float32_t a_factor_val = 1.0F / mw_zero_guard((1.0F - (start_val / mw_zero_guard(stop_val, 0.001F))), 0.001F);

	// factor_out
	factor_out = (1.0F - (inp_val / mw_zero_guard(stop_val, 1.0F))) * a_factor_val + remaining_val;
	factor_out = mw_sat_float(factor_out, 1.0F, 0.0F);

	return factor_out;
}

float32_t mw_linear_scheduling(const float32_t scheduling_val, const float32_t A_param, const float32_t B_param,
								const float32_t high_val, const float32_t low_val)
{
	float32_t a_lpf_sched = 1.0F / mw_zero_guard((1.0F - (low_val / mw_zero_guard(high_val, 0.001F))), 0.001F);

	float32_t factor_used = mw_sat_float(((1.0F - (scheduling_val / mw_zero_guard(high_val, 0.001F))) * a_lpf_sched), 1.0F, 0.0F);

	return (A_param * factor_used + B_param * (1.0F - factor_used));
}

bool mw_compare_vectors_f32(const float32_t *first_vec_inp, const float32_t *second_vec_inp, uint32_t vec_len)
{
	float32_t stack_eps = 0.001F;
	bool stack_status[vec_len];
	uint32_t stack_status_sum = 0U;
	(void)memset(&stack_status, 0U, sizeof(stack_status));

	for (size_t i=0; i<vec_len; ++i)
	{
		stack_status[i] = !mw_float_comparison(*(first_vec_inp + i), *(second_vec_inp + i),stack_eps);
		stack_status_sum =+ (uint32_t)stack_status[i];
	}

	if (stack_status_sum == 0)
	{
		return true;
	} else
	{
		return false;
	}

}

