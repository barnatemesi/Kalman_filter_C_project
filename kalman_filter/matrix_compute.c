#include "matrix_compute.h"


Matrix_T mw_matrix_add(const Matrix_T *first_mat, const Matrix_T *second_mat)
{
	Matrix_T mw_temp_matrix_object = {
			.rows = NUMOFROWS,
			.cols = NUMOFELE,
			.status = false,
	};
    uint8_t rows_used = first_mat->rows;
    uint8_t cols_used = first_mat->cols;
    
    mw_init_array(mw_temp_matrix_object.matrix, 0.0F, (rows_used + cols_used));
    
    /* If rows are not equal, return with false status */
    if (first_mat->rows != second_mat->rows)
    {
        return mw_temp_matrix_object;
    }
    
    /* If cols are not equal, return with false status */
    if (first_mat->cols != second_mat->cols)
    {
        return mw_temp_matrix_object;
    }
    
    for(size_t i=0; i<(rows_used + cols_used); ++i) {
        mw_temp_matrix_object.matrix[i] =
            first_mat->matrix[i] + second_mat->matrix[i];
    }
    
    mw_temp_matrix_object.status = true;
    
	return mw_temp_matrix_object;
}

#if NUMOFROWS == 2
    VectorT mw_matrix_mult(const Matrix_T *mat_inp, const VectorT *vec_inp)
    {
        VectorT mw_temp_vector_object = {
        		.rows = NUMOFROWS,
				.arr_cap = NUMOFROWS,
				.status = false,
        };
        mw_init_array(mw_temp_vector_object.vector, 0.0F, NUMOFROWS);

        /* If matrix cols are not equal to vector rows, return with false status */
        if (mat_inp->cols != vec_inp->rows)
        {
            return mw_temp_vector_object;
        }
        
        /* Manually unrolling the for-loop */
        mw_temp_vector_object.vector[0] =
                ( mat_inp->matrix[0] * vec_inp->vector[0] ) +
                ( mat_inp->matrix[2] * vec_inp->vector[1] )
                ;
        
        mw_temp_vector_object.vector[1] =
                ( mat_inp->matrix[1] * vec_inp->vector[0] ) +
                ( mat_inp->matrix[3] * vec_inp->vector[1] )
                ;

        mw_temp_vector_object.status = true;
        
        return mw_temp_vector_object;
    }
#endif

#if NUMOFROWS == 3
    VectorT mw_matrix_mult(const Matrix_T *mat_inp, const VectorT *vec_inp)
    {
    	VectorT mw_temp_vector_object = {
    	        		.rows = NUMOFROWS,
    					.arr_cap = NUMOFROWS,
    					.status = false,
    	};
    	mw_init_array(mw_temp_vector_object.vector, 0.0F, NUMOFROWS);
        
        /* If matrix cols are not equal to vector rows, return with false status */
        if (mat_inp->cols != vec_inp->rows)
        {
            return mw_temp_vector_object;
        }
            
        for(size_t i = 0; i < 3; ++i) {
            mw_temp_vector_object.vector[i] =
                ( mat_inp->matrix[i] * vec_inp->vector[0] ) +
                ( mat_inp->matrix[i + 3] * vec_inp->vector[1] ) +
                ( mat_inp->matrix[i + 6] * vec_inp->vector[2] )
                ;
        }
        
        mw_temp_vector_object.status = true;
        
        return mw_temp_vector_object;
    }
#endif

#if NUMOFROWS == 4
    VectorT mw_matrix_mult(const Matrix_T *mat_inp, const VectorT *vec_inp)
    {
        /* Init */
    	VectorT mw_temp_vector_object = {
    	        		.rows = NUMOFROWS,
    					.arr_cap = NUMOFROWS,
    					.status = false,
    	};
    	mw_init_array(mw_temp_vector_object.vector, 0.0F, NUMOFROWS);
        
        /* If matrix cols are not equal to vector rows, return with false status */
        if (mat_inp->cols != vec_inp->rows)
        {
            return mw_temp_vector_object;
        }
            
        for(size_t i = 0; i < 4; ++i) {
            mw_temp_vector_object.vector[i] =
                ( mat_inp->matrix[i] * vec_inp->vector[0] ) +
                ( mat_inp->matrix[i + 4] * vec_inp->vector[1] ) +
                ( mat_inp->matrix[i + 8] * vec_inp->vector[2] ) +
                ( mat_inp->matrix[i + 12] * vec_inp->vector[3] )
                ;
        }
        
        mw_temp_vector_object.status = true;
        
        return mw_temp_vector_object;
    }
#endif

#if NUMOFROWS == 5
    VectorT mw_matrix_mult(const Matrix_T *mat_inp, const VectorT *vec_inp)
    {
    	VectorT mw_temp_vector_object = {
    	        		.rows = NUMOFROWS,
    					.arr_cap = NUMOFROWS,
    					.status = false,
    	};
    	mw_init_array(mw_temp_vector_object.vector, 0.0F, NUMOFROWS);
        
        /* If matrix cols are not equal to vector rows, return with false status */
        if (mat_inp->cols != vec_inp->rows)
        {
            return mw_temp_vector_object;
        }
            
        for(size_t i = 0; i < 5; ++i) {
            mw_temp_vector_object.vector[i] =
                ( mat_inp->matrix[i] * vec_inp->vector[0] ) +
                ( mat_inp->matrix[i + 5] * vec_inp->vector[1] ) +
                ( mat_inp->matrix[i + 10] * vec_inp->vector[2] ) +
                ( mat_inp->matrix[i + 15] * vec_inp->vector[3] ) +
                ( mat_inp->matrix[i + 20] * vec_inp->vector[4] )
                ;
        }
        
        mw_temp_vector_object.status = true;
        
        return mw_temp_vector_object;
    }
#endif

MatrixT mw_matrix_mult_full_classic(const MatrixT *mat_inp1, const MatrixT *mat_inp2)
{
	uint8_t cols_used = mat_inp1->cols;
	uint8_t rows_used = mat_inp1->rows;

    MatrixT mw_temp_matrix_object = {
    		.rows = rows_used,
			.cols = cols_used,
			.status = false,
    };

    
    mw_init_array(&mw_temp_matrix_object.matrix[0][0], 0.0F, (rows_used + cols_used));
    
    size_t i, j, k = 0;
    for (i = 0; i < rows_used; ++i) {
        for (j = 0; j < rows_used; ++j) {
            for (k = 0; k < rows_used; ++k) {
                mw_temp_matrix_object.matrix[i][j]
                            += mat_inp1->matrix[i][k] * mat_inp2->matrix[k][j];
            }
        }
    }
    
    mw_temp_matrix_object.status = true;
            
	return mw_temp_matrix_object;
}

Matrix_T mw_matrix_mult_full(const Matrix_T *mat_inp1, const Matrix_T *mat_inp2)
{
    (void)mat_inp2;
    Matrix_T mw_temp_matrix_object = {
    		.status = false,
    };
	uint8_t cols_used = mat_inp1->cols;
	uint8_t rows_used = mat_inp1->rows;
    
    mw_init_array(mw_temp_matrix_object.matrix, 0.0F, (rows_used + cols_used));
    
    (void)cols_used;
    (void)rows_used;

	return mw_temp_matrix_object;
}
