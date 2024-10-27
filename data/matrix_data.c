#include "matrix_data.h"

/* **************** E-bike DU **************** */
#if SYSTEM_SWITCH == 0
    const Matrix_T d_matrix_A_DU = {
                         .status = false,
                         .rows = NUMOFROWS,
                         .cols = NUMOFELE,
                         .matrix = {-0.9225791192F,0.7316825587F,2.1680922647F,-2.1681729712F,
                                    -0.0008319079F,-0.4839861319F,-0.0232956667F,0.0232975934F,
                                    -0.0000052088F,-0.0000309196F,0.9807935189F,0.0001458727F,
                                    0.0506601422F,0.3001158002F,1.3996981414F,-0.4187460696F},
    };

    const Matrix_T d_matrix_B_plus_K_DU = {
                         .status = false,
                         .rows = NUMOFROWS,
                         .cols = NUMOFCOLS_B_PLUS_KF,
                         .matrix = {1.4633651173F,1.9139063926F,-0.0268660321F,0.0000806982F,
                                    1.0320277363F,0.0009250986F,0.4286607090F,-0.0000019266F,
                                    -0.0000618393F,0.0000057923F,-0.0000531143F,0.0190526768F,
                                    0.6002316005F,-0.0563351171F,0.5183963439F,0.0190399988F},
    };

    const Matrix_T d_matrix_C_DU = {
                         .status = false,
                         .rows = NUMOFROWS,
                         .cols = NUMOFELE,
                         .matrix = {0.0387104404F,0.3658412793F,1.0840461324F,-1.0840864856F,
                                    -0.0004159539F,0.2580069341F,-0.0116478334F,0.0116487967F,
                                    -0.0000026044F,-0.0000154598F,0.9903967595F,0.0000729363F,
                                    0.0253300711F,0.1500579001F,0.6998490707F,0.2906269652F},
    };

    const Matrix_T d_matrix_D_DU = {
                         .status = false,
                         .rows = NUMOFROWS,
                         .cols = NUMOFELE,
                         .matrix = {0.7316825587F,0.9569531963F,-0.0134330160F,0.0000403491F,
                                    0.5160138681F,0.0004625493F,0.2143303545F,-0.0000009633F,
                                    -0.0000309196F,0.0000028961F,-0.0000265572F,0.0095263384F,
                                    0.3001158002F,-0.0281675586F,0.2591981719F,0.0095199994F},
    };
    
    /* Initial conditions */
    const float32_t x_ini[NUMOFELE] = {0.0F, 0.0F, 0.0F, 0.0F};
    const float32_t x_k_1_ini[NUMOFELE] = {0.0F, 0.0F, 0.0F, 0.0F};

#endif // SYSTEM_SWITCH == 0

/* **************** E-bike DU - Transposed matrices **************** */
#if SYSTEM_SWITCH == 1
    const Matrix_T d_matrix_A_DU = {
                         .status = false,
                         .rows = NUMOFROWS,
                         .cols = NUMOFELE,
                         .matrix = {-0.9225791192F,-0.0008319079F,-0.0000052088F,0.0506601422F,
                                    0.7316825587F,-0.4839861319F,-0.0000309196F,0.3001158002F,
                                    2.1680922647F,-0.0232956667F,0.9807935189F,1.3996981414F,
                                    -2.1681729712F,0.0232975934F,0.0001458727F,-0.4187460696F},
    };

    const Matrix_T d_matrix_B_plus_K_DU = {
                         .status = false,
                         .rows = NUMOFROWS,
                         .cols = NUMOFCOLS_B_PLUS_KF,
                         .matrix = {1.4633651173F,1.0320277363F,-0.0000618393F,0.6002316005F,
                                    1.9139063926F,0.0009250986F,0.0000057923F,-0.0563351171F,
                                    -0.0268660321F,0.4286607090F,-0.0000531143F,0.5183963439F,
                                    0.0000806982F,-0.0000019266F,0.0190526768F,0.0190399988F},
    };

    const Matrix_T d_matrix_C_DU = {
                         .status = false,
                         .rows = NUMOFROWS,
                         .cols = NUMOFELE,
                         .matrix = {0.0387104404F,-0.0004159539F,-0.0000026044F,0.0253300711F,
                                    0.3658412793F,0.2580069341F,-0.0000154598F,0.1500579001F,
                                    1.0840461324F,-0.0116478334F,0.9903967595F,0.6998490707F,
                                    -1.0840864856F,0.0116487967F,0.0000729363F,0.2906269652F},
    };

    const Matrix_T d_matrix_D_DU = {
                         .status = false,
                         .rows = NUMOFROWS,
                         .cols = NUMOFELE,
                         .matrix = {0.7316825587F,0.5160138681F,-0.0000309196F,0.3001158002F,
                                    0.9569531963F,0.0004625493F,0.0000028961F,-0.0281675586F,
                                    -0.0134330160F,0.2143303545F,-0.0000265572F,0.2591981719F,
                                    0.0000403491F,-0.0000009633F,0.0095263384F,0.0095199994F},
    };
    
    /* Initial conditions */
    const float32_t x_ini[NUMOFELE] = {0.0F, 0.0F, 0.0F, 0.0F};
    const float32_t x_k_1_ini[NUMOFELE] = {0.0F, 0.0F, 0.0F, 0.0F};

#endif // SYSTEM_SWITCH == 1

/* **************** MOC example / Thesis / Open-source **************** */
#if SYSTEM_SWITCH == 2
    const Matrix_T d_matrix_A_OPEN = {
                         .status = false,
                         .rows = NUMOFROWS,
                         .cols = NUMOFELE,
                         .matrix = {0.97670299F, -0.01353906F,
                                    0.01976703F,  0.99986461F},
    };

    const Matrix_T d_matrix_B_plus_K_OPEN = {
                         .status = false,
                         .rows = NUMOFROWS,
                         .cols = NUMOFCOLS_B_PLUS_KF,
                         .matrix = {0.01353906F,  0.02327446F,
                                    0.00013539F, -0.01976726F},
    };

    const Matrix_T d_matrix_C_OPEN = {
                         .status = false,
                         .rows = NUMOFROWS,
                         .cols = NUMOFELE,
                         .matrix = {0.98835150F, -0.00676953F,
                                    0.00988351F,  0.99993230F},
    };

    const Matrix_T d_matrix_D_OPEN = {
                         .status = false,
                         .rows = NUMOFROWS,
                         .cols = NUMOFELE,
                         .matrix = {0.00676953F,  0.01163723F,
                                    0.00006770F, -0.00988362F},
    };

    /* Initial conditions */
    const float32_t x_ini[NUMOFELE] = {0.0F, 0.0F};
    const float32_t x_k_1_ini[NUMOFELE] = {0.0F, 0.0F};

#endif // SYSTEM_SWITCH == 2
