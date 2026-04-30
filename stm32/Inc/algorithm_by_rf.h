/*#ifndef ALGORITHM_BY_RF_H_
#define ALGORITHM_BY_RF_H_

#include <stdint.h>

#define ST 4
#define FS 25

#define BUFFER_SIZE (FS * ST)
#define FS60 (FS * 60)

#define MAX_HR 125
#define MIN_HR 40

#define min_autocorrelation_ratio 0.5f
#define min_pearson_correlation 0.80f

#define mean_X ((float)(BUFFER_SIZE - 1) / 2.0f)
#define sum_X2 ((float)(BUFFER_SIZE * (BUFFER_SIZE * BUFFER_SIZE - 1)) / 12.0f)

#define LOWEST_PERIOD (FS60 / MAX_HR)
#define HIGHEST_PERIOD (FS60 / MIN_HR)

void rf_heart_rate_and_oxygen_saturation(uint32_t *pun_ir_buffer,
                                         int32_t n_ir_buffer_length,
                                         uint32_t *pun_red_buffer,
                                         float *pn_spo2,
                                         int8_t *pch_spo2_valid,
                                         int32_t *pn_heart_rate,
                                         int8_t *pch_hr_valid,
                                         float *ratio,
                                         float *correl);

float rf_linear_regression_beta(float *pn_x, float xmean, float sum_x2);
float rf_autocorrelation(float *pn_x, int32_t n_size, int32_t n_lag);
float rf_rms(float *pn_x, int32_t n_size, float *sumsq);
float rf_Pcorrelation(float *pn_x, float *pn_y, int32_t n_size);
void rf_initialize_periodicity_search(float *pn_x, int32_t n_size, int32_t *p_last_periodicity,
                                      int32_t n_max_distance, float min_aut_ratio, float aut_lag0);
void rf_signal_periodicity(float *pn_x, int32_t n_size, int32_t *p_last_periodicity,
                           int32_t n_min_distance, int32_t n_max_distance, float min_aut_ratio,
                           float aut_lag0, float *ratio);

#endif*/
#ifndef ALGORITHM_BY_RF_H_
#define ALGORITHM_BY_RF_H_

#include <stdint.h>

#define ST 4
#define FS 50

#define BUFFER_SIZE (FS * ST)
#define FS60 (FS * 60)

#define MAX_HR 125
#define MIN_HR 40

#define min_autocorrelation_ratio 0.5f
#define min_pearson_correlation 0.80f

#define mean_X ((float)(BUFFER_SIZE - 1) / 2.0f)
#define sum_X2 ((float)(BUFFER_SIZE * (BUFFER_SIZE * BUFFER_SIZE - 1)) / 12.0f)

#define LOWEST_PERIOD (FS60 / MAX_HR)
#define HIGHEST_PERIOD (FS60 / MIN_HR)

typedef struct {
    float b0, b1, b2;
    float a1, a2;

    float y_d1;
    float y_d2;
} IIR_Filter_t;

void IIR_Init(IIR_Filter_t *filter, float b0, float b1, float b2, float a1, float a2);
float IIR_Filter_Process(IIR_Filter_t *filter, float input);

void rf_heart_rate_and_oxygen_saturation(uint32_t *pun_ir_buffer,
                                         int32_t n_ir_buffer_length,
                                         uint32_t *pun_red_buffer,
                                         float *pn_spo2,
                                         int8_t *pch_spo2_valid,
                                         int32_t *pn_heart_rate,
                                         int8_t *pch_hr_valid,
                                         float *ratio,
                                         float *correl);

float rf_linear_regression_beta(float *pn_x, float xmean, float sum_x2);
float rf_autocorrelation(float *pn_x, int32_t n_size, int32_t n_lag);
float rf_rms(float *pn_x, int32_t n_size, float *sumsq);
float rf_Pcorrelation(float *pn_x, float *pn_y, int32_t n_size);
void rf_initialize_periodicity_search(float *pn_x, int32_t n_size, int32_t *p_last_periodicity,
                                      int32_t n_max_distance, float min_aut_ratio, float aut_lag0);
void rf_signal_periodicity(float *pn_x, int32_t n_size, int32_t *p_last_periodicity,
                           int32_t n_min_distance, int32_t n_max_distance, float min_aut_ratio,
                           float aut_lag0, float *ratio);

#endif
