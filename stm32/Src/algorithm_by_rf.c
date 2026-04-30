/*#include "algorithm_by_rf.h"
#include <math.h>
#include <string.h>
#include <float.h>
#include <stddef.h>

#ifdef RF_DEBUG
#include <stdio.h>
#endif

static inline float eps_safe(float v, float eps) {
    if (fabsf(v) < eps) {
        return (v >= 0.0f) ? eps : -eps;
    }
    return v;
}

float rf_linear_regression_beta(float *pn_x, float xmean, float sum_x2)
{
    float numerator = 0.0f;
    for (int i = 0; i < BUFFER_SIZE; i++) {
        numerator += ((float)i - xmean) * pn_x[i];
    }
    if (!isfinite(sum_x2) || fabsf(sum_x2) < 1e-6f) return 0.0f;
    float beta = numerator / sum_x2;
    return beta;
}

float rf_autocorrelation(float *pn_x, int32_t n_size, int32_t n_lag)
{
    if (n_size <= 0 || n_lag < 0 || n_lag >= n_size) return 0.0f;
    float result = 0.0f;
    int32_t n = n_size - n_lag;
    for (int i = 0; i < n; i++) {
        result += pn_x[i] * pn_x[i + n_lag];
    }
    return result / (float)n;
}

float rf_rms(float *pn_x, int32_t n_size, float *sumsq)
{
    float sqsum = 0.0f;
    if (n_size <= 0) { *sumsq = 0.0f; return 0.0f; }
    for (int i = 0; i < n_size; i++) {
        sqsum += pn_x[i] * pn_x[i];
    }
    *sumsq = sqsum;
    return sqrtf(sqsum / (float)n_size);
}

float rf_Pcorrelation(float *pn_x, float *pn_y, int32_t n_size)
{
    if (n_size <= 0) return 0.0f;
    float sum_x2 = 0.0f, sum_y2 = 0.0f, sum_xy = 0.0f;
    for (int i = 0; i < n_size; i++) {
        sum_x2 += pn_x[i] * pn_x[i];
        sum_y2 += pn_y[i] * pn_y[i];
        sum_xy += pn_x[i] * pn_y[i];
    }
    if (sum_x2 <= 0.0f || sum_y2 <= 0.0f) return 0.0f;
    float denom = sqrtf(sum_x2 * sum_y2);
    if (!isfinite(denom) || denom == 0.0f) return 0.0f;
    float corr = sum_xy / denom;
    if (!isfinite(corr)) return 0.0f;
    return corr;
}

void rf_initialize_periodicity_search(
    float *pn_x, int32_t n_size, int32_t *p_last_periodicity,
    int32_t n_max_distance, float min_aut_ratio, float aut_lag0)
{
    *p_last_periodicity = 0;
    if (n_size <= 0 || !isfinite(aut_lag0) || fabsf(aut_lag0) < 1e-9f) return;

    for (int lag = 1; lag <= n_max_distance && lag < n_size; lag++) {
        float a = rf_autocorrelation(pn_x, n_size, lag);
        float r = a / aut_lag0;
        if (r >= min_aut_ratio) {
            *p_last_periodicity = lag;
            return;
        }
    }
}

#define HARMONIC_THRESHOLD 0.80f

void rf_signal_periodicity(
    float *pn_x, int32_t n_size, int32_t *p_last_periodicity,
    int32_t n_min_distance, int32_t n_max_distance,
    float min_aut_ratio, float aut_lag0, float *ratio)
{
    *ratio = 0.0f;
    if (n_size <= 0 || n_min_distance <= 0 || n_max_distance < n_min_distance) {
        *p_last_periodicity = 0;
        return;
    }

    float f_autocorr_max = 0.0f;
    int32_t n_best_lag = 0;

    for (int n_lag = n_min_distance; n_lag <= n_max_distance && n_lag < n_size; n_lag++) {
        float f_autocorr = rf_autocorrelation(pn_x, n_size, n_lag);
        if (f_autocorr > f_autocorr_max) {
            f_autocorr_max = f_autocorr;
            n_best_lag = n_lag;
        }
    }

    if (!isfinite(aut_lag0) || fabsf(aut_lag0) < 1e-9f) {
        *ratio = 0.0f;
        *p_last_periodicity = 0;
        return;
    }

    if (n_best_lag > 0) {
        int doubled = n_best_lag * 2;
        if (doubled <= n_max_distance && doubled < n_size) {
            float autocorr_double = rf_autocorrelation(pn_x, n_size, doubled);
            if (autocorr_double > HARMONIC_THRESHOLD * f_autocorr_max) {
                n_best_lag = doubled;
                f_autocorr_max = autocorr_double;
            }
        }
    }

    *ratio = f_autocorr_max / aut_lag0;

    if (*ratio > min_aut_ratio)
        *p_last_periodicity = n_best_lag;
    else
        *p_last_periodicity = 0;

#ifdef RF_DEBUG
    printf("rf_signal_periodicity: best_lag=%d f_autocorr_max=%.6f ratio=%.6f\n",
           n_best_lag, f_autocorr_max, *ratio);
#endif
}

void rf_heart_rate_and_oxygen_saturation(
    uint32_t *pun_ir_buffer,
    int32_t n_ir_buffer_length,
    uint32_t *pun_red_buffer,
    float *pn_spo2,
    int8_t *pch_spo2_valid,
    int32_t *pn_heart_rate,
    int8_t *pch_hr_valid,
    float *ratio,
    float *correl)
{
    float f_ir_mean = 0.0f, f_red_mean = 0.0f;
    static float an_x[BUFFER_SIZE], an_y[BUFFER_SIZE];
    float f_autocorr_lag0 = 0.0f, f_autocorr_ratio = 0.0f;
    float beta_ir = 0.0f, beta_red = 0.0f;
    int32_t n_i;
    int32_t n_last_peak_interval = LOWEST_PERIOD;
    float f_sumsq = 0.0f;

    *pch_hr_valid = 0;
    *pch_spo2_valid = 0;
    *pn_heart_rate = 0;
    *pn_spo2 = 0.0f;
    *ratio = 0.0f;
    *correl = 0.0f;

    if (n_ir_buffer_length <= 0 || pun_ir_buffer == NULL || pun_red_buffer == NULL) {
        return;
    }

    int32_t n_len = (n_ir_buffer_length < BUFFER_SIZE) ? n_ir_buffer_length : BUFFER_SIZE;

    float f_sum_x = 0.0f, f_sum_y = 0.0f;
    for (n_i = 0; n_i < n_len; n_i++) {
        f_sum_x += (float)pun_ir_buffer[n_i];
        f_sum_y += (float)pun_red_buffer[n_i];
    }
    f_ir_mean = f_sum_x / (float)n_len;
    f_red_mean = f_sum_y / (float)n_len;

    for (n_i = 0; n_i < n_len; n_i++) {
        an_x[n_i] = (float)pun_ir_buffer[n_i] - f_ir_mean;
        an_y[n_i] = (float)pun_red_buffer[n_i] - f_red_mean;
    }
    for (; n_i < BUFFER_SIZE; n_i++) {
        an_x[n_i] = 0.0f;
        an_y[n_i] = 0.0f;
    }

    rf_rms(an_x, n_len, &f_sumsq);
    if (n_len <= 0) return;
    f_autocorr_lag0 = f_sumsq / (float)n_len;

    if (!isfinite(f_autocorr_lag0) || fabsf(f_autocorr_lag0) < 1e-9f) {
        return;
    }

    rf_initialize_periodicity_search(an_x, n_len, &n_last_peak_interval, HIGHEST_PERIOD, min_autocorrelation_ratio, f_autocorr_lag0);

    rf_signal_periodicity(an_x, n_len, &n_last_peak_interval, LOWEST_PERIOD, HIGHEST_PERIOD, min_autocorrelation_ratio, f_autocorr_lag0, &f_autocorr_ratio);

    if (n_last_peak_interval == 0) {
        *pch_hr_valid = 0;
        *pn_heart_rate = 0;
    } else {
        int32_t hr = (int32_t)(FS60 / n_last_peak_interval);
        if ((hr < MIN_HR) || (hr > MAX_HR)) {
            *pch_hr_valid = 0;
            *pn_heart_rate = 0;
        } else {
            *pch_hr_valid = 1;
            static float hr_smoothed = 0.0f;
            const float hr_alpha = 0.25f;
            if (hr_smoothed <= 0.0f) hr_smoothed = (float)hr;
            else hr_smoothed = hr_alpha * (float)hr + (1.0f - hr_alpha) * hr_smoothed;
            *pn_heart_rate = (int32_t)(hr_smoothed + 0.5f);
        }
    }

    *correl = rf_Pcorrelation(an_x, an_y, n_len);
    if (!isfinite(*correl) || (*correl < min_pearson_correlation)) {
        *pch_spo2_valid = 0;
        *pn_spo2 = 0.0f;
        *ratio = 0.0f;
        return;
    }

    beta_ir  = rf_linear_regression_beta(an_x, mean_X, sum_X2);
    beta_red = rf_linear_regression_beta(an_y, mean_X, sum_X2);

    if (!isfinite(beta_ir) || !isfinite(beta_red) ||
        !isfinite(f_ir_mean) || !isfinite(f_red_mean) ||
        fabsf(f_ir_mean)  < 1e-6f || fabsf(f_red_mean)  < 1e-6f ||
        fabsf(beta_ir)    < 1e-12f) {
        *pch_spo2_valid = 0;
        *pn_spo2 = 0.0f;
        *ratio = 0.0f;
        return;
    }

    float ac_ir_norm  = beta_ir  / f_ir_mean;
    float ac_red_norm = beta_red / f_red_mean;

    if (!isfinite(ac_ir_norm) || fabsf(ac_ir_norm) < 1e-12f) {
        *pch_spo2_valid = 0;
        *pn_spo2 = 0.0f;
        *ratio = 0.0f;
        return;
    }

    float R = ac_red_norm / ac_ir_norm;
    if (!isfinite(R)) {
        *pch_spo2_valid = 0;
        *pn_spo2 = 0.0f;
        *ratio = 0.0f;
        return;
    }

    if (R < 0.25f) R = 0.25f;
    if (R > 1.8f)  R = 1.8f;
    *ratio = R;

    float spo2 = 120.0f - 17.0f * R;
    if (!isfinite(spo2)) spo2 = 0.0f;
    if (spo2 > 100.0f) spo2 = 100.0f;
    if (spo2 < 0.0f) spo2 = 0.0f;

    {
        static float spo2_smoothed = 0.0f;
        static int spo2_init = 0;
        const float alpha = 0.15f;
        if (!spo2_init) { spo2_smoothed = spo2; spo2_init = 1; }
        else spo2_smoothed = alpha * spo2 + (1.0f - alpha) * spo2_smoothed;
        *pn_spo2 = spo2_smoothed;
    }

    *pch_spo2_valid = 1;

#ifdef RF_DEBUG
    printf("HR=%d HR_ok=%d best_interval=%d correl=%.3f aut0=%.3f autRatio=%.3f beta_ir=%.6f beta_red=%.6f ir_mean=%.1f red_mean=%.1f R=%.3f SpO2=%.2f\n",
            *pn_heart_rate, *pch_hr_valid, n_last_peak_interval, *correl, f_autocorr_lag0, f_autocorr_ratio,
            beta_ir, beta_red, f_ir_mean, f_red_mean, R, *pn_spo2);
#endif
}*/
#include "algorithm_by_rf.h"
#include <math.h>
#include <string.h>
#include <float.h>
#include <stddef.h>

#ifdef RF_DEBUG
#include <stdio.h>
#endif

/* IIR Band-Pass Filter Coefficients (Example for Fs=50Hz, HR Band: ~0.5Hz to ~4.0Hz) */
// NOTE: These coefficients MUST be calculated using a professional DSP tool for optimal performance.
// These are example values for a 2nd order Butterworth Band-Pass Filter.
#define IIR_B0  0.02424f
#define IIR_B1  0.0f
#define IIR_B2 -0.02424f
#define IIR_A1 -1.7828f
#define IIR_A2  0.7850f

// Static filter variables for persistent state across calls
static IIR_Filter_t ir_filter;
static IIR_Filter_t red_filter;
static int filter_init_done = 0;

/* IIR Filter Implementation */

void IIR_Init(IIR_Filter_t *filter, float b0, float b1, float b2, float a1, float a2) {
    // Store coefficients
    filter->b0 = b0;
    filter->b1 = b1;
    filter->b2 = b2;
    filter->a1 = a1;
    filter->a2 = a2;

    // Initialize state variables (delay line)
    filter->y_d1 = 0.0f;
    filter->y_d2 = 0.0f;
}

float IIR_Filter_Process(IIR_Filter_t *filter, float input) {
    float output;

    // Direct Form II Transposed implementation
    // w[n] = x[n] - a1*w[n-1] - a2*w[n-2] (internal state update)
    // y[n] = b0*w[n] + b1*w[n-1] + b2*w[n-2] (output calculation)
    // However, the state update below uses the simpler (and often faster) common form where state variables (y_d1, y_d2) are used as delay lines for the output/input terms.

    // y[n] = b0*x[n] + w[n-1]
    output = filter->b0 * input + filter->y_d1;

    // Update delay states w[n-1] = b1*x[n] - a1*y[n] + w[n-2]
    filter->y_d1 = filter->b1 * input - filter->a1 * output + filter->y_d2;
    filter->y_d2 = filter->b2 * input - filter->a2 * output;

    return output;
}

/* Small helper to avoid denormals/zero division */
static inline float eps_safe(float v, float eps) {
    if (fabsf(v) < eps) {
        return (v >= 0.0f) ? eps : -eps;
    }
    return v;
}

/* ---------- Implementation (Original functions) ---------- */

float rf_linear_regression_beta(float *pn_x, float xmean, float sum_x2)
{
    float numerator = 0.0f;
    for (int i = 0; i < BUFFER_SIZE; i++) {
        numerator += ((float)i - xmean) * pn_x[i];
    }
    if (!isfinite(sum_x2) || fabsf(sum_x2) < 1e-6f) return 0.0f;
    float beta = numerator / sum_x2;
    return beta;
}

float rf_autocorrelation(float *pn_x, int32_t n_size, int32_t n_lag)
{
    if (n_size <= 0 || n_lag < 0 || n_lag >= n_size) return 0.0f;
    float result = 0.0f;
    int32_t n = n_size - n_lag;
    for (int i = 0; i < n; i++) {
        result += pn_x[i] * pn_x[i + n_lag];
    }
    return result / (float)n;
}

float rf_rms(float *pn_x, int32_t n_size, float *sumsq)
{
    float sqsum = 0.0f;
    if (n_size <= 0) { *sumsq = 0.0f; return 0.0f; }
    for (int i = 0; i < n_size; i++) {
        sqsum += pn_x[i] * pn_x[i];
    }
    *sumsq = sqsum;
    return sqrtf(sqsum / (float)n_size);
}

float rf_Pcorrelation(float *pn_x, float *pn_y, int32_t n_size)
{
    if (n_size <= 0) return 0.0f;
    float sum_x2 = 0.0f, sum_y2 = 0.0f, sum_xy = 0.0f;
    for (int i = 0; i < n_size; i++) {
        sum_x2 += pn_x[i] * pn_x[i];
        sum_y2 += pn_y[i] * pn_y[i];
        sum_xy += pn_x[i] * pn_y[i];
    }
    if (sum_x2 <= 0.0f || sum_y2 <= 0.0f) return 0.0f;
    float denom = sqrtf(sum_x2 * sum_y2);
    if (!isfinite(denom) || denom == 0.0f) return 0.0f;
    float corr = sum_xy / denom;
    if (!isfinite(corr)) return 0.0f;
    return corr;
}

void rf_initialize_periodicity_search(
    float *pn_x, int32_t n_size, int32_t *p_last_periodicity,
    int32_t n_max_distance, float min_aut_ratio, float aut_lag0)
{
    *p_last_periodicity = 0;
    if (n_size <= 0 || !isfinite(aut_lag0) || fabsf(aut_lag0) < 1e-9f) return;

    for (int lag = 1; lag <= n_max_distance && lag < n_size; lag++) {
        float a = rf_autocorrelation(pn_x, n_size, lag);
        float r = a / aut_lag0;
        if (r >= min_aut_ratio) {
            *p_last_periodicity = lag;
            return;
        }
    }
}

/* Harmonic check threshold */
#define HARMONIC_THRESHOLD 0.80f

void rf_signal_periodicity(
    float *pn_x, int32_t n_size, int32_t *p_last_periodicity,
    int32_t n_min_distance, int32_t n_max_distance,
    float min_aut_ratio, float aut_lag0, float *ratio)
{
    *ratio = 0.0f;
    if (n_size <= 0 || n_min_distance <= 0 || n_max_distance < n_min_distance) {
        *p_last_periodicity = 0;
        return;
    }

    float f_autocorr_max = 0.0f;
    int32_t n_best_lag = 0;

    for (int n_lag = n_min_distance; n_lag <= n_max_distance && n_lag < n_size; n_lag++) {
        float f_autocorr = rf_autocorrelation(pn_x, n_size, n_lag);
        if (f_autocorr > f_autocorr_max) {
            f_autocorr_max = f_autocorr;
            n_best_lag = n_lag;
        }
    }

    if (!isfinite(aut_lag0) || fabsf(aut_lag0) < 1e-9f) {
        *ratio = 0.0f;
        *p_last_periodicity = 0;
        return;
    }

    /* Harmonic check: if autocorr at 2*lag is close to autocorr_max, prefer 2*lag (to avoid picking harmonic) */
    if (n_best_lag > 0) {
        int doubled = n_best_lag * 2;
        if (doubled <= n_max_distance && doubled < n_size) {
            float autocorr_double = rf_autocorrelation(pn_x, n_size, doubled);
            if (autocorr_double > HARMONIC_THRESHOLD * f_autocorr_max) {
                n_best_lag = doubled;
                f_autocorr_max = autocorr_double;
            }
        }
    }

    *ratio = f_autocorr_max / aut_lag0;

    if (*ratio > min_aut_ratio)
        *p_last_periodicity = n_best_lag;
    else
        *p_last_periodicity = 0;

#ifdef RF_DEBUG
    printf("rf_signal_periodicity: best_lag=%d f_autocorr_max=%.6f ratio=%.6f\n",
           n_best_lag, f_autocorr_max, *ratio);
#endif
}

/* Main algorithm: heart rate and SpO2 computation. */
void rf_heart_rate_and_oxygen_saturation(
    uint32_t *pun_ir_buffer,
    int32_t n_ir_buffer_length,
    uint32_t *pun_red_buffer,
    float *pn_spo2,
    int8_t *pch_spo2_valid,
    int32_t *pn_heart_rate,
    int8_t *pch_hr_valid,
    float *ratio,
    float *correl)
{
    /* local variables */
    float f_ir_mean = 0.0f, f_red_mean = 0.0f;
    static float an_x[BUFFER_SIZE], an_y[BUFFER_SIZE]; // static to avoid large stack usage
    float f_autocorr_lag0 = 0.0f, f_autocorr_ratio = 0.0f;
    float beta_ir = 0.0f, beta_red = 0.0f;
    int32_t n_i;
    int32_t n_last_peak_interval = LOWEST_PERIOD;
    float f_sumsq = 0.0f;

    /* init outputs to safe defaults */
    *pch_hr_valid = 0;
    *pch_spo2_valid = 0;
    *pn_heart_rate = 0;
    *pn_spo2 = 0.0f;
    *ratio = 0.0f;
    *correl = 0.0f;

    if (n_ir_buffer_length <= 0 || pun_ir_buffer == NULL || pun_red_buffer == NULL) {
        return;
    }

    int32_t n_len = (n_ir_buffer_length < BUFFER_SIZE) ? n_ir_buffer_length : BUFFER_SIZE;

    /* compute means (DC component) */
    float f_sum_x = 0.0f, f_sum_y = 0.0f;
    for (n_i = 0; n_i < n_len; n_i++) {
        f_sum_x += (float)pun_ir_buffer[n_i];
        f_sum_y += (float)pun_red_buffer[n_i];
    }
    f_ir_mean = f_sum_x / (float)n_len;
    f_red_mean = f_sum_y / (float)n_len;

    /* Initialize IIR filter once */
    if (!filter_init_done) {
        IIR_Init(&ir_filter, IIR_B0, IIR_B1, IIR_B2, IIR_A1, IIR_A2);
        IIR_Init(&red_filter, IIR_B0, IIR_B1, IIR_B2, IIR_A1, IIR_A2);
        filter_init_done = 1;
    }

    /* remove DC & Apply IIR Band-Pass Filter */
    for (n_i = 0; n_i < n_len; n_i++) {
        float ir_ac = (float)pun_ir_buffer[n_i] - f_ir_mean;
        float red_ac = (float)pun_red_buffer[n_i] - f_red_mean;

        // B1: Lọc Band-Pass IIR trên tín hiệu AC (an_x và an_y giờ là tín hiệu đã lọc)
        an_x[n_i] = IIR_Filter_Process(&ir_filter, ir_ac);
        an_y[n_i] = IIR_Filter_Process(&red_filter, red_ac);
    }

    /* zero pad remainder if any */
    for (; n_i < BUFFER_SIZE; n_i++) {
        an_x[n_i] = 0.0f;
        an_y[n_i] = 0.0f;
    }

    /* compute RMS & autocorr lag0 (now using filtered AC signal) */
    rf_rms(an_x, n_len, &f_sumsq);
    if (n_len <= 0) return;
    f_autocorr_lag0 = f_sumsq / (float)n_len;

    if (!isfinite(f_autocorr_lag0) || fabsf(f_autocorr_lag0) < 1e-9f) {
        return;
    }

    /* initial periodicity test */
    rf_initialize_periodicity_search(an_x, n_len, &n_last_peak_interval, HIGHEST_PERIOD, min_autocorrelation_ratio, f_autocorr_lag0);
    /* detailed periodicity search, with harmonic handling inside */
    rf_signal_periodicity(an_x, n_len, &n_last_peak_interval, LOWEST_PERIOD, HIGHEST_PERIOD, min_autocorrelation_ratio, f_autocorr_lag0, &f_autocorr_ratio);

    /* compute HR from n_last_peak_interval */
    if (n_last_peak_interval == 0) {
        *pch_hr_valid = 0;
        *pn_heart_rate = 0;
    } else {
        int32_t hr = (int32_t)(FS60 / n_last_peak_interval);
        if ((hr < MIN_HR) || (hr > MAX_HR)) {
            *pch_hr_valid = 0;
            *pn_heart_rate = 0;
        } else {
            *pch_hr_valid = 1;
            /* smoothing HR using EMA */
            static float hr_smoothed = 0.0f;
            const float hr_alpha = 0.25f;
            if (hr_smoothed <= 0.0f) hr_smoothed = (float)hr;
            else hr_smoothed = hr_alpha * (float)hr + (1.0f - hr_alpha) * hr_smoothed;
            *pn_heart_rate = (int32_t)(hr_smoothed + 0.5f);
        }
    }

    /* Pearson correlation between red and IR (quality) */
    *correl = rf_Pcorrelation(an_x, an_y, n_len);
    if (!isfinite(*correl) || (*correl < min_pearson_correlation)) {
        *pch_spo2_valid = 0;
        *pn_spo2 = 0.0f;
        *ratio = 0.0f;
        return;
    }

    /* linear regression betas (AC components) */
    beta_ir  = rf_linear_regression_beta(an_x, mean_X, sum_X2);
    beta_red = rf_linear_regression_beta(an_y, mean_X, sum_X2);

    /* validation */
    if (!isfinite(beta_ir) || !isfinite(beta_red) ||
        !isfinite(f_ir_mean) || !isfinite(f_red_mean) ||
        fabsf(f_ir_mean)  < 1e-6f || fabsf(f_red_mean)  < 1e-6f ||
        fabsf(beta_ir)    < 1e-12f) {
        *pch_spo2_valid = 0;
        *pn_spo2 = 0.0f;
        *ratio = 0.0f;
        return;
    }

    /* normalized AC / DC */
    float ac_ir_norm  = beta_ir  / f_ir_mean;
    float ac_red_norm = beta_red / f_red_mean;

    if (!isfinite(ac_ir_norm) || fabsf(ac_ir_norm) < 1e-12f) {
        *pch_spo2_valid = 0;
        *pn_spo2 = 0.0f;
        *ratio = 0.0f;
        return;
    }

    float R = ac_red_norm / ac_ir_norm;
    if (!isfinite(R)) {
        *pch_spo2_valid = 0;
        *pn_spo2 = 0.0f;
        *ratio = 0.0f;
        return;
    }

    /* clamp ratio to reasonable range */
    if (R < 0.25f) R = 0.25f;
    if (R > 1.8f)  R = 1.8f;
    *ratio = R;

    /* simple linear mapping to SpO2 (original RF formula) */
    float spo2 = 120.0f - 17.0f * R;
    if (!isfinite(spo2)) spo2 = 0.0f;
    if (spo2 > 100.0f) spo2 = 100.0f;
    if (spo2 < 0.0f) spo2 = 0.0f;

    /* smoothing SpO2 using EMA */
    {
        static float spo2_smoothed = 0.0f;
        static int spo2_init = 0;
        const float alpha = 0.15f;
        if (!spo2_init) { spo2_smoothed = spo2; spo2_init = 1; }
        else spo2_smoothed = alpha * spo2 + (1.0f - alpha) * spo2_smoothed;
        *pn_spo2 = spo2_smoothed;
    }

    *pch_spo2_valid = 1;

#ifdef RF_DEBUG
    printf("HR=%d HR_ok=%d best_interval=%d correl=%.3f aut0=%.3f autRatio=%.3f beta_ir=%.6f beta_red=%.6f ir_mean=%.1f red_mean=%.1f R=%.3f SpO2=%.2f\n",
            *pn_heart_rate, *pch_hr_valid, n_last_peak_interval, *correl, f_autocorr_lag0, f_autocorr_ratio,
            beta_ir, beta_red, f_ir_mean, f_red_mean, R, *pn_spo2);
#endif
}
