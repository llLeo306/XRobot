#include "comp_kalman.hpp"

using namespace Component;

KalmanFilter::KalmanFilter(KalmanFilter::Param &param) :
param_(param)
{
    mat_init(&this->KF_t.xhat, 2, 1, static_cast<float*>(this->param_.xhat_data));
    mat_init(&this->KF_t.xhatminus, 2, 1, static_cast<float*>(this->param_.xhatminus_data));
    mat_init(&this->KF_t.z, 2, 1, static_cast<float*>(this->param_.z_data));
    mat_init(&this->KF_t.A, 2, 2, static_cast<float*>(this->param_.A_data));
    mat_init(&this->KF_t.H, 2, 2, static_cast<float*>(this->param_.H_data));
    mat_init(&this->KF_t.Q, 2, 2, static_cast<float*>(this->param_.Q_data));
    mat_init(&this->KF_t.R, 2, 2, static_cast<float*>(this->param_.R_data));
    mat_init(&this->KF_t.P, 2, 2, static_cast<float*>(this->param_.P_data));
    mat_init(&this->KF_t.Pminus, 2, 2, static_cast<float*>(this->param_.Pminus_data));
    mat_init(&this->KF_t.K, 2, 2, static_cast<float*>(this->param_.K_data));
    mat_init(&this->KF_t.AT, 2, 2, static_cast<float*>(this->param_.AT_data));
    mat_trans(&this->KF_t.A, &this->KF_t.AT);
    mat_init(&this->KF_t.HT, 2, 2, static_cast<float*>(this->param_.HT_data));
    mat_trans(&this->KF_t.H, &this->KF_t.HT);
}





float* KalmanFilter::kalman_filter_calc(float signal1, float signal2) {
    const float ones_data[4] = {1, 0, 0, 1};
    float temp_data[4] = {0, 0, 0, 0};
    float temp_data21[2] = {0, 0};
    mat ones, temp, temp21;

    mat_init(&ones, 2, 2, const_cast<float*>(ones_data));
    mat_init(&temp, 2, 2, static_cast<float*>(temp_data));
    mat_init(&temp21, 2, 1, static_cast<float*>(temp_data21));

    this->KF_t.z.pData[0] = signal1;
    this->KF_t.z.pData[1] = signal2;

    // 1. xhat'(k) = A xhat(k-1)
    mat_mult(&this->KF_t.A, &this->KF_t.xhat, &this->KF_t.xhatminus);

    // 2. P'(k) = A P(k-1) AT + Q
    mat_mult(&this->KF_t.A, &this->KF_t.P, &this->KF_t.Pminus);
    mat_mult(&this->KF_t.Pminus, &this->KF_t.AT, &temp);
    mat_add(&temp, &this->KF_t.Q, &this->KF_t.Pminus);

    // 3. K(k) = P'(k) HT / (H P'(k) HT + R)
    mat_mult(&this->KF_t.H, &this->KF_t.Pminus, &this->KF_t.K);
    mat_mult(&this->KF_t.K, &this->KF_t.HT, &temp);
    mat_add(&temp, &this->KF_t.R, &this->KF_t.K);

    mat_inv(&this->KF_t.K, &this->KF_t.P);
    mat_mult(&this->KF_t.Pminus, &this->KF_t.HT, &temp);
    mat_mult(&temp, &this->KF_t.P, &this->KF_t.K);

    // 4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
    mat_mult(&this->KF_t.H, &this->KF_t.xhatminus, &temp21);
    mat_sub(&this->KF_t.z, &temp21, &this->KF_t.xhat);
    mat_mult(&this->KF_t.K, &this->KF_t.xhat, &temp21);
    mat_add(&this->KF_t.xhatminus, &temp21, &this->KF_t.xhat);

    // 5. P(k) = (1-K(k)H)P'(k)
    mat_mult(&this->KF_t.K, &this->KF_t.H, &this->KF_t.P);
    mat_sub(&ones, &this->KF_t.P, &temp);
    mat_mult(&temp, &this->KF_t.Pminus, &this->KF_t.P);

    this->KF_t.filtered_value[0] = this->KF_t.xhat.pData[0];
    this->KF_t.filtered_value[1] = this->KF_t.xhat.pData[1];
    return this->KF_t.filtered_value;
}
