#include "kalman_filter.h"

namespace kalman_filter {
    KalmanFilter::KalmanFilter(float data, float prediction, float sigma_system, float sigma_acce, float sigma_gyro) {
        Ds = data;
        Pr = prediction;
        Ss = sigma_system;
        Sa = sigma_acce;
        Sg = sigma_gyro;
        
    }
    
    // 预测
    void KalmanFilter::predict(float new_acce, float new_gryo) {
        float average_acce = (Da + new_acce) / 2;
        float avarage_gryo = (Dg + new_gryo) / 2;
        Ds = Ds + (Wa * average_acce) + (Wg * avarage_gryo);
        Pr = Pr + Ss;
        Da = new_acce;
        Dg = new_gryo;
    }

    // 更新
    void KalmanFilter::correct() {
        // 计算权重（轮换对称）
        float den = (Sa * Sg) + (Sg * Pr) + (Ss * Pr); // 分母
        Ws = (Sa * Sg) / den;
        Wa = (Sg * Pr) / den;
        Wg = (Pr * Sa) / den;

        // 归一化
        float sum = Ws + Wa + Wg;
        Ws /= sum;
        Wa /= sum;
        Wg /= sum;

        Pr = Pr * (Wa + Wg);
        Ds = (Ws * Ds) + (Wa * Da) + (Wg * Dg);
    }

    float KalmanFilter::calc(float acce, float gryo) {
        predict(acce, gryo);
        correct();
        return Ds;
    }
}