#ifndef KALMAN_FILTER
#define KALMAN_FILTER
namespace kalman_filter{
    class KalmanFilter{
        public:
            float Ss; // 系统预测值的方差
            float Sa; // 加速度传感器的方差
            float Sg; // 陀螺仪传感器的方差
            float T; // 获取数据的间隔

            /*
             * @param data 初始值
             * @param prediction 预测方差
             * @param sigma_system 系统方差
             * @param sigma_acce 加速度传感器方差
             * @param sigma_gyro 陀螺仪传感器方差
            */
            KalmanFilter(float data, float prediction, float sigma_system, float sigma_acce, float sigma_gyro);
            /*
             * @param acce 加速的传感器的值
             * @param gryo 陀螺仪传感器的值
             */
            float calc(float acce, float gryo);

        private:
            float Pr; // 预测值方差
            float Ds; // 上一轮系统预测的数据
            float Da; // 上一轮加速度传感器的数据
            float Dg; // 上一轮陀螺仪传感器的数据
            float Ws; // 系统预测值的权重
            float Wa; // 加速度传感器的权重
            float Wg; // 陀螺仪传感器的权重

            void predict(float new_acce, float new_gryo);
            void correct();
    };
}
#endif