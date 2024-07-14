#include<iostream>
#include "kalman_filter.h"
using namespace std;
using namespace kalman_filter;

int main() {
    KalmanFilter k = KalmanFilter(0, 1, 1, 0 ,1);
    for(int i = 1; i < 100; i++) {
        float result = k.calc(i, i+1);
        cout << result << endl;
    }
}
