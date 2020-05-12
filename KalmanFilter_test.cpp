//  KalmanFilter_test.cpp
//
//  Tests Kalman filter functions
//  g++ KalmanFilter_test.cpp KalmanFilter.cpp Util.cpp -std=c++2a && ./a.out
//
//
//  Created by 周吉夫 on 2020/5/1.
//

#include <stdio.h>

#include "KalmanFilter.hpp"
#include "Util.hpp"

// Verifies filtered results performs better
void verify(const vector<vector<float>>& pos_truth,
            const vector<vector<float>>& state_estimates,
            const vector<vector<float>>& pos_measurements) {
    Print(state_estimates);
    vector<float> diff_m_l2;
    vector<float> diff_e_l2;
    int T = pos_truth.size();
    int n = pos_truth[0].size();
    for (int i = 0; i < T; i++) {
        float diff_m_l2_i = 0;
        float diff_e_l2_i = 0;
        for (int j = 0; j < n; j++) {
            float diff_measure = std::abs(pos_truth[i][j] - pos_measurements[i][j]);
            float diff_estimate = std::abs(pos_truth[i][j] - state_estimates[i][j]);
            diff_m_l2_i += diff_measure * diff_measure;
            diff_e_l2_i += diff_estimate * diff_estimate;
        }
        diff_m_l2.push_back(std::sqrt(diff_m_l2_i));
        diff_e_l2.push_back(std::sqrt(diff_e_l2_i));
    }
    Print(diff_m_l2);
    Print(diff_e_l2);
    
}

void KF_2D_DryRun(vector<vector<float>> m, vector<vector<float>>* state_estimates, vector<vector<float>> A0, vector<vector<float>> B0, vector<vector<float>> sig_d0, vector<vector<float>> sig_m0) {
    int T = m.size();
    vector<vector<float>> sig0_hat = sig_d0;
    vector<vector<float>> sig_di = sig_d0;
    vector<vector<float>> A = A0; // constant 1st derivative
    vector<vector<float>> sig_mi = sig_m0;
    vector<vector<float>> B = B0;  // stable sensing only on pos
    vector<vector<float>> B_T = MatTranspose(B);
    
    vector<vector<float>> Id = MatId<float>(4);

    // init state and var
    float v_x0 = m[1][0] - m[0][0];
    float v_y0 = m[1][1] - m[0][1];
    vector<vector<float>> x_prev_post = {
        {m[0][0]},
        {m[0][1]},
        {v_x0},
        {v_y0}
    };
    vector<vector<float>> sig_prev_post = MatAdd(sig_di, MatMul(A, A));
    vector<float> kalman_gain_metric;
    for (int i = 2; i < T; i++) {
        vector<vector<float>> measturements = {
            {m[i][0]},
            {m[i][1]}
        };
        
        // predict
        vector<vector<float>> x_curr_predicted = MatMul(A, x_prev_post);
        x_curr_predicted = MatAdd(x_curr_predicted, MatTranspose<float>({MatDiag(sig_di)}));
        vector<vector<float>> prop_sig = MatMul(A, MatMul(sig_prev_post, A));
        vector<vector<float>> sig_curr = MatAdd(sig_di, prop_sig);
        
        // kalman gain
        vector<vector<float>> m_noise = MatInv(MatAdd(MatMul(B, MatMul(sig_curr, B_T)), sig_mi));
        vector<vector<float>> Ki = MatMul(sig_curr, MatMul(B_T, m_noise));
        kalman_gain_metric.push_back(Ki[0][0]);
        
        // update correction
        vector<vector<float>> measurement_predicted = MatMul(B, x_curr_predicted);
        vector<vector<float>> adjustment = MatAdd(measturements, MatMul<float>((float)-1.0, measurement_predicted));
        vector<vector<float>> x_curr_post = MatAdd(x_curr_predicted, MatMul(Ki, adjustment));
        vector<vector<float>> sig_curr_post = MatMul(MatAdd(Id, MatMul<float>(-1.0, MatMul(Ki, B))), sig_curr);
        
        // stores current states
        vector<float> curr_esitmates = {x_curr_post[0][0],x_curr_post[1][0]};
        state_estimates->push_back(curr_esitmates);
        x_prev_post = x_curr_post;
        sig_prev_post = sig_curr_post;
    }
    std::cout << "kalman gain metric:" << std::endl;
    Print(kalman_gain_metric);
}

void KF_2D_DryRun(vector<vector<float>> m, vector<vector<float>>* state_estimates) {
    // init
    float v_x0 = m[1][0] - m[0][0];
    float v_y0 = m[1][1] - m[0][1];
    float t = 1;
    vector<vector<float>> A0 = {
        {1, 0, t*v_x0, 0},
        {0, 1, 0, t*v_y0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    };
    vector<vector<float>> sig_d0 = {
        {10e-6, 0, 0, 0},
        {0, 10e-6, 0, 0},
        {0, 0, 10e-9, 0},
        {0, 0, 0, 10e-9}
    };
    
    vector<vector<float>> B0 = {
        {1, 0, 0, 0},
        {0, 1, 0, 0}
    };
    vector<vector<float>> sig_m0 {
        {0.05, 0},
        {0, 0.05}
    };
    KF_2D_DryRun(m, state_estimates, A0, B0, sig_d0, sig_m0);
}

// TODO(jifu)
void KF_3D(vector<vector<float>> m, vector<vector<float>>* state_estimates) {
    // init
    float v_x0 = m[1][0] - m[0][0];
    float v_y0 = m[1][1] - m[0][1];
    float v_x1 = m[2][0] - m[1][0];
    float v_y1 = m[2][1] - m[1][1];
    float a_x0 = v_x1 - v_x0;
    float a_y0 = v_y1 - v_y0;
    float t = 1;
    vector<vector<float>> A0 = {
        {1, 0, t*v_x0, 0, 0, 0},
        {0, 1, 0, t*v_y0, 0, 0},
        {0, 0, 1, 0, t*a_x0, 0},
        {0, 0, 0, 1, 0, t*a_y0},
        {0, 0, 0, 0, 1, 0},
        {0, 0, 0, 0, 0, 1}
    };
    vector<vector<float>> sig_d0 = {
        {10e-6, 0, 0, 0, 0, 0},
        {0, 10e-6, 0, 0, 0, 0},
        {0, 0, 10e-9, 0, 0, 0},
        {0, 0, 0, 10e-9, 0, 0},
        {0, 0, 0, 0, 10e-9, 0},
        {0, 0, 0, 0, 0, 10e-9}
    };
    
    vector<vector<float>> B0 = {
        {1, 0, 0, 0, 0, 0},
        {0, 1, 0, 0, 0, 0}
    };
    vector<vector<float>> sig_m0 {
        {0.05, 0},
        {0, 0.05}
    };
//    KF_3D(m, state_estimates, A0, B0, sig_d0, sig_m0);
    KF_2D_DryRun(m, state_estimates, A0, B0, sig_d0, sig_m0);
}


void KF_2D_Test() {
    std::cout << "KF_2D_Test" << std::endl;

    // constant 1st derivative
    vector<vector<float>> pos_measurements = {
        {0,0},
        {1,1},
        {2.1,1.9},
        {3.2,3.1},
        {3.9,4.1},
        {5.2,5.2},
        {5.9,5.8},
        {7.1,7.1},
        {8.2,8.1},
        {9.1,9.1}
    };
    int m = pos_measurements.size();
    int n = pos_measurements[0].size();
    vector<vector<float>> state_estimates = {
        {pos_measurements[0][0], pos_measurements[0][1]},
        {pos_measurements[1][0], pos_measurements[1][1]}
    };
    KF_2D_DryRun(pos_measurements, &state_estimates);
    vector<vector<float>> pos_truth = {
        {0,0},
        {1,1},
        {2,2},
        {3,3},
        {4,4},
        {5,5},
        {6,6},
        {7,7},
        {8,8},
        {9,9}
    };
    verify(pos_truth, state_estimates, pos_measurements);

    std::cout << "PASS" << std::endl;

}

void KF_Test() {
    std::cout << "KF_Test" << std::endl;

    unique_ptr<KalmanFilter> kf;

    float sig_m = 0.05;
    vector<vector<float>> pos_measurements = {
        {0,0},
        {1,1},
        {2.1,1.9},
        {3.2,3.1},
        {3.9,4.1},
        {5.2,5.2},
        {5.9,5.8},
        {7.1,7.1},
        {8.2,8.1},
        {9.1,9.1}
    };
    vector<vector<float>> measurements;
    for (int i = 0; i < pos_measurements.size(); i++) {
        measurements.push_back(pos_measurements[i]);
        measurements[i].push_back(sig_m);
    }
    vector<vector<float>> init_m(measurements.begin(), measurements.begin() + 2);
    kf.reset(new KalmanFilter(init_m));
    int T = measurements.size();
    
    vector<vector<float>> state_estimates = {
        {measurements[0][0], measurements[0][1]},
        {measurements[1][0], measurements[1][1]}
    };

    vector<float> curr_state;
    for (int i = 2; i < T; i++) {
        std::cout << "apply on " << i << std::endl;
        kf->apply(measurements[i], &curr_state);
        state_estimates.push_back(curr_state);
    }
    
    vector<vector<float>> pos_truth = {
        {0,0},
        {1,1},
        {2,2},
        {3,3},
        {4,4},
        {5,5},
        {6,6},
        {7,7},
        {8,8},
        {9,9}
    };
    verify(pos_truth, state_estimates, pos_measurements);
    std::cout << "PASS" << std::endl;
}

int main() {
    KF_2D_Test();
    KF_Test();
}
