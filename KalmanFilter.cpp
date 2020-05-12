//
//  KalmanFilter.cpp
//  
//
//  Created by 周吉夫 on 2020/5/7.
//

#include "KalmanFilter.hpp"

//void apply(const vector<vector<float>>& m, vector<vector<float>>* state) {
void KalmanFilter::apply(const vector<float>& m, vector<float>* state) {
    // Initializes
    vector<vector<float>> sig0_hat = sig_d0_;
    // TODO(jifu) time dependend sig_d1 in practice may be a func of (distance,v)
    vector<vector<float>> sig_di = sig_d0_;
    vector<vector<float>> A = A0_; // constant 1st derivative
    // TODO(jifu) time dependend sig_mi in practice may be a func of (temp match discrepancy)
    vector<vector<float>> sig_mi = sig_m_[0];
    sig_m_.push_back(sig_mi);
    vector<vector<float>> B = B0_;  // stable sensing only on pos
    vector<vector<float>> B_T = MatTranspose(B);
    vector<vector<float>> Id = MatId<float>(4);
    vector<vector<float>> measurements = {
        {m[0]},
        {m[1]}
    };
    vector<vector<float>> x_prev_post = states_.back();
    sig_di = sig_d0_;
    vector<vector<float>> sig_prev_post = sig_.back();
    
    // Predicts
    vector<vector<float>> x_curr_predicted = MatMul(A, x_prev_post);
    x_curr_predicted = MatAdd(x_curr_predicted, MatTranspose<float>({MatDiag(sig_di)}));

    // Kalman gain
    vector<vector<float>> prop_sig = MatMul(A, MatMul(sig_prev_post, A));
    vector<vector<float>> sig_curr = MatAdd(sig_di, prop_sig);
    vector<vector<float>> m_noise = MatInv(MatAdd(MatMul(B, MatMul(sig_curr, B_T)), sig_mi));
    vector<vector<float>> Ki = MatMul(sig_curr, MatMul(B_T, m_noise));

    // UpdateS & correction
    vector<vector<float>> measurement_predicted = MatMul(B, x_curr_predicted);
    vector<vector<float>> adjustment = MatAdd(measurements, MatMul<float>(-1.0, measurement_predicted));
    vector<vector<float>> x_curr_post = MatAdd(x_curr_predicted, MatMul(Ki, adjustment));
    vector<vector<float>> sig_curr_post = MatMul(MatAdd(Id, MatMul<float>(-1.0, MatMul(Ki, B))), sig_curr);
    
    // Stores current states & logging
    vector<vector<float>> curr_esitmates = {
        {x_curr_post[0][0]},
        {x_curr_post[1][0]},
        {x_curr_post[2][0]},
        {x_curr_post[3][0]}
    };
    kalman_gain_metric_.push_back(Ki[0][0]);
    states_.push_back(curr_esitmates);
    sig_.push_back(sig_curr_post);
    *state = {curr_esitmates[0][0], curr_esitmates[1][0]};
}
