//
//  KalmanFilter.hpp
//  
//
//  Created by 周吉夫 on 2020/5/7.
//

#ifndef KalmanFilter_hpp
#define KalmanFilter_hpp

#include <stdio.h>

#include <vector>

#include "Util.hpp"

// TODO(jifu) generalize
// define State vector<float>

using std::vector;
using util::MatAdd;
using util::MatDiag;
using util::MatId;
using util::MatInv;
using util::MatMul;
using util::MatTranspose;
using util::Print;

class KalmanFilter {
public:
    KalmanFilter() {
        
    }

    KalmanFilter(const vector<vector<float>>& m) {
        std::cout << "init KalmanFilter with" << std::endl;
        util::Print(m);
        assert(m.size() >= 2);
        measurements_.push_back(m);
        // TODO(jifu)  parameterize
        float t = 1;
        // TODO(jifu) sig_m -> fitting 2d gaussian on top k
        float sig_m = 10e-6;
        float v_x0 = m[1][0] - m[0][0];
        float v_y0 = m[1][1] - m[0][1];
        A0_ = {
            {1, 0, t*v_x0, 0},
            {0, 1, 0, t*v_y0},
            {0, 0, 1, 0},
            {0, 0, 0, 1}
        };
        std::cout << "Kalman filter init A:" << std::endl;
        Print(A0_);
        sig_d0_ = {
            {10e-6, 0, 0, 0},
            {0, 10e-6, 0, 0},
            {0, 0, 10e-6, 0},
            {0, 0, 0, 10e-6}
        };
        
        B0_ = {
            {1, 0, 0, 0},
            {0, 1, 0, 0}
        };
        sig_m_ = {{
            {sig_m, 0},
            {0, sig_m}
        }};
        x0_ = {
            { m[0][0] },
            { m[0][1] },
            { v_x0 },
            { v_y0 }
        };
        states_.push_back(x0_);
        sig_ = {
            MatAdd(sig_d0_, MatMul(A0_, A0_))
        };
    }
    
    // Applies filter given new measurements
    void apply(const vector<float>& m, vector<float>* state);
    
    void print_kalman_gain() {
        std::cout << "Kalman Gains:" << std::endl;
        util::Print(kalman_gain_metric_);
    }
private:
    vector<vector<float>> x0_;
    vector<vector<float>> A0_;
    vector<vector<float>> sig_d0_;
    vector<vector<float>> B0_;
    
    // Logging purposes
    vector<float> kalman_gain_metric_;
    vector<vector<vector<float>>> measurements_;
    vector<vector<vector<float>>> sig_m_;
    vector<vector<vector<float>>> sig_;
    vector<vector<vector<float>>> states_;
};

#endif /* KalmanFilter_hpp */
