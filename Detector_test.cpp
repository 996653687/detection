//  Detector_test.cpp
//
// Tests detector functions:
// g++ Detector_test.cpp Detector.cpp Util.cpp -std=c++2a && ./a.out
//
//
//
//  Created by 周吉夫 on 2020/5/8.
//

#include <stdio.h>

#include "Detector.hpp"

    void detect_test() {
        std::cout << "detect_test" << std::endl;

        vector<vector<int>> tmpl_b = {
            {0, 1, 0},
            {1, 1, 1},
            {0, 1, 0}
        };
        int n = tmpl_b.size();
        int m = tmpl_b[0].size();
        vector<vector<vector<int>>> tmpl(n, vector<vector<int>>(m, vector<int>(3, 0)));
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                tmpl[i][j][0] = tmpl_b[i][j];
                tmpl[i][j][1] = tmpl_b[i][j];
                tmpl[i][j][2] = tmpl_b[i][j];
            }
        }
        int downsample_scale = 1;
        int top_k = 20;
        unique_ptr<Detection> detector(new Detection(tmpl, downsample_scale, top_k));

        vector<vector<int>> frame_b = {
            {0, 0, 0, 0},
            {0, 0, 1, 0},
            {0, 1, 1, 1},
            {0, 0, 1, 0}
        };
        n = frame_b.size();
        m = frame_b[0].size();
        vector<vector<vector<int>>> frame(n, vector<vector<int>>(m, vector<int>(3, 0)));
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                frame[i][j][0] = frame_b[i][j];
                frame[i][j][1] = frame_b[i][j];
                frame[i][j][2] = frame_b[i][j];
            }
        }

        vector<float> curr_det;
        detector->detect(frame, &curr_det);

        util::Print(curr_det);
        vector<float> ans = {2, 2, 3, 3};
        assert(util::TestEQ<float>(ans, curr_det));
        std::cout << "PASS" << std::endl;

    }

    void response_top_K_test() {
        std::cout << "response_top_K_test" << std::endl;
        
        vector<vector<int>> tmpl_b = {
            {0, 1, 0},
            {1, 1, 1},
            {0, 1, 0}
        };
        int n = tmpl_b.size();
        int m = tmpl_b[0].size();
        vector<vector<vector<int>>> tmpl(n, vector<vector<int>>(m, vector<int>(3, 0)));
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                tmpl[i][j][0] = tmpl_b[i][j];
                tmpl[i][j][1] = tmpl_b[i][j];
                tmpl[i][j][2] = tmpl_b[i][j];
            }
        }
        int downsample_scale = 1;
        unique_ptr<Detection> detector(new Detection(tmpl, downsample_scale, 20));
        
        vector<vector<int>> frame_b = {
            {0, 0, 0, 0},
            {0, 0, 1, 0},
            {0, 1, 1, 1},
            {0, 0, 1, 0}
        };
        n = frame_b.size();
        m = frame_b[0].size();
        vector<vector<vector<int>>> frame(n, vector<vector<int>>(m, vector<int>(3, 0)));
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                frame[i][j][0] = frame_b[i][j];
                frame[i][j][1] = frame_b[i][j];
                frame[i][j][2] = frame_b[i][j];
            }
        }
        
        vector<float> curr_det;
        detector->detect(frame, &curr_det);
        
        vector<vector<float>> responses = detector->prev_response();
        vector<vector<float>> top_k = detector->response_top_K(responses);
        std::cout << "det" << std::endl;
        util::Print(curr_det);
        std::cout << "response" << std::endl;
        util::Print(detector->prev_response());

        std::cout << "top_k" << std::endl;
        util::Print(top_k);
        
//        vector<vector<float>> ans = {{2, 2, 1},{1, 1, 0.516398}};
        assert(top_k[0][0] -2 < 10e-4);
        assert(top_k[0][1] -2 < 10e-4);
        assert(top_k[0][2] -1 < 10e-4);
        for (int i = 1; i < top_k.size(); i++) {
            std::cout << i << ":" <<  top_k[i-1][2] - top_k[i][2] << std::endl;
            assert(top_k[i-1][2] >= top_k[i][2]);

        }

        std::cout << "PASS" << std::endl;
    }

    void convolve_test() {
        
    }

    void update_template_test() {
        
    }

    void background_filter_test() {
        
    }

//};

int main() {
//    unique_ptr<DetectorTest> dt(new DetectorTest());
//    dt->detect_test();
//    dt->response_top_K_test();
    detect_test();
    response_top_K_test();
}
