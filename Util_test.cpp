//  Util_test.cpp
//
// Tests Util functions:
//  g++ Util_test.cpp Util.cpp -std=c++2a && ./a.out
//
//
//
//  Created by 周吉夫 on 2020/5/3.
//

#include "Util.hpp"

#include <stdio.h>

#include <cassert>
#include <iostream>
#include <memory>

using std::string;
using std::unique_ptr;
using std::vector;
using util::TestEQ;

class UtilTest {
    
public:
    UtilTest() {

    }

    void Gradient_tTest() {
        std::cout << "Gradient_tTest" << std::endl;
        
        vector<vector<float>> im0 = {
            {-1, 0, 0},
            {5, -1, 0},
            {-1, 0, 0}
        };
        vector<vector<float>> im1 = {
            {0, -1, 0},
            {-1, 5, -1},
            {0, -1, 0}
        };
        vector<vector<float>> res = {
            {1, -1, 0},
            {-6, 6, -1},
            {1, -1, 0}
        };
        vector<vector<float>> grad_t;
        util::gradient_t<float>(im0, im1, &grad_t);
        for (int i = 0; i < grad_t.size(); i++) {
            for (int j = 0; j < grad_t[0].size(); j++) {
                assert(grad_t[i][j] == res[i][j]);
            }
        }
        std::cout << "PASS" << std::endl;
    }
    
    void Gradient_xTest() {
        std::cout << "Gradient_xTest" << std::endl;
        vector<vector<float>> im = {
            {0, -1, 0},
            {-1, 5, -1},
            {0, -1, 0}
        };
        vector<vector<float>> res = {
            {-1, 0, 1},
            {5, 0, -5},
            {-1, 0, 1}
        };
        vector<vector<float>> grad_x;
        util::gradient_x<float>(im, &grad_x);
        for (int i = 0; i < grad_x.size(); i++) {
            for (int j = 0; j < grad_x[0].size(); j++) {
                assert(grad_x[i][j] == res[i][j]);
            }
        }
        std::cout << "PASS" << std::endl;
        
    }
    
    void Gradient_yTest() {
        std::cout << "Gradient_yTest" << std::endl;
        
        vector<vector<float>> im = {
            {0, -1, 0},
            {-1, 5, -1},
            {0, -1, 0}
        };
        vector<vector<float>> res = {
            {-1, 5, -1},
            {0, 0, 0},
            {1, -5, 1}
        };
        
        vector<vector<float>> grad_y;
        util::gradient_y<float>(im, &grad_y);
        for (int i = 0; i < grad_y.size(); i++) {
            for (int j = 0; j < grad_y[0].size(); j++) {
                assert(grad_y[i][j] == res[i][j]);
            }
        }
        std::cout << "PASS" << std::endl;
    }



    void MatMulTest() {
        std::cout << "MatMulTest" << std::endl;
        vector<vector<double>> B = {
            {5},
            {6}
        };
        vector<vector<double>> A = {
            {1, 2},
            {3, 4}
        };
        vector<vector<double>> C = util::MatMul(A, B);
        vector<vector<double>> ans = {
            {17},
            {39}
        };

        assert(TestEQ<double>(ans, C));
        std::cout << "PASS" << std::endl;
    }
    
    void MatMulIITest() {
        std::cout << "MatMulIITest" << std::endl;
        vector<vector<double>> A = {
            {1, 2},
            {3, 4}
        };
        vector<vector<double>> C = util::MatMul(0.1, A);
        vector<vector<double>> ans = {
            {0.1, 0.2},
            {0.3, 0.4}
        };
        
        assert(TestEQ<double>(ans, C));
        std::cout << "PASS" << std::endl;
    }
    
    void MatAddTest() {
        std::cout << "MatAddTest" << std::endl;
        vector<vector<double>> B = {
            {5, 7},
            {6, 8}
        };
        vector<vector<double>> A = {
            {1, 2},
            {3, 4}
        };
        vector<vector<double>> C = util::MatAdd(A, B);
        vector<vector<double>> ans = {
            {6, 9},
            {9, 12}
        };
        
        assert(TestEQ<double>(ans, C));
        std::cout << "PASS" << std::endl;
    }
    
    void MatTransposeTest() {
        std::cout << "MatTransposeTest" << std::endl;
        vector<vector<double>> A = {
            {1, 2},
            {3, 4}
        };
        vector<vector<double>> C = util::MatTranspose(A);
        vector<vector<double>> ans = {
            {1, 3},
            {2, 4}
        };
        
        assert(TestEQ<double>(ans, C));
        std::cout << "PASS" << std::endl;
    }
    
    void MatInv3by3Test() {
        std::cout << "MatInv3by3Test" << std::endl;
        vector<vector<double>> A = {
            {3, 0, 2},
            {2, 0, -2},
            {0, 1, 1}
        };
        vector<vector<double>> C = util::MatInv(A);
        //Print(C);

        vector<vector<double>> ans = {
            {0.2, 0.2, 0},
            {-0.2, 0.3, 1},
            {0.2, -0.3, 0}
        };
        int n = A.size();
        vector<vector<double>> ans_I = util::MatMul(A, C);
        assert(TestEQ<double>(ans_I, util::MatId<double>(n)));
        assert(TestEQ<double>(ans, C));

        std::cout << "PASS" << std::endl;
    }
    
    void MatInv2by2Test() {
        std::cout << "MatInv2by2Test" << std::endl;
        vector<vector<double>> A = {
            {4, 7},
            {2, 6}
        };
        vector<vector<double>> C = util::MatInv<double>(A);
        vector<vector<double>> ans = {
            {0.6, -0.7},
            {-0.2, 0.4}
        };
        assert(TestEQ<double>(ans, C));
        
        vector<vector<double>> ans_I = util::MatMul(A, C);

        int n = A.size();
        assert(TestEQ<double>(ans_I, util::MatId<double>(n)));
        std::cout << "PASS" << std::endl;
    }
    
    void GaussianRandomTest() {
        std::cout << "GaussianRandomTest" << std::endl;
        vector<double> data;
        double mu = 5.0;
        double sig = 2.0;
        int num = 10000;
        util::gaussian_rand(mu, sig, num, &data);

        // 3 sig
        int count = 0;
        for (int i = 0; i < num; i++) {
            if (data[i] <= mu + 3 * sig && data[i] >= mu - 3 * sig) {
                count++;
            }
        }
        assert(double(count) / num > .99);
        
        count = 0;
        for (int i = 0; i < num; i++) {
            if (data[i] <= mu + 2 * sig && data[i] >= mu - 2 * sig) {
                count++;
            }
        }
        assert(double(count) / num > .95);

        count = 0;
        for (int i = 0; i < num; i++) {
            if (data[i] <= mu + 1 * sig && data[i] >= mu - 1 * sig) {
                count++;
            }
        }
        assert(double(count) / num > .68);

        std::cout << "PASS" << std::endl;
    }

    void DotTest() {
        std::cout << "DotTest" << std::endl;
        vector<int> A = {1, 2, 3};
        vector<int> B = {4, 5, 6};
        int res = 32;
        assert(res == util::Dot<int>(A, B));
        std::cout << "PASS" << std::endl;
    }

    void Norm_L2Test() {
        std::cout << "Norm_L2Test" << std::endl;
        vector<double> A = {1, 2, 3};
        double res = std::sqrt(14);
        assert(std::abs(res - util::Norm_L2<double>(A)) < 10e-6);
        std::cout << "PASS" << std::endl;
    }
        
    void PadTest() {
        std::cout << "PadTest" << std::endl;
        vector<vector<float>> in = {
            {1, 1},
            {1, 1},
        };
        vector<vector<float>> padded;
        util::pad<float>(in, 1, 1, &padded);
        vector<vector<float>> ans = {
            {0, 0, 0, 0},
            {0, 1, 1, 0},
            {0, 1, 1, 0},
            {0, 0, 0, 0},
        };
        assert(util::TestEQ<float>(ans, padded));
        std::cout << "PASS" << std::endl;
    }
        
    void DownscaleTest() {
        std::cout << "DownscaleTest" << std::endl;

        vector<vector<float>> in = {
            {1, 2, 3, 4},
            {5, 6, 7, 8},
            {9, 10, 11, 12},
            {13, 14, 15, 16}
        };
        vector<vector<float>> out;
        util::downscale(in, 2, &out);
        vector<vector<float>> res = {
            {14/4.0, 22/4.0},
            {46/4.0, 54/4.0}
        };
        assert(util::TestEQ<float>(res, out));
       
        int n = in.size();
        int m = in[0].size();
        vector<vector<vector<float>>> in3 = vector<vector<vector<float>>>(n, vector<vector<float>>(m, vector<float>(3, 0)));
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                in3[i][j][0] = in[i][j];
                in3[i][j][1] = in[i][j];
                in3[i][j][2] = in[i][j];
            }
        }
        vector<vector<vector<float>>> out3;
        util::downscale(in3, 2, &out3);
        for (int k = 0; k < 3; k++) {
            for (int i = 0; i < out3.size(); i++) {
                for (int j = 0; j < out3[0].size(); j++) {
                    out[i][j] = out3[i][j][k];
                }
            }
            assert(util::TestEQ<float>(res, out));
        }

        std::cout << "PASS" << std::endl;
    }

    /************************************************************/
    // Geometry

    void island_marksTest() {
        std::cout << "island_marksTest" << std::endl;
        vector<vector<int>> mask = {
            {0, 1, 0},
            {0, 0, 0,},
            {1, 1, 0}
        };
        vector<vector<int>> mark = Util::island_marks(mask);
        vector<vector<int>> ans = {
            {-1, 0, -1},
            {-1, -1, -1},
            {1, 1, -1}
        };
        assert(TestEQ<int>(ans, mask));
        std::cout << "PASS" << std::endl;
    }
    
    void islands_by_indicesTest() {
        std::cout << "islands_by_indicesTest" << std::endl;
        vector<vector<int>> islands = {
            {0, -1, -1},
            {-1, 1, -1,},
            {-1, 1, -1},
        };
        vector<vector<vector<int>>> indices = Util::islands_by_indices(islands);
        vector<vector<vector<int>>> ans = {
            {{0, 0}},
            {{1, 1}, {2, 1}}
        };
        assert(util::TestEQ<int>(ans, indices));

        islands = {
            {-1, 0, -1},
            {-1, -1, -1},
            {1, 1, -1}
        };
        indices = Util::islands_by_indices(islands);
        ans = {
            {{0, 1}},
            {{2, 0}, {2, 1}}
        };
        assert(util::TestEQ<int>(ans, indices));
        std::cout << "PASS" << std::endl;
    }
    
    
    void islandsTest() {
        std::cout << "islandsTest" << std::endl;
        vector<vector<int>> mask = {
            {1, 0, 0, 0},
            {0, 1, 0, 0},
            {0, 0, 0, 0,},
            {0, 0, 0, 0},
            {0, 0, 0, 1}
        };
        vector<vector<vector<int>>> islands = Util::islands(mask, 1);
        vector<vector<vector<int>>> ans = {
            {{0, 0}, {0, 1}, {0, 2}, {1, 0}, {1, 1}, {1, 2}, {2, 0}, {2, 1}, {2, 2},
             {3, 2}, {3, 3}, {4, 2}, {4, 3}}
        };
        util::Print(islands[0]);
        assert(util::TestEQ_unordered<int>(ans, islands));
        std::cout << "PASS" << std::endl;
    }
    
    
    void AngleTest() {
        std::cout << "AngleTest" << std::endl;

        vector<int> A = {0, 1};
        vector<int> B = {1, 1};
        assert(45 == Util::Angle(A, B));
        B = {0, 1};
        assert(0 == Util::Angle(A, B));
        B = {0, -1};
        assert(std::abs(180.0 - Util::Angle(A, B)) < 10e-3);
        B = {-1, 0};
        assert(std::abs(90.0 - Util::Angle(A, B)) < 10e-3);
        std::cout << "PASS" << std::endl;

    }
    void in_polygonTest() {
        std::cout << "in_polygonTest" << std::endl;
        
        // 0 0 1 0
        // 0 1 0 1
        // 0 1 0 1
        // 0 0 1 0
        
        vector<vector<int>> polygon = {
            {0, 2},
            {1, 1},
            {1, 3},
            {2, 1},
            {2, 3},
            {3, 2},
        };
        vector<vector<bool>> in_polygon_ans = {
            {0, 0, 1, 0},
            {0, 1, 1, 1},
            {0, 1, 1, 1},
            {0, 0, 1, 0}
        };
        vector<vector<bool>> in_polygon = {
            {0, 0, 1, 0},
            {0, 1, 1, 1},
            {0, 1, 1, 1},
            {0, 0, 1, 0}
        };
        int n = in_polygon.size();
        int m = in_polygon[0].size();
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                bool in = Util::in_polygon({i, j}, polygon);
                assert(in == in_polygon[i][j]);
            }
        }
        std::cout << "PASS" << std::endl;
        
    }
    
    void polygon_fit_2dTest() {
        std::cout << "polygon_fit_2dTest" << std::endl;
        
        // 0 0 1 0
        // 0 1 1 0
        // 0 0 1 1
        // 0 0 1 0
        
        vector<vector<int>> pos = {
            {0, 2},
            {1, 1},
            {1, 2},
            {2, 2},
            {2, 3},
            {3, 2}
        };
        vector<vector<int>> polygon = Util::polygon_fit_2d(pos);
        vector<vector<int>> ans = {
            {0, 2},
            {1, 1},
            {1, 2},
            {2, 2},
            {3, 2},
            {2, 3}
        };
        std::cout << "polygon:" << std::endl;
        util::Print(polygon);
        assert(util::TestEQ<int>(ans, polygon));

        std::cout << "PASS" << std::endl;
        
    }
    
    void sort_base_on_weightTest() {
        std::cout << "sort_base_on_weightTest" << std::endl;
        
        vector<vector<int>> v = {
            {0, 0},
            {1, 1},
            {2, 2}
        };
        vector<float> weights = {
            2,
            1,
            3
        };
        
        Util::sort_base_on_weight(&v, weights);
        std::cout << "v" << std::endl;
        util::Print(v);
        vector<vector<int>> ans = {
            {1, 1},
            {0, 0},
            {2, 2}
        };
        assert(util::TestEQ<int>(ans, v));

        std::cout << "PASS" << std::endl;
        
    }
    void contoursTest() {
        std::cout << "contoursTest" << std::endl;

        vector<vector<int>> mask = {
            {0, 1, 0, 0, 0},
            {1, 0, 0, 1, 0},
            {0, 1, 0, 1, 0},
            {0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0},
            {0, 0, 1, 0, 0}
        };
        
        vector<vector<vector<int>>> contours_ans = {
            {{0, 1}, {1, 0}, {2, 1}, {2, 3}, {1, 3}},
            {{5, 2}},
        };

        vector<vector<int>> contours_mask_ans = {
            {1, 1, 1, 1, 1},
            {1, 1, 1, 1, 1},
            {1, 1, 1, 1, 1},
            {1, 1, 1, 1, 1},
            {0, 1, 1, 1, 0},
            {0, 1, 1, 1, 0}
        };
        vector<vector<vector<int>>> contours;
        vector<vector<int>> contours_mask;
        Util::contours(mask, 1, &contours, &contours_mask);
        std::cout << "contours_mask" << std::endl;
        util::Print(contours_mask);
        assert(util::TestEQ<int>(contours_mask, contours_mask_ans));

        std::cout << "PASS" << std::endl;

    }

 

};


int main() {
    unique_ptr<UtilTest> util_test(new UtilTest);
    util_test->MatMulTest();
    util_test->MatMulIITest();
    util_test->MatAddTest();
    util_test->MatInv2by2Test();
    util_test->MatInv3by3Test();
    util_test->MatTransposeTest();
    util_test->GaussianRandomTest();
    util_test->DotTest();
    util_test->Norm_L2Test();
    
    util_test->Gradient_xTest();
    util_test->Gradient_yTest();
    util_test->Gradient_tTest();
    
    util_test->PadTest();
    util_test->DownscaleTest();
    
    util_test->island_marksTest();
    util_test->islands_by_indicesTest();
    util_test->islandsTest();
    

    util_test->AngleTest();
    
    util_test->sort_base_on_weightTest();
    util_test->in_polygonTest();
    util_test->polygon_fit_2dTest();
    util_test->contoursTest();

    
    return 0;
}
