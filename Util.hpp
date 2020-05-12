//
//  Util.hpp
//  
//
//  Created by 周吉夫 on 2020/5/3.
//
//  g++ Util_test.cpp Util.cpp -std=c++2a && ./a.out

#ifndef Util_hpp
#define Util_hpp

#include <stdio.h>

#include <cstdint>
#include <fstream>
#include <iostream>
#include <map>
#include <random>
#include <set>
#include <string>
#include <vector>

using std::set;
using std::map;
using std::string;
using std::unique_ptr;
using std::vector;

class Util {
public:

    /************************************************************/
    // File I/O

    static int get_filenames(const string& base_path, vector<string>* file_names);
    
    static bool dir_exists(const string& path);
    
    static bool file_exists(const string& path);
    
    static bool mk_dir(const string& path);
    
    static bool is_int(const string& arg);
    
    
    /************************************************************/
    // Visualizations

    static void overlay(const vector<vector<float>>& A, int v_a,
                        const vector<vector<float>>& B, int v_b,
                        int scale,
                        vector<vector<vector<int>>>* frame);
    
    
    /************************************************************/
    // Geometry
    
    static void contours(const vector<vector<int>>& mask, int radius, vector<vector<vector<int>>>* contours, vector<vector<int>>* contours_mask);
    
    static void sort_base_on_weight(vector<vector<int>>* v, const vector<float>& weghts);
    static float Angle(const vector<int>& A, const vector<int>& B);
    
    // Fits polygon contour given a set of 2D points
    static vector<vector<int>> polygon_fit_2d(const vector<vector<int>>& pos);
    
    // Checks if a base 2D point (x,y) is in polygon
    static bool in_polygon(const vector<int>& base, const vector<vector<int>>& polygon);
    
    // island_marks subroutine -
    // Recursively marks location surrounding (i,j) with index count
    static bool island(const vector<vector<int>>& mask, vector<vector<int>>* island_marks, int i, int j, int count);
    // Marks islands in a 2D mask from 0,1,...,num_islands
    // (an island is connected positions with value >= 0)
    static vector<vector<int>> island_marks(const vector<vector<int>>& mask);
    
    // islands_by_indices subroutine -
    // Recursively finds positions of island (surrouding i, j) with index count
    static void islands_by_indices(const vector<vector<int>>& islands,  vector<vector<bool>>* visited, int i, int j, int count, vector<vector<int>>* pos);
    // Retrieves a list of islands (each island is a list of 2D points)
    static vector<vector<vector<int>>> islands_by_indices(const vector<vector<int>>& islands);
    // Finds all sets of islands in the mask (an island has value >= 0) where each set consists of islands within radius distance to each other
    static vector<vector<vector<int>>> islands(const vector<vector<int>>& mask, int radius);


private:
};

// TODO(jifu) replaces
typedef vector<vector<float>> Mat2;
typedef vector<vector<vector<float>>> Mat3;

namespace util {

    /************************************************************/
    // vector operation
    
    template <class T, class S>
    T Norm_L2(vector<S> A) {
        T res = 0;
        int n = A.size();
        for (int i = 0; i < n; i++) {
            res += (T)A[i] * (T)A[i];
        }
        return std::sqrt((T)res);
    }
    
    template <class T>
    T Dot(vector<T> A, vector<T> B) {
        int n = A.size();
        assert(n == B.size());
        T res = 0;
        for (int i = 0; i < n; i++) {
            res += A[i] * B[i];
        }
        return res;
    }
    
    template <class T>
    vector<T> ColMean(const vector<vector<T>>& in) {
        int n = in.size();
        int m = in[0].size();
        vector<T> res(m, 0);
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                res[j] += in[i][j];
            }
        }
        for (int j = 0; j < m; j++) {
            res[j] /= n;
        }
        return res;
    }
    
    
    /************************************************************/
    // File I/O
    // TODO(jifu) int -> uchar

    template <class T>
    void csv2vec(const string& fname, vector<vector<vector<T>>>* data) {
        std::cout << "Read image =======" << std::endl << fname << std::endl;
        std::ifstream file;
        try {
            file.open(fname);
        }
        catch (std::ios_base::failure& e) {
            std::cerr << e.what() << '\n';
        }
        string desc;
        string da;
        getline(file, desc, file.widen('\n'));
        getline(file, da, file.widen('\n'));
        
        vector<int> shapes;
        std::string delimiter = ",";
        assert(!desc.empty());
        for (int i = 0; i < desc.length();) {
            string curr;
            int j = i;
            for (; j < desc.length() && desc[j] != ','; j++) {
                curr += desc[j];
            }
            shapes.push_back(std::stoi(curr));
            i = j + 1;
        }
        assert(3 == shapes.size());
        int n = shapes[0];
        int m = shapes[1];
        int k = shapes[2];
        
        vector<int> data_v;
        for (int i = 0; i < da.length();) {
            string curr;
            int j = i;
            for (; j < da.length() && da[j] != ','; j++) {
                curr += da[j];
            }
            data_v.push_back(std::stoi(curr));
            i = j + 1;
        }
        assert(data_v.size() == n*m*k);
        
        *data = vector<vector<vector<T>>>(n, vector<vector<T>>(m, vector<T>(k, 0)));
        int curr = 0;
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                for (int c = 0; c < k; c++) {
                    curr++;
                    (*data)[i][j][c] = (T)(data_v[curr]);
                }
            }
        }
    }
    
    template <class T>
    void vec2csv(vector<vector<vector<T>>> data, string fname) {
        int n = data.size();
        assert(n > 0);
        int m = data[0].size();
        assert(m > 0);
        int k = data[0][0].size();
        assert(k > 0);
        std::ofstream file;
        file.open(fname);
        assert(file.is_open());
        
        file << n << "," << m << "," << k << "\n";
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                file << data[i][j][0] << ",";
                file << data[i][j][1] << ",";
                if (i == n-1 && j == m-1) {
                    file << data[i][j][2] << "\n";
                } else {
                    file << data[i][j][2] << ",";
                }
            }
        }
    }
    
    template <class T>
    void vec2csv(vector<vector<T>> data, string fname) {
        int n = data.size();
        int m = data[0].size();
        std::ofstream file;
        file.open(fname);
        assert(file.is_open());
        file << n << "," << m  << "\n";
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                T v = data[i][j];
                if (j == m - 1 && i == n -1) {
                    file << v << "\n";
                } else {
                    file << v << ",";
                }
            }
        }
    }


    /************************************************************/
    // Image (3D matrix) operators

    // Flattens im into a vector, concating each channel
    template <class T>
    void flatten(vector<vector<vector<T>>> im, vector<T>* flattened) {
        flattened->clear();
        int n = im.size();
        assert(n > 0);
        int m = im[0].size();
        assert(m > 0);
        int channeles = im[0][0].size();
        for (int k = 0; k < channeles; k++) {
            for (int i = 0; i < n; i++) {
                for (int j = 0; j < m; j++) {
                    flattened->push_back(im[i][j][k]);
                }
            }
        }
    }
    
    // Flattens the window [i, j, i+n_t, j+m_t] within im, concating each channel
    template <class T>
    void flatten(vector<vector<vector<T>>> im,
                 int i, int j, int n_t, int m_t,
                 vector<T>* flattened) {
        // TODO(jifu) speed up by flatten everything first into continuous data array and op on it
        assert(i >= 0 && i + n_t <= im.size());
        assert(j >= 0 && j + m_t <= im[0].size());
        flattened->clear();
        int channeles = im[0][0].size();
        for (int k = 0; k < channeles; k++) {
            for (int ii = i; ii < i + n_t; ii++) {
                for (int jj = j; jj < j + m_t; jj++) {
                    flattened->push_back(im[ii][jj][k]);
                }
            }
        }
    }

    // Downscales a 2D matrix by a factor of scale
    template <class T>
    void downscale(vector<vector<T>> in, int scale, vector<vector<T>>* out) {
        if (scale == 1) {
            *out = in;
            return;
        }
        int n = in.size();
        assert(n > 0);
        int m = in[0].size();
        assert(m > 0);
        int out_n = std::ceil((float)n / scale);
        int out_m = std::ceil((float)m / scale);
        assert(out_n > 0);
        assert(out_n > 0);
        
        *out = vector<vector<T>>(out_n, vector<T>(out_m, 0));
        int out_i = 0;
        int v_debug = 0;
        for (int i = 0; i < n; i += scale) {
            int out_j = 0;
            for (int j = 0; j < m; j += scale) {
                T v = 0;
                int mult = 0;
                for (int ii = i; ii < i + scale && ii < in.size(); ii++) {
                    for (int jj = j; jj < j + scale && jj < in[ii].size(); jj++) {
                        v += in[ii][jj];
                        mult++;
                    }
                }
                v /= mult;
                (*out)[out_i][out_j] = v;
                out_j++;
            }
            out_i++;
        }
    }

    // Downscales a 3D matrix by a factor of scale
    template <class T>
    void downscale(vector<vector<vector<T>>> in, int scale, vector<vector<vector<T>>>* out) {
        int n = in.size();
        int m = in[0].size();
        int channels = in[0][0].size();
        int out_n = std::ceil((float)n / scale);
        int out_m = std::ceil((float)m / scale);
        *out = vector<vector<vector<T>>>(out_n, vector<vector<T>>(out_m, vector<T>(channels, 0)));
        
        for (int k = 0; k < channels; k++) {
            vector<vector<T>> in_k = vector<vector<T>>(n, vector<T>(m, 0));
            vector<vector<T>> out_k;
            for (int i = 0; i < n; i++) {
                for (int j = 0; j < m; j++) {
                    in_k[i][j] = in[i][j][k];
                }
            }
            downscale(in_k, scale, &out_k);
            for (int i = 0; i < out_n; i++) {
                for (int j = 0; j < out_m; j++) {
                    (*out)[i][j][k] = out_k[i][j];
                }
            }
        }
    }
    
    // Pads a 2D matrix of size n along first dimension and m along second dimension
    template <class T>
    void pad(vector<vector<T>> in, int n, int m, vector<vector<T>>* out) {
        int n0 = in.size();
        int m0 = in[0].size();
        *out = vector<vector<T>>(n0 + (2 * n), vector<T>(m0 + (2 * m), 0));
        for (int i = n; i < n + n0; i++) {
            for (int j = m; j < m + m0; j++) {
                (*out)[i][j] = in[i - n][j - m];
            }
        }
    }
    
    // Pads a 2D matrix of size template
    template <class T>
    void pad(vector<vector<vector<T>>> vec_img1, vector<vector<vector<T>>> vec_template, vector<vector<vector<T>>>* out) {
        vector<vector<vector<int>>> im_padded;
        vector<vector<int>> im_padded_k;
        int n = vec_img1.size();
        int m = vec_img1[0].size();
        int n_t = vec_template.size();
        int m_t = vec_template[0].size();
        int channels = 3;
        vector<vector<int>> im_k = vector<vector<int>>(n, vector<int>(m, 0));
        for (int k = 0; k < channels; k++) {
            for (int i = 0; i < n; i++) {
                for (int j = 0; j < m; j++) {
                    im_k[i][j] = vec_img1[i][j][k];
                }
            }
            util::pad(im_k, n_t - 1, m_t - 1, &im_padded_k);
            if (im_padded.empty()) {
                im_padded = vector<vector<vector<int>>>(im_padded_k.size(), vector<vector<int>>(im_padded_k[0].size(), vector<int>(channels, 0)));
            }
            for (int i = 0; i < im_padded_k.size(); i++) {
                for (int j = 0; j < im_padded_k[0].size(); j++) {
                    im_padded[i][j][k] = im_padded_k[i][j];
                }
            }
        }
        *out = im_padded;
    }

    // Convolves two vectors (normalized)
    template <class T, class S>
    void convolve(vector<T> input, vector<T> templ, vector<S>* response) {
        int n = input.size();
        int m = templ.size();
        response->clear();
        assert(n >= m);
        float norm_templ = util::Norm_L2<S,T>(templ);
        for (int i = 0; i <= n - m; i++) {
            vector<T> window(input.begin() + i, input.begin() + i + m);
            S prod = util::Dot<T>(templ, window);
            S norm_window = util::Norm_L2<S,T>(window);
            S res = 0;
            if (norm_window > 0 && norm_templ > 0) {
                res = prod / norm_templ / norm_window;
            }
            response->push_back(res);
        }
    }
    
    // Convolves 3D matrix (an RGB image with an RGB template)
    template <class T, class S>
    void convolve(vector<vector<vector<T>>> im,
                  vector<vector<vector<T>>> templ,
                  vector<vector<S>>* response) {
        int n = im.size();
        assert(n > 0);
        int m = im[0].size();
        assert(m > 0);
        int channeles = im[0][0].size();
        
        int n_t = templ.size();
        assert(n_t > 0);
        int m_t = templ[0].size();
        assert(m_t > 0);
        int c_t = templ[0][0].size();
        
        assert(n >= n_t);
        assert(m >= m_t);
        vector<T> templ_flatten;
        flatten<T>(templ, &templ_flatten);
        vector<vector<S>> response_raw = vector<vector<S>>(n - n_t + 1, vector<S>(m - m_t + 1, 0));
        for (int i = 0; i <= n - n_t; i++) {
            for (int j = 0; j <= m - m_t; j++) {
                vector<T> flattened;
                util::flatten(im, i, j, n_t, m_t, &flattened);
                vector<S> response_ij;
                convolve<T,S>(flattened, templ_flatten, &response_ij);
                
                response_raw[i][j] = response_ij[0];
            }
        }
        
        // shift response to the center of the template from upperleft corner
        int n_raw = response_raw.size();
        int m_raw = response_raw[0].size();
        vector<vector<S>> res = vector<vector<S>>(n - 2*(n_t - 1), vector<S>(m - 2*(m_t - 1), 0));
        int res_n = res.size();
        int res_m = res[0].size();
        
        int down = n_t / 2;
        int right = m_t / 2;
        for (int i = 0; i < res_n; i++) {
            for (int j = 0; j < res_m; j++) {
                res[i][j] =  response_raw[i + down][j + right];
            }
        }
        *response = res;        
    }

    // Absolute L2 distance between two RGB images pixelwise
    template <class T, class S>
    vector<vector<S>> AbsDiff_L2(const vector<vector<vector<T>>>& curr,
                                 const vector<vector<vector<T>>>& prev) {
        int n = curr.size();
        assert(n > 0);
        int m = curr[0].size();
        assert(m > 0);
        int k = curr[0][0].size();
        assert(k > 0);
        assert(n == prev.size());
        assert(m == prev[0].size());
        assert(k == prev[0][0].size());
        vector<vector<S>> diff(n, vector<S>(m, 0));
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                S d = 0;
                for (int c = 0; c < k; c++) {
                    d += S(curr[i][j][c] - prev[i][j][c]) / 255;
                }
                diff[i][j] = std::abs(d) / k;
            }
        }
        return diff;
    }
    
    // Thresholding a 2D matrix (above which 1 and otherwise 0)
    template <class T>
    void Threshold(vector<vector<T>>* diff, T threshold) {
        int n = diff->size();
        int m = (*diff)[0].size();
        for (int ii = 0; ii < n; ii++) {
            for (int jj = 0; jj < m; jj++) {
                if ((*diff)[ii][jj] >= threshold) {
                    (*diff)[ii][jj] = 1;
                } else {
                    (*diff)[ii][jj] = 0;
                }
            }
        }
    }

    // Average value in window of size h x w around position (c_i, c_j)
    template <class T>
    T BboxAvg(const vector<vector<T>>& response, int c_i, int c_j, int h, int w) {
        int i0 = c_i - h / 2;
        int i1 = i0 + h;
        int j0 = c_j - w / 2;
        int j1 = j0 + w;
        T response_avg = 0;
        if (i0 < 0 || i1 > response.size() || j0 < 0 || j1 > response[0].size()) {
            return -1;
        }
        for (int i = i0; i < i1; i++) {
            for (int j = j0; j < j1; j++) {
                response_avg += response[i][j];
            }
        }
        response_avg /= T(h) * T(w);
        return response_avg;
    }
    
    // Gradient of a gray image along x
    template <class T>
    void gradient_x(vector<vector<T>>& img, vector<vector<T>>* grad_x) {
        *grad_x = img;
        int n = img.size();
        int m = img[0].size();
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                T r = 0;
                T l = 0;
                if (j - 1 >= 0) {
                    l = img[i][j - 1];
                }
                if (j + 1 < m) {
                    r = img[i][j + 1];
                }
                (*grad_x)[i][j] = r - l;
            }
        }
    }
    
    // Gradient of a gray image along y
    template <class T>
    void gradient_y(vector<vector<T>>& img, vector<vector<T>>* grad_y) {
        *grad_y = img;
        int n = img.size();
        int m = img[0].size();
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                T u = 0;
                T d = 0;
                if (i - 1 >= 0) {
                    u = img[i - 1][j];
                }
                if (i + 1 < n) {
                    d = img[i + 1][j];
                }
                (*grad_y)[i][j] = d - u;
            }
        }
    
    }
    
    // Gradient of two consecutive frames of images (along time)
    template <class T>
    void gradient_t(vector<vector<T>>& prev,
                    vector<vector<T>>& img,
                    vector<vector<T>>* grad_t) {
        *grad_t = img;
        int n = img.size();
        int m = img[0].size();
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                (*grad_t)[i][j] = (img[i][j] - prev[i][j]);
            }
        }
    }
    
    /************************************************************/
    // Debugging, Logging & Testing
    
    template <class T>
    void Print(vector<T> vec) {
        int len = vec.size();
        for (int i = 0; i < len; i++) {
            std::cout << vec[i] << " ";
        }
        std::cout << std::endl;
    }
    
    template <class T>
    void Print(vector<vector<T>> vec) {
        int len = vec.size();
        for (int i = 0; i < len; i++) {
            int wid = vec[i].size();
            for (int j = 0; j < wid; j++) {
                std::cout << vec[i][j] << " ";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }
    
    // Histogram of a 2D matrix
    template <class T>
    void Hist(const vector<vector<T>>& in) {
        std::cout << "hist-----" << std::endl;
        vector<int> hist(11, 0);
        int n = in.size();
        int m = in[0].size();
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                T v = in[i][j];
                if (isnan(v) || v > 255 || v < 0) {
                    hist[10]++;
                } else {
                    hist[int(v / 25)]++;
                }
            }
        }
        Print(hist);
        std::cout << "---------" << std::endl;
    }

    // Histogram of a 3D matrix
    template <class T>
    void Hist(const vector<vector<vector<T>>>& in) {
        std::cout << "hist-----" << std::endl;
        vector<vector<int>> hist = vector<vector<int>>(3, vector<int>(11, 0));
        int n = in.size();
        int m = in[0].size();
        assert(in[0][0].size() == 3);
        for (int cc = 0; cc < 3; cc++) {
            for (int i = 0; i < n; i++) {
                for (int j = 0; j < m; j++) {
                    T v = in[i][j][cc];
                    if (isnan(v) || v > 255 || v < 0) {
                        hist[cc][10]++;
                    } else {
                        hist[cc][int(v / 25)]++;
                    }
                }
                
            }
            Print(hist[cc]);
        }
        std::cout << "---------" << std::endl;
    }
    
    // Testing equality between two vectors
    template <class T>
    bool TestEQ(vector<T> first, vector<T> second) {
        bool eq = first.size() == second.size();
        if (!eq) {
            return false;
        }
        int len = first.size();
        for (int i = 0; i < len; i++) {
            eq = eq && first[i] == second[i];
        }
        return eq;
    }
    
    // Testing equality between two 2D matrices
    template <class T>
    bool TestEQ(vector<vector<T>> first, vector<vector<T>> second) {
        bool eq = first.size() == second.size();
        int len = first.size();
        double eps = 10e-6;
        for (int i = 0; i < len; i++) {
            int len_i = first[i].size();
            eq = eq && int(first[i].size()) == int(second[i].size());
            if (!eq) {
                std::cout << first[i].size() << "!size=" << second[i].size() << std::endl;
            }
            for (int j = 0; j < len_i; j++) {
                //            eq = eq && first[i][j] == second[i][j];
                eq = eq && first[i][j] - second[i][j] < eps;
                if (!eq) {
                    std::cout << first[i][j] << "!=" << second[i][j] << std::endl;
                }
            }
        }
        return eq;
    }
    
    // Testing equality between two 3D matrices
    template <class T>
    bool TestEQ(vector<vector<vector<T>>> first, vector<vector<vector<T>>> second) {
        bool eq = first.size() == second.size();
        int len = first.size();
        double eps = 10e-6;
        for (int i = 0; i < len; i++) {
            int len_i = first[i].size();
            eq = eq && int(first[i].size()) == int(second[i].size());
            if (!eq) {
                std::cout << first[i].size() << "!size=" << second[i].size() << std::endl;
            }
            for (int j = 0; j < len_i; j++) {
                int len_i_j = first[i][j].size();
                eq = eq && int(first[i][j].size()) == int(second[i][j].size());
                if (!eq) {
                    std::cout << first[i][j].size() << "!size=" << second[i][j].size() << std::endl;
                }
                for (int k = 0; k < len_i_j; k++) {
                    eq = eq && first[i][j][k] - second[i][j][k] < eps;
                    if (!eq) {
                        std::cout << first[i][j][k] << "!=" << second[i][j][k] << std::endl;
                    }
                }
            }
        }
        return eq;
    }
    
    // Testing equality ignoring order
    template <class T>
    bool TestEQ_unordered(vector<vector<vector<T>>> first, vector<vector<vector<T>>> second) {
        bool eq = first.size() == second.size();
        int len = first.size();
        double eps = 10e-6;
        set<T> set_a;
        set<T> set_b;
        for (int i = 0; i < len; i++) {
            int len_i = first[i].size();
            eq = eq && int(first[i].size()) == int(second[i].size());
            if (!eq) {
                std::cout << first[i].size() << "!size=" << second[i].size() << std::endl;
            }
            for (int j = 0; j < len_i; j++) {
                //            eq = eq && first[i][j] == second[i][j];
                int len_i_j = first[i][j].size();
                eq = eq && int(first[i][j].size()) == int(second[i][j].size());
                if (!eq) {
                    std::cout << first[i][j].size() << "!size=" << second[i][j].size() << std::endl;
                }
                for (int k = 0; k < len_i_j; k++) {
                    set_a.emplace(first[i][j][k]);
                    set_b.emplace(second[i][j][k]);
                }
            }
        }
        for (auto k : set_a) {
            if (set_b.find(k) == set_b.end()) {
                return false;
            }
        }
        return eq;
    }
    
    /**********************************************/
    // Visualization
    
    // Draws two sets of line points on img
    template <class T, class S>
    void OverlayLines(const vector<vector<S>>& points, T val, S scale,
                      vector<vector<vector<T>>>* overlay) {
        int n = overlay->size();
        assert(n > 0);
        int m = (*overlay)[0].size();
        assert(m > 0);
        int k = (*overlay)[0][0].size();
        int num_pts = points.size();
        int dim = points[0].size();
        if (num_pts == 0) return;
        assert(dim == 2); // 2D im
        for (int i = 0; i < num_pts; i++) {
            int x = (int)points[i][0];
            int y = (int)points[i][1];
            
            int x0 = x - scale / 2;
            int x1 = x + scale / 2;
            int y0 = y - scale / 2;
            int y1 = y + scale / 2;
            for (int ii = x0; ii < x1; ii++) {
                for (int jj = y0; jj < y1; jj++) {
                    for (int c = 0; c < k; c++) {
                        (*overlay)[ii][jj][c] = val;
                    }
                }
            }
        }
    }
    
    // Draws bounding box of hxw at (i,j) on img
    template <class T>
    void OverlayBbox(int i, int j, int h, int w, const vector<vector<vector<T>>>& img, vector<vector<vector<T>>>* boxed) {
        int n = img.size();
        assert(n > 0);
        int m = img[0].size();
        assert(m > 0);
        int k = img[0][0].size();
        *boxed = img;
        int i0 = i - h/2;
        int j0 = j - w/2;
        int i1 = i0 + h;
        int j1 = j0 + w;
        assert(i0 >= 0 && i1 <= n);
        assert(j0 >= 0 && j1 <= m);
        for (int c = 0; c < k; c++) {
            for (int ii = i0; ii < i1; ii++) {
                for (int jj = j0; jj < j1; jj++) {
                    if (ii == i0 || jj == j0 || ii == i1 - 1 || jj == j1 - 1) {
                        (*boxed)[ii][jj][c] = 0;
                    }
                }
            }
        }
    }

    /**********************************************/
    // Matrix Operations
    
    template <class T>
    vector<vector<T>> MatAdd(vector<vector<T>> A, vector<vector<T>> B) {
        int m = A.size();
        int n = A[0].size();
        assert(B.size() == m);
        assert(B[0].size() == n);
        vector<vector<T>> C = vector<vector<T>>(m, vector<T>(n, 0));
        for (int i = 0; i < m; i++) {
            for (int j = 0; j < n; j++) {
                C[i][j] = A[i][j] + B[i][j];
            }
        }
        return C;
    }

    template <class T>
    vector<T> MatDiag(vector<vector<T>> A) {
        // TODO(jifu) opt / mt
        int m = A.size();
        vector<T> d = vector<T>(m, 0);
        for (int i = 0; i < m; i++) {
            d[i] = A[i][i];
        }
        return d;
    }
    
    template <class T, class S>
    vector<vector<S>> MatAnd(vector<vector<T>> A, vector<vector<S>> B) {
        // TODO(jifu) opt / mt
        int n = A.size();
        int m = A[0].size();
        assert(B.size() == n);
        assert(B[0].size() == m);
        vector<vector<S>> C(n, vector<S>(m, 0));
        
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                if (A[i][j] == 0) {
                    C[i][j] = 0;
                } else {
                    C[i][j] = B[i][j];
                }
            }
        }
        return C;
    }
    
    template <class T>
    vector<vector<T>> MatMul(vector<vector<T>> A, vector<vector<T>> B) {
        // TODO(jifu) opt / mt
        int m = A.size();
        int l = A[0].size();
        assert(B.size() == l);
        int n = B[0].size();
        vector<vector<T>> C = vector<vector<T>>(m, vector<T>(n, 0));
        
        for (int i = 0; i < m; i++) {
            for (int j = 0; j < n; j++) {
                T v_ij = 0;
                for (int k = 0; k < l; k++) {
                    v_ij += A[i][k] * B[k][j];
                }
                C[i][j] = v_ij;
            }
        }
        return C;
    }
    
    template <class T> vector<vector<T>> MatMul(T mult, vector<vector<T>> A) {
        // TODO(jifu) opt / mt
        int m = A.size();
        int n = A[0].size();
        vector<vector<T>> C = vector<vector<T>>(m, vector<T>(n, 0));
        
        for (int i = 0; i < m; i++) {
            for (int j = 0; j < n; j++) {
                C[i][j] = A[i][j] * mult;
            }
        }
        return C;
    }

    // Creates an identity square matrix of size n x n
    template <class T> vector<vector<T>> MatId(int n) {
        vector<vector<T>> out = vector<vector<T>>(n, vector<T>(n, 0));
        for (int i = 0; i < n; i++) {
            out[i][i] = 1;
        }
        return out;
    }

    // Computes determinant of the given matrix
    template <class T>
    T MatDet(vector<vector<T>> M) {
        // TODO(jifu) opt / mt
        int m = M.size();
        if (m == 2) {
            T a = M[0][0];
            T b = M[0][1];
            T c = M[1][0];
            T d = M[1][1];
            return a * d - b * c;
        } else if (m == 3) {
            T a = M[0][0];
            T b = M[0][1];
            T c = M[0][2];
            T d = M[1][0];
            T e = M[1][1];
            T f = M[1][2];
            T g = M[2][0];
            T h = M[2][1];
            T i = M[2][2];
            T A = e * i - f * h;
            T B = -( d * i - f * g );
            T C = d * h - e * g;
            return a * A + b * B + c * C;
        } else {
            std::cout << "TODO(jifu) mat det of dim " << m << std::endl;
        }
        return INT_MAX;
    }
    
    // Computes inverse of the given matrix
    template <class T>
    vector<vector<T>> MatInv(vector<vector<T>> M) {
        int n = M.size();
        if (n == 3) {
            // 3x3
            T a = M[0][0];
            T b = M[0][1];
            T c = M[0][2];
            T d = M[1][0];
            T e = M[1][1];
            T f = M[1][2];
            T g = M[2][0];
            T h = M[2][1];
            T i = M[2][2];
            T A = e * i - f * h;
            T B = -( d * i - f * g );
            T C = d * h - e * g;
            T D = -( b * i - c * h );
            T E = a * i - c * g;
            T F = -( a * h - b * g );
            T G = b * f - c * e;
            T H = -( a * f - c * d );
            T I = a * e - b * d;
            vector<vector<T>> out = {
                {A, D, G},
                {B, E, H},
                {C, F, I}
            };
            T det = MatDet(M);
            return MatMul(1 / det, out);
        } else if (n == 2) {
            // 2x2
            T a = M[0][0];
            T b = M[0][1];
            T c = M[1][0];
            T d = M[1][1];
            vector<vector<T>> C = {
                {d, -b},
                {-c, a}
            };
            T det = MatDet(M);
            return MatMul(1 / det, C);
        } else {
            std::cout << "TODO(jifu) mat inv of dim " << n << std::endl;
            // general: lu Q R
        }
        return {{}};
    }
    
    // Computes transposes of the given matrix
    template <class T>
    vector<vector<T>> MatTranspose(vector<vector<T>> A) {
        int m = A.size();
        int n = A[0].size();
        vector<vector<T>> C = vector<vector<T>>(n, vector<T>(m, 0));
        for (int i = 0; i < m; i++) {
            for (int j = 0; j < n; j++) {
                C[j][i] = A[i][j];
            }
        }
        return C;
    }


    /**********************************************/
    // Random generator

    template <class T> void gaussian_rand(T mu, T sig, int num, vector<T>* data) {
        data->clear();
    
        // ref: http://www.cplusplus.com/reference/random/normal_distribution/
        const int nrolls=std::max(num, 10000);  // number of experiments
        const int nstars=std::max(100, num / 100);    // maximum number of stars to distribute
    
        std::default_random_engine generator;
        std::normal_distribution<T> distribution(mu, sig);
    
        T lower_bound = mu - (3 * sig);
        T higher_bound = mu + (3 * sig);
        T step = (higher_bound - lower_bound) / 10;
    
        int p[10]={};
    
        for (int i=0; i<nrolls; ++i) {
            T number = distribution(generator);
            data->push_back(number);
            int index = (number - lower_bound) / step;
            if ((number>=lower_bound)&&(number<higher_bound)) ++p[index];
        }
    
        std::cout << "normal_distribution " << mu << "," << sig << std::endl;
    
        for (int i=0; i<10; ++i) {
            std::cout << i << "-" << (i+1) << ": ";
            std::cout << p[i] << ":";
            std::cout << std::string(p[i]*nstars/nrolls,'*') << std::endl;
        }
    }
}
#endif /* Util_hpp */
