//
//  Detector.cpp
//  
//
//  Created by 周吉夫 on 2020/5/7.
//

#include "Detector.hpp"

using util::AbsDiff_L2;
using util::BboxAvg;
using util::ColMean;
using util::OverlayBbox;
using util::Threshold;
using util::downscale;
using util::pad;

void Detection::detect(const vector<vector<vector<int>>>& frame, vector<float>* curr_det) {
    vector<vector<float>> response;
    vector<vector<vector<int>>> im_downsampled;
    convolve(frame, &im_downsampled, &response);

    // Filter stationary background
    vector<vector<float>> bg_mask;
    background_filter(im_downsampled, response, &bg_mask);

    // Detects (searches best detection, updates templates, post-process)
    int high_i;
    int high_j;
    vector<vector<vector<int>>> tmpl = tmpl_8_;
    vector<vector<float>> response_filtered;
    detect(im_downsampled, response, bg_mask, &tmpl, &response_filtered, &high_i, &high_j);
    
    // Overlays bbox
    vector<vector<vector<int>>> img_detect_box_overlay;
    util::OverlayBbox(high_i, high_j, tmpl.size(), (tmpl)[0].size(), im_downsampled, &img_detect_box_overlay);
    *curr_det = {
        (float)high_i,
        (float)high_j,
        (float)(tmpl.size()),
        (float)((tmpl)[0].size())
    };
    prev_img_ = im_downsampled;
    responses_.push_back(response);
    prev_fg_mask_ = bg_mask;
    prev_detect_bbox_ = img_detect_box_overlay;
    responses_filtered_.push_back(response_filtered);
}

void Detection::convolve(const vector<vector<vector<int>>>& frame,
                         vector<vector<vector<int>>>* frame_processed,
                         vector<vector<float>>* response) {
    // downsample
    downscale(frame, downsample_scale_, frame_processed);

    // pad
    vector<vector<vector<int>>> im_downsampled_padded;
    pad(*frame_processed, tmpl_8_, &im_downsampled_padded);
    
    // convolves with templates
    util::convolve(im_downsampled_padded, tmpl_8_, response);
}

void Detection::background_filter(const vector<vector<vector<int>>>& curr_img, const vector<vector<float>>& response, vector<vector<float>>* bg_mask) {
    if (prev_img_.empty()) return;
    // changes compared to previous frame (abs(curr - prev_img) / prev_img)
    vector<vector<float>> diff = AbsDiff_L2<int, float>(curr_img, prev_img_);
    float threshold = 0.1;
    Threshold<float>(&diff, threshold);
    // TODO(jifu) experiment directly filters response / decay response by diff
    //    util::Ones<float>(&diff, prev_box_i_, prev_box_j_, prev_box_h_*1.2, prev_box_w_*1.2);
    //    util::MatDotMul<float>(diff, response);
    *bg_mask = diff;
}

void Detection::detect(const vector<vector<vector<int>>>& im_downsampled,
                       const vector<vector<float>>& response,
                       const vector<vector<float>>& bg_mask,
                       vector<vector<vector<int>>>* tmpl_updated,
                       vector<vector<float>>* response_filter,
                       int* high_response_ci, int* high_response_cj) {
    // the mask around highest response (0.95) that is <= size of curr tmpl templ_3f_
    // 1. highest avg response mask of same size as curr tmpl
    // 2. if < prev response * 0.999, adjust mask size to achieve >= avg response
    int n_t = tmpl_updated->size();
    int m_t = (*tmpl_updated)[0].size();
    int n = response.size();
    int m = response[0].size();
    assert(n == im_downsampled.size());
    assert(m == im_downsampled[0].size());
    int highest_i = 0;
    int highest_j = 0;
    int highest_i_debug = 0;
    int highest_j_debug = 0;
    float highest_point_avg_score = 0;
    float highest_score_avg = 0;
    float highest_res_debug = 0;
    float highest_response_adjusted = 0;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            if (!bg_mask.empty() && !bg_mask[i][j]) continue;
            if (highest_res_debug < response[i][j]) {
                highest_res_debug = response[i][j];
                highest_i_debug = i;
                highest_j_debug = j;
                highest_point_avg_score = BboxAvg(response, i, j, n_t, m_t);
            }
            if (response[i][j] > 0.85) {
                float score = BboxAvg(response, i, j, n_t, m_t);
                if (score > highest_score_avg) {
                    highest_score_avg = score;
                    highest_i = i;
                    highest_j = j;
                }
            }
        }
    }

    // TODO(jifu) experiment refit (top_K_avg -> x,y, refit to find best bbox fit)
    //    std::cout << "try refit " << std::endl;
    //    int resize_pad = 2;
    //    int search_rad = std::max(n_t, m_t) / 1.5;
    //    float tmpl_decay = 0.85;
    //    //    float decay = 0.99;
    //    float highest_score_refit = 0;
    //    vector<vector<vector<int>>> new_template;
    
    
    // If not refit
    assert(highest_i > 0);
    // sets to highest AVERAGE point
    //    *high_response_ci = highest_i;
    //    *high_response_cj = highest_j;
    highest_prev_response_ = highest_score_avg;
    // sets to highest point
    //    *high_response_ci = highest_i_debug;
    //    *high_response_cj = highest_j_debug;
    //    highest_prev_response_ = highest_res_debug;
    // sets to highest K poitns
    vector<vector<float>> top_K_res = response_top_K(response);
    vector<float> center = ColMean(top_K_res);
    *high_response_ci = (int)round((center[0]));
    *high_response_cj = (int)round((center[1]));
    
    // Fits foreground objects with polygon contour
    if (!bg_mask.empty()) {
        vector<vector<vector<int>>> contours;
        vector<vector<int>> contours_mask;
        int radius = 1;
        n = bg_mask.size();
        assert(n > 0);
        m = bg_mask[0].size();
        vector<vector<int>> bg_mask_threshold(n, vector<int>(m, 0));
        vector<vector<float>> bg_mask_tmp = bg_mask;

        // Filters background/foreground mask and response
        util::Threshold<float>(&bg_mask_tmp, 0.001);
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                bg_mask_threshold[i][j] = int(bg_mask_tmp[i][j]);
            }
        }

        vector<vector<float>> response_threshold = response;
        for (int i = 0; i < response.size(); i++) {
            for (int j = 0; j < response[i].size(); j++) {
                if (response_threshold[i][j] < 0.8) response_threshold[i][j] = 0;
            }
        }

        // Contour & response
        Util::contours(bg_mask_threshold, radius, &contours, &contours_mask);
        *response_filter = util::MatAnd(contours_mask, response_threshold/*response*/);
        
        // Updates best bbox fit position based on filtered responses
        center = util::ColMean(response_top_K(*response_filter));
        *high_response_ci = (int)(center[0]);
        *high_response_cj = (int)(center[1]);
        prev_fg_contour_mask_.push_back(contours_mask);
    }
}


vector<vector<float>> Detection::response_top_K(const vector<vector<float>>& responses) {
    vector<vector<float>> v;
    map<float, vector<vector<float>>> hash_sort;
    
    int n = responses.size();
    int m = responses[0].size();
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            // TODO(jifu) verify 0 filtering
            if (responses[i][j] == 0) continue;
            hash_sort[responses[i][j]].push_back({(float)i, (float)j});
        }
    }
    for (auto kv : hash_sort) {
        for (int i = 0; i < kv.second.size(); i++) {
            vector<float> curr = kv.second[i];
            curr.push_back(kv.first);
            v.push_back(curr);
        }
    }
    for (int i = 0; i < v.size() / 2; i++) {
        vector<float> tmp = v[i];
        v[i] = v[v.size() - 1 - i];
        v[v.size() - 1 - i] = tmp;
    }
        vector<vector<float>> res;
        for (int i = 0; i < std::min(top_K_particles_, (int)v.size()); i++) {
            res.push_back(v[i]);
        }
        return res;
}

vector<vector<float>> Detection::response_top_K_trace(int T) {
    assert(T <= responses_.size());
    vector<vector<float>> trace;
    for (int i = 0; i < T; i++) {
        vector<vector<float>> particles = response_top_K(responses_[i]);
        vector<float> center = ColMean(particles);
        trace.push_back({center[0], center[1]});
    }
    return trace;
}

void Detection::OverlayBbox(const vector<float>& state) {
    ::OverlayBbox((int)round(state[0]), (int)round(state[1]), tmpl_8_.size(), (tmpl_8_)[0].size(), prev_img_, &prev_detect_bbox_);
}
