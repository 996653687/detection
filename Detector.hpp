//
//  Detector.hpp
//  
//
//  Created by 周吉夫 on 2020/5/7.
//

#ifndef Detector_hpp
#define Detector_hpp

#include <stdio.h>

#include "Util.hpp"

class Detection {
public:
    Detection(const vector<vector<vector<int>>>& tmpl, int downsample_scale, int top_K_particles) : tmpl_(tmpl) {
        util::downscale(tmpl_, downsample_scale, &tmpl_8_);
        downsample_scale_ = downsample_scale;
        top_K_particles_ = top_K_particles;
    }
    
    // Applies template based detection
    void detect(const vector<vector<vector<int>>>& frame, vector<float>* curr_det);

    // Overlays bounding box on frame based on detection position
    void OverlayBbox(const vector<float>& state);

    
    // Getters
    
    vector<vector<float>> response_top_K_trace(int T);

    vector<vector<float>> prev_response() {
        assert(!responses_.empty());
        return responses_.back();
    }
    
    vector<vector<float>> prev_response_filtered() {
        assert(!responses_filtered_.empty());
        return responses_filtered_.back();
    }

    vector<vector<int>> prev_fg_contour_mask() {
        assert(!prev_fg_contour_mask_.empty());
        return prev_fg_contour_mask_.back();
    }

    vector<vector<vector<int>>> prev_tmpl() {
        return tmpl_8_;
    }
    
    vector<vector<float>> fg_mask() {
        return prev_fg_mask_;
    }
    vector<vector<vector<int>>> img_detect_box() {
        return prev_detect_bbox_;
    }
    friend void detect_test();
    friend void response_top_K_test();
    
private:
    // Convolves frame with template to get response
    void convolve(const vector<vector<vector<int>>>& frame,
                  vector<vector<vector<int>>>* frame_processed,
                  vector<vector<float>>* response);
    
    // detect subroutine:
    void detect(const vector<vector<vector<int>>>& im_padded_downsampled,
                const vector<vector<float>>& response,
                const vector<vector<float>>& bg_mask,
                vector<vector<vector<int>>>* tmpl_updated,
                vector<vector<float>>* response_filter,
                int* high_response_ci, int* high_response_cj);
    
    // Segments foregournd / background
    void background_filter(const vector<vector<vector<int>>>& curr_img,
                           const vector<vector<float>>& response,
                           vector<vector<float>>* bg_mask);
    
    // Retrieves top K responses pixels
    vector<vector<float>> response_top_K(const vector<vector<float>>& responses);

    // Logging and filtering purposes
    vector<vector<vector<int>>> tmpl_;
    vector<vector<vector<int>>> tmpl_8_;
    vector<vector<vector<int>>> prev_img_;  // for foreground object segmentations
    vector<vector<vector<int>>> prev_detect_bbox_;
    vector<vector<float>> prev_fg_mask_;
    vector<vector<vector<int>>> prev_fg_contour_mask_;
    float highest_prev_response_;

    // History of responses
    // (Top K particles of first T frame responses initializes Kalman Filter)
    vector<vector<vector<float>>> responses_;
    vector<vector<vector<float>>> responses_filtered_;

    // Detector parameters
    int downsample_scale_;
    int top_K_particles_;

};


#endif /* Detector_hpp */
