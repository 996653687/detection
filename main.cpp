//
//  main.cpp
//
//  Runs detection and filters given a set of test image frames and a template.
//
// g++ main.cpp Detector.cpp KalmanFilter.cpp Util.cpp  -std=c++2a && ./a.out ./data/test_data/ball/ output1/ 8 20
//
//  Created by 周吉夫 on 2020/5/7.
//

#include <stdio.h>

#include <iostream>

#include "Detector.hpp"
#include "KalmanFilter.hpp"

// TODO(jifu) parameterizes in a seperate config file
class Config {
public:
    Config(int argc, char** args) {
        iter_ = 0;
        assert(argc == 5);
        // Parsing arguments
        base_path_ = args[1];
        assert(Util::dir_exists(base_path_));

        string out_subdir = args[2];
        output_path_ = base_path_ + out_subdir;
        if (!Util::dir_exists(output_path_)) {
            assert(Util::mk_dir(output_path_));
        }

        downsample_scale_ = 8;  // Default
        assert(Util::is_int(string(args[3])));
        downsample_scale_ = std::stoi(args[3]);

        top_K_particles_ = 20;  // Default
        assert(Util::is_int(string(args[4])));
        top_K_particles_ = std::stoi(args[4]);

        // Preprocessed images converted to csv
        csv_subdir_ = "csv/";
        assert(Util::dir_exists(base_path_ + csv_subdir_));
        
        // Templates
        string template_subdir = "template/";
        template_filename_ = "template.jpg";
        templ_path_ = base_path_ + csv_subdir_ + template_filename_ + ".csv";
        assert(Util::file_exists(templ_path_));
        assert(Util::dir_exists(base_path_ + template_subdir));
        assert(Util::file_exists(base_path_ + template_subdir + template_filename_));
        
    }
    
    // TODO(jifu) to replace with the prod pipeline
    bool read_frame(vector<vector<vector<int>>>* frame, string* fn_img) {
        // Processes each image
        if (iter_ == 0) {
            vector<string> img_names;
            assert(Util::get_filenames(base_path_ + csv_subdir_, &img_names) != EXIT_FAILURE);
            vector<vector<vector<int>>> img;
            vector<vector<float>> response;
            vector<vector<float>> fg_mask;
            assert(!img_names.empty());
            for (string fn_img : img_names) {
                if (fn_img == template_filename_ + ".csv") {
                    continue;
                }
                std::cout << fn_img << "........";
                string img_path = base_path_ + csv_subdir_ + fn_img;
                // reads image
                util::csv2vec(img_path, &img);
                imgs_.push_back(img);
                imgs_filename_.push_back(fn_img);
            }
        }

        if (iter_ >= imgs_.size()) return false;
        *frame = imgs_[iter_];
        *fn_img = imgs_filename_[iter_];
        iter_++;
        return !frame->empty();
    }

    void Log(const vector<vector<vector<int>>>& img,
             const string& base, const string& log_name) {
        string output_file = base + "." + log_name + ".jpg.csv";
        std::cout << "writes " << log_name << " at " << output_file << std::endl;
        util::vec2csv(img, output_file);
    }
    
    void Log(const vector<vector<float>>& data,
             const string& base, const string& log_name) {
        string output_file = base + "." + log_name + ".csv";
        std::cout << "writes " << log_name << " at " << output_file << std::endl;
        util::vec2csv(data, output_file);
    }
    void Log(const vector<vector<int>>& data,
             const string& base, const string& log_name) {
        string output_file = base + "." + log_name + ".csv";
        std::cout << "writes " << log_name << " at " << output_file << std::endl;
        util::vec2csv(data, output_file);
    }

    string base_path() { return base_path_; }
    string template_filename() { return template_filename_; }
    string template_path() { return templ_path_; }
    string output_path() { return output_path_; }
    int downsample_scale() { return downsample_scale_; }
    int top_K_particles() { return top_K_particles_; }

private:
    string base_path_;
    string csv_subdir_;
    string templ_path_;
    string template_filename_;
    string output_path_;
    int downsample_scale_;

    int iter_;
    int top_K_particles_;
    vector<vector<vector<vector<int>>>> imgs_;
    vector<string> imgs_filename_;
};



int main(int argc, char** args) {
    if (argc != 5) {
        std::cout << "usage [BASE_PATH] [OUTPUT_SUBDIR] [DOWNSAMPLE_SCALE] [TOP_K_PARTICLES]" << std::endl;
        std::cout << "example: ./a.out ./experiment outputs/ 8 20" << std::endl;
    }
    Config config(argc, args);

    // Init detector
    vector<vector<vector<int>>> tmpl;
    util::csv2vec(config.template_path(), &tmpl);
    unique_ptr<Detection> detector(new Detection(tmpl, config.downsample_scale(), config.top_K_particles()));
    
    // Logging
    vector<vector<float>> detections;
    vector<float> curr_det;
    vector<vector<float>> states;
    vector<float> curr_state;
    
    // Applies detection and filtering (state estimates) for each frame
    unique_ptr<KalmanFilter> kf;
    vector<vector<vector<int>>> frame;
    string frame_img_filename;
    int num_frames = 0;
    // TODO(jifu) Parameterizes in config
    int init_filter_step = 2;
    while (config.read_frame(&frame, &frame_img_filename)) {
        num_frames++;
        std::cout << "\n--------frame " << num_frames << " : " << frame_img_filename << "----------" << std::endl;
        // Detects
        detector->detect(frame, &curr_det);
        detections.push_back(curr_det);
        if (num_frames == init_filter_step) {
            // Inits filter given a few frames
            kf.reset(new KalmanFilter(detector->response_top_K_trace(init_filter_step)));
            states.push_back({curr_det[0], curr_det[1]});

        } else if (num_frames > init_filter_step) {

            // Applies filter
            // TODO(jifu) use detector->curr_confidence() metric as sig_m;
            kf->apply(curr_det, &curr_state);
            states.push_back({curr_state[0], curr_state[1]});
        } else {
            states.push_back({curr_det[0], curr_det[1]});
        }

        // Visualizes detection
        detector->OverlayBbox(states.back());

        // Logging
        std::cout << "Detection: " << std::endl;
        Print(curr_det);
        std::cout << "States (estimated): " << std::endl;
        Print(curr_state);
        string output_base = config.output_path() + frame_img_filename;
        config.Log(detector->prev_response(), output_base, "response");
        if (!detector->prev_response_filtered().empty()) {
            config.Log(detector->prev_response_filtered(), output_base, "response_filtered");
        }
        if (!detector->fg_mask().empty()) {
            config.Log(detector->prev_fg_contour_mask(), output_base, "bg_mask");
        }
        config.Log(detector->img_detect_box(), output_base, "detection");
        std::cout << "done --------frame " << num_frames << std::endl;

    }
    // Logging
    kf->print_kalman_gain();
    std::cout << "All detections " << std::endl;
    Print(detections);
    std::cout << "All states " << std::endl;
    Print(states);

    // Visualizes detection traces
    Util::overlay(detections, 255, states, 0, config.downsample_scale(), &frame);
    config.Log(frame, config.output_path() + "output", "traces");
}
