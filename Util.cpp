//
//  Util.cpp
//  
//
//  Created by 周吉夫 on 2020/5/3.
//

#include "Util.hpp"

#include <dirent.h>
#include <ctype.h>
//#include <io.h>
#include <sys/stat.h>

void Util::overlay(const vector<vector<float>>& A, int v_a,
               const vector<vector<float>>& B, int v_b,
               int scale,
               vector<vector<vector<int>>>* frame) {
    vector<vector<float>> lines_A;
    for (int i = 0; i < A.size(); i++) {
        lines_A.push_back({A[i][0] * scale, A[i][1] * scale});
    }
    vector<vector<float>> lines_B;
    for (int i = 0; i < B.size(); i++) {
        lines_B.push_back({B[i][0] * scale, B[i][1] * scale});
    }
    util::OverlayLines(lines_A, v_a, (float)scale, frame);
    util::OverlayLines(lines_B, v_b, (float)scale, frame);
}

int Util::get_filenames(const string& base_path, vector<string>* file_names) {
    file_names->clear();
    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir (base_path.c_str())) != NULL) {
        /* print all the files and directories within directory */
        while ((ent = readdir (dir)) != NULL) {
            string fn(ent->d_name);
            if(fn.substr(fn.find_last_of(".") + 1) == "csv") {
                file_names->push_back(fn);
            }
        }
        closedir (dir);
    } else {
        /* could not open directory */
        perror ("");
        return EXIT_FAILURE;
    }
    
    // sort images according to the name
    sort(file_names->begin(), file_names->end());
    for (int i = 0; i < file_names->size(); i++) {
        std::cout << (*file_names)[i] << std::endl;
    }
    
    return EXIT_SUCCESS;
}

bool Util::dir_exists(const string& path) {
    struct stat buffer;
    if (stat (path.c_str(), &buffer) != 0) {
        std::cout << path << " is not accessible" << std::endl;
        return false;
    }
    return true;
    
}

bool Util::file_exists(const string& path) {
    struct stat buffer;
    if (stat (path.c_str(), &buffer) != 0) {
        std::cout << path << " does not exists" << std::endl;
        return false;
    }
    return true;
}

bool Util::mk_dir(const string& path) {
    if (mkdir(path.c_str(), 0777) != 0) {
        std::cout << "failed to make directory " << path << std::endl;
        return false;
    }
    return true;
}

bool Util::is_int(const string& arg) {
    for (auto it = arg.begin(); it != arg.end(); it++) {
        if (!isdigit(*it)) {
            std::cout << arg << " is not an integer" << std::endl;
            return false;
        }
    }
    return true;
}

/************************************************************/
// Geometry

bool Util::island(const vector<vector<int>>& mask, vector<vector<int>>* island_marks, int i, int j, int count) {
    int n = mask.size();
    int m = mask[0].size();
    if (i < n && i >= 0 && j < m && j >= 0 && mask[i][j] && (*island_marks)[i][j] < 0) {
        (*island_marks)[i][j] = count;
        island(mask, island_marks, i - 1, j, count);
        island(mask, island_marks, i, j - 1, count);
        island(mask, island_marks, i, j, count);
        island(mask, island_marks, i, j + 1, count);
        island(mask, island_marks, i + 1, j, count);
        return true;
    }
    return false;
}

vector<vector<int>> Util::island_marks(const vector<vector<int>>& mask) {
    int count = 0;
    int n = mask.size();
    int m = mask[0].size();
    vector<vector<int>> curr_mask = mask;
    vector<vector<int>> marks(n, vector<int>(m, -1));
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            if(island(curr_mask, &marks, i, j, count)) {
                count++;
            }
        }
    }
    return marks;
}

void Util::islands_by_indices(const vector<vector<int>>& islands, vector<vector<bool>>* visited, int i, int j, int count, vector<vector<int>>* pos) {
    int n = islands.size();
    int m = islands[0].size();
    if (i >= 0 && i < n && j >=0 && j < m && islands[i][j] == count && !(*visited)[i][j]) {
        (*visited)[i][j] = true;
        vector<int> xy = {i, j};
        pos->push_back(xy);
        islands_by_indices(islands, visited, i - 1, j - 1, count, pos);
        islands_by_indices(islands, visited, i - 1, j, count, pos);
        islands_by_indices(islands, visited, i - 1, j + 1, count, pos);
        islands_by_indices(islands, visited, i, j - 1, count, pos);
        islands_by_indices(islands, visited, i, j, count, pos);
        islands_by_indices(islands, visited, i, j + 1, count, pos);
        islands_by_indices(islands, visited, i + 1, j - 1, count, pos);
        islands_by_indices(islands, visited, i + 1, j, count, pos);
        islands_by_indices(islands, visited, i + 1, j + 1, count, pos);
    }
}

vector<vector<vector<int>>> Util::islands_by_indices(const vector<vector<int>>& islands) {
    int count = 0;
    vector<vector<vector<int>>> islands_pos;
    int n = islands.size();
    assert(n > 0);
    int m = islands[0].size();
    vector<vector<bool>> visited(n, vector<bool>(m, 0 ));
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            if (islands[i][j] == count) {
                vector<vector<int>> pos;
                islands_by_indices(islands, &visited, i, j, count, &pos);
                islands_pos.push_back(pos);
                count++;
            }
        }
    }
    return islands_pos;
}

vector<vector<vector<int>>> Util::islands(const vector<vector<int>>& mask, int radius) {
    int n = mask.size();
    int m = mask[0].size();
    vector<vector<int>> mask_radius = mask;
    vector<vector<vector<int>>> island_marks;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            if (mask[i][j]) {
                // TODO(jifu) L2 radius
                for (int ii = i - radius; ii <= i + radius; ii++) {
                    for (int jj = j - radius; jj <= j + radius; jj++) {
                        if (ii >= 0 && ii < n && jj >= 0 && jj < m) {
                            mask_radius[ii][jj] = 1;
                        }
                    }
                }
            }
        }
    }
    vector<vector<int>> islands = Util::island_marks(mask_radius);
    return islands_by_indices(islands);
}


float Util::Angle(const vector<int>& A, const vector<int>& B) {
    float prod = util::Dot(A, B);
    float norm_a = util::Norm_L2<float, int>(A);
    float norm_b = util::Norm_L2<float, int>(B);
    assert(norm_a > 0);
    assert(norm_b > 0);
    float ang = prod / norm_a / norm_b;
    return acos(ang) * 180.0 / 3.141592653;
}

void Util::sort_base_on_weight(vector<vector<int>>* v, const vector<float>& weights) {
    map<float, vector<int>> m;
    int n = weights.size();
    for (int i = 0; i < n; i++) {
        m[weights[i]].push_back(i);
    }
    vector<int> ind;
    for (auto kv : m) {
        for (int i = 0; i < kv.second.size(); i++) {
            ind.push_back(kv.second[i]);
        }
    }
    vector<vector<int>> v_sorted;
    for (int i = 0; i < ind.size(); i++) {
        v_sorted.push_back((*v)[ind[i]]);
    }
    *v = v_sorted;
}

vector<vector<int>> Util::polygon_fit_2d(const vector<vector<int>>& pos) {
    assert(pos.size() > 0);
    int x0 = pos[0][0];
    int y0 = pos[0][1];
    
    vector<float> angles;
    for (int i = 1; i < pos.size(); i++) {
        int x = pos[i][0] - x0;
        int y = pos[i][1] - y0;
        float ang = atan((float)y / x) * 180 / 3.141592653;
        angles.push_back(ang);
    }
    vector<vector<int>> polygon(pos.begin()+1, pos.end());

    sort_base_on_weight(&polygon, angles);

    polygon.insert(polygon.begin(), pos[0]);
    return polygon;
}

bool Util::in_polygon(const vector<int>& base, const vector<vector<int>>& polygon) {
    int num_points = polygon.size();
    for (int i = 0; i < num_points; i++) {
        if (base[0] == polygon[i][0] && base[1] == polygon[i][1]) {
            return true;
        }
    }
    float max_ang = -INT_MAX;
    float min_ang = INT_MAX;
    for (int i = 1; i < num_points; i++) {
        int x = polygon[i][0] - base[0];
        int y = polygon[i][1] - base[1];
        float ang = atan((float)y / x) * 180 / 3.141592653;
        if (ang > max_ang) max_ang = ang;
        if (ang < min_ang) min_ang = ang;
    }
    float diff = max_ang - min_ang;
    // assuming convex
    return diff >= 180.0 - 10e-4;
}

void Util::contours(const vector<vector<int>>& mask, int radius, vector<vector<vector<int>>>* contours, vector<vector<int>>* contours_mask) {
    vector<vector<vector<int>>> islands = Util::islands(mask, radius);
    contours->clear();
    int num_islands = islands.size();
    for (int i = 0; i < num_islands; i++) {
        contours->push_back(Util::polygon_fit_2d(islands[i]));
    }
    // contour mask
    int n = mask.size();
    int m = mask[0].size();
    int num_polygon = contours->size();
    *contours_mask = vector<vector<int>>(n, vector<int>(m, 0));
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            for (int p = 0; p < num_polygon; p++) {
                if (in_polygon({i, j}, (*contours)[p])) {
                    (*contours_mask)[i][j] = 1;
                }
            }
        }
    }
    
}
