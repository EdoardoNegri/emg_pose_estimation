#include "process.h"
#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <array>
#include <iostream>
#include <cmath>

#define PI_DIV_360 0.008726646259971648f // Half of 180 degrees in radians

struct Limb {
    float length;
    std::array<float, 3> half_angles;
    std::vector<int> child_idxs;
};

static Limb limbs[24];

void initialize_parameters(const char* file_path) {
    std::ifstream file(file_path);

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream line_stream(line);

        int limb_start, limb_end;
        float length_mm;
        float min_x, max_x, min_y, max_y, min_z, max_z;
        char comma;

        line_stream
            >> limb_start >> comma
            >> limb_end >> comma
            >> length_mm >> comma
            >> min_x >> comma
            >> max_x >> comma
            >> min_y >> comma
            >> max_y >> comma
            >> min_z >> comma
            >> max_z;

        limbs[limb_start].child_idxs.push_back(limb_end);
        limbs[limb_end].half_angles = { // to radians divided by 2
            (max_x - min_x) * PI_DIV_360, 
            (max_y - min_y) * PI_DIV_360, 
            (max_z - min_z) * PI_DIV_360};
        limbs[limb_end].length = length_mm ;
    }
}


// process input data from the emg
void preprocess_emg_data(const std::vector<float>& raw_emg_data, std::vector<float>& processed_data) {
    // apply filters to remove noise
    // normalize the data
    // extract features (e.g., mean, variance, etc.)
}

// transform model output to position of limbs (use AVX2.0?)
void postprocess_model_output(const float model_output[24][3], std::vector<float>& limb_positions) {
    Limb current_limb = limbs[0]; // Start with the root limb
    std::array<std::array<float, 3>, 24> joint_positions; // Initial position of the root limb
    joint_positions[0] = {0.0f, 0.0f, 0.0f}; // Assuming the root limb is at the origin

    for (const int child_idx : current_limb.child_idxs){
        Limb& child_limb = limbs[child_idx];
        
        const float roll = model_output[child_idx][0] * child_limb.half_angles[0];
        const float pitch = model_output[child_idx][1] * child_limb.half_angles[1];
        const float yaw = model_output[child_idx][2] * child_limb.half_angles[2];

        const float cr = std::cos(roll);
        const float sr = std::sin(roll);
        const float cp = std::cos(pitch);
        const float sp = std::sin(pitch);
        const float cy = std::cos(yaw);
        const float sy = std::sin(yaw);

        
    }

}
