#pragma once
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <iostream>
#include <cmath>

void initialize_parameters(const char* file_path = "././data/limb_info.csv");

void preprocess_emg_data(const std::vector<float>& raw_emg_data, std::vector<float>& processed_data);

void postprocess_model_output(const float model_output[24][3], std::vector<float>& limb_positions);
