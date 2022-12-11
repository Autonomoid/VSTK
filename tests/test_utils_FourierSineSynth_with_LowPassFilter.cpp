#include <fstream>
#include <iostream>

#include "../../VS/include/utils.hpp"

int main(int argc, const char *argv[])
{

    double disturbance = 0.0;
    double constant_disturbance = 10.0;

    // Time-varying disturbance
    double amplitude1 = 2.0;
    double frequency1 = 0.2;

    double amplitude2 = 1.0;
    double frequency2 = 0.5;

    double amplitude3 = 0.1;
    double frequency3 = 2.0;

    // Filter parameters
    //double input = 0.0;
    double output = 0.0;
    //double prev_output = 0.0;
    double sensitivity = 0.01;

    double runTime = 60.0;
    double timeResolution = 0.1; // step size /s

    std::string filename("test_utils_FourierSineSynth_with_LowPassFilter.csv");

    std::ofstream file(filename.c_str());
    file << "time, disturbance, output," << std::endl;

    // Build Fourier sine series
    std::vector<std::pair<double, double> > components;
    components.push_back(std::pair<double, double>(amplitude1, frequency1));
    components.push_back(std::pair<double, double>(amplitude2, frequency2));
    components.push_back(std::pair<double, double>(amplitude3, frequency3));

    double t = 0.0; // time
    int iterations = runTime / timeResolution;

    vs::utils::LowPassFilter lpf(0.0, sensitivity);

    // Test filter over a finite number of steps.
    for(int i=0; i<iterations; ++i)
    {
      t = i * timeResolution;
      disturbance = vs::utils::FourierSineSynth(t, components) + constant_disturbance;
      output = lpf(disturbance);

      // Save output to disk
      file << t << "," << disturbance << "," << output << "," << std::endl;
    }

    file.close();
    return 0;
}
