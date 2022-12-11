/////////////////////////////////////////////////////////////////
//
// Name: test_Utils_LowPassFilter
//
// Test app for utils::LowPassFilter
//

#include <fstream>
#include <iostream>

#include "../../VS/include/utils.hpp"

int main(int argc, const char *argv[])
{
    // Filter parameters
    double input = 0.0;
    double output = 0.0;
    //double prev_output = 0.0;
    double sensitivity = 0.3;

    // Time-varying + constant disturbance
    double amplitude = 2.0;
    double frequency = 1.57;
    double constant_disturbance = 5.0;
    double runTime = 60.0;
    double timeResolution = 0.1; // step size /s
    std::string filename("test_utils_LowPassFilter.csv");

    // Get commandline parameters
    if(argc == 8)
    {
      filename = argv[1];
      amplitude = atof(argv[2]);
      frequency = atof(argv[3]);
      constant_disturbance = atof(argv[4]);
      sensitivity = atof(argv[5]);
      runTime = atof(argv[6]);
      timeResolution = atof(argv[7]);
    }
    else
    {
      std::cout << "USAGE: [output filename] [amplitude] [frequency/Hz] [constant disturbance] [sensitivity] [runTime] [time resolution]" << std::endl;
      std::cout << "Using default values." << std::endl;
    }

    std::cout << "filename = " << filename << std::endl;
    std::cout << "amplitude = " << amplitude << std::endl;
    std::cout << "frequency = " << frequency << std::endl;
    std::cout << "constant disturbance = " << constant_disturbance << std::endl;
    std::cout << "sensitivity = " << sensitivity << std::endl;
    std::cout << "run time = " << runTime << std::endl;
    std::cout << "time resolution = " << timeResolution << std::endl;

    std::ofstream file(filename.c_str());
    file << "time, input, output," << std::endl;

    double PI_2 = 2.0 * 3.14159265358979323;
    frequency *= PI_2;

    double t = 0.0; // time
    int iterations = runTime / timeResolution;

    // Create a LowPassFilter functor.
    vs::utils::LowPassFilter lpf(0.0, sensitivity);

    // Test filter over a finite number of steps.
    for(int i=0; i<iterations; ++i)
    {
      t = i * timeResolution;
      input = (amplitude * sin(frequency * t)) + constant_disturbance;
      output = lpf(input);

      // Save output to disk
      file << t << "," << input << "," << output << "," << std::endl;
    }

    file.close();

    return 0;
}

