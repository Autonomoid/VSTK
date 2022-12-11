/////////////////////////////////////////////////////////////////
//
// Name: test_Utils_FourierSineSynth
//
// Test app for utils::FourierSineSynth
//
// Return superpostion of several sine waves of
// given amplitudes and frequencies.
//
//

#include <fstream>
#include <iostream>

#include "../../VS/include/utils.hpp"

int main(int argc, const char *argv[])
{
    double output = 0.0;

    double amplitude1 = 2.0;
    double frequency1 = 0.2;

    double amplitude2 = 1.0;
    double frequency2 = 0.5;

    double amplitude3 = 0.1;
    double frequency3 = 2.0;

    double runTime = 10.0;
    double timeResolution = 0.1; // step size /s
    std::string filename("test_utils_FourierSineSynth.csv");

    // Get commandline parameters
    if(argc == 10)
    {
      filename = argv[1];
      amplitude1 = atof(argv[2]);
      frequency1 = atof(argv[3]);
      amplitude2 = atof(argv[4]);
      frequency2 = atof(argv[5]);
      amplitude3 = atof(argv[6]);
      frequency3 = atof(argv[7]);
      runTime = atof(argv[8]);
      timeResolution = atof(argv[9]);
    }
    else
    {
      std::cout << "USAGE: [output filename] [amp1] [freq1] [amp2] [freq2] [amp3] [freq3] [runTime] [time resolution]" << std::endl;
      std::cout << "Using default values." << std::endl;
    }

    std::cout << "filename = " << filename << std::endl;
    std::cout << "amplitude 1 = " << amplitude1 << std::endl;
    std::cout << "frequency 1 = " << frequency1 << std::endl;
    std::cout << "amplitude 2 = " << amplitude2 << std::endl;
    std::cout << "frequency 2 = " << frequency2 << std::endl;
    std::cout << "amplitude 3 = " << amplitude3 << std::endl;
    std::cout << "frequency 3 = " << frequency3 << std::endl;
    std::cout << "run time = " << runTime << std::endl;
    std::cout << "time resolution = " << timeResolution << std::endl;

    std::ofstream file(filename.c_str());
    file << "time, output," << std::endl;

    std::vector<std::pair<double, double> > components;
    components.push_back(std::pair<double, double>(amplitude1, frequency1));
    components.push_back(std::pair<double, double>(amplitude2, frequency2));
    components.push_back(std::pair<double, double>(amplitude3, frequency3));

    double t = 0.0; // time
    int iterations = runTime / timeResolution;

    // Test filter over a finite number of steps.
    for(int i=0; i<iterations; ++i)
    {
      t = i * timeResolution;
      output = vs::utils::FourierSineSynth(t, components);

      // Save output to disk
      file << t << "," << output << std::endl;
    }

    file.close();

    return 0;
}
