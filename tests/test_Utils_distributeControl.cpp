/////////////////////////////////////////////////////////////////
//
// Name: test_Utils_distributeControl
//
// Test app for utils::distributeControl
//
// Smoothly transition beitween two control modes
// as a approach distance varies.
//

#include <fstream>

#include "../../VS/include/utils.hpp"

int main(int argc, const char *argv[])
{
    double interventionInput = 10.0;
    double approachInput = 20.0;
    double criterion = 0.0;
    double startingDistance = 100.0;
    double approachThreshold = 75.0;
    double interventionThreshold = 25.0;
    double output = 0.0;

    std::string filename("test_utils_distributeControl.csv");

    // Get commandline parameters
    if(argc == 7)
    {
      filename = argv[1];
      approachInput = atof(argv[2]);
      interventionInput = atof(argv[3]);
      startingDistance = atof(argv[4]);
      approachThreshold = atof(argv[5]);
      interventionThreshold = atof(argv[6]);
    }
    else
    {
      std::cout << "USAGE: [output filename] [approachInput] [interventionInput] [criterion] [approachThreshold] [interventionThreshold]" << std::endl;
      std::cout << "Using default values." << std::endl;
    }

    std::cout << "filename = " << filename << std::endl;
    std::cout << "approachInput = " << approachInput << std::endl;
    std::cout << "interventionInput = " << interventionInput << std::endl;
    std::cout << "startingDistance = " << startingDistance << std::endl;
    std::cout << "approachThreshold = " << approachThreshold << std::endl;
    std::cout << "interventionThreshold = " << interventionThreshold << std::endl;
    std::ofstream file(filename.c_str());
    file << "output," << std::endl;

    for(int i=startingDistance; i>0; --i)
    {
      criterion = (double)(i);
      output = vs::utils::distributeControl(approachInput, interventionInput, criterion,
                                        approachThreshold, interventionThreshold);

      file << output << "," << std::endl;
    }

    file.close();
    return 0;
}

