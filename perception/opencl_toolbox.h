#ifndef OPENCL_TOOLBOX_H
#define OPENCL_TOOLBOX_H

// OpenCL includes
#include <utility>
#define __NO_STD_VECTOR // Use cl::vector instead of STL version
#include <CL/cl.hpp>

// C++ includes
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <iterator>

class OpenClToolbox
{
public:
    OpenClToolbox();

    bool Init_opencl();

private:
	const std::string hw;
};

#endif // OPENCL_TOOLBOX_H
