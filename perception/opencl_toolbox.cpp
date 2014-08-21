#include "opencl_toolbox.h"

// error checking function
inline void checkErr(cl_int err, const char * name){
	if (err != CL_SUCCESS) {
		std::cerr << "ERROR: " << name << " (" << err << ")" << std::endl;
		exit(EXIT_FAILURE);
	}
}

const std::string hw("Hello World\n");

OpenClToolbox::OpenClToolbox()
{

}


// Initializes the OpenCL objects.
bool OpenClToolbox::Init_opencl() {
 /*
	cl_int err;
    cl::vector< cl::Platform > platformList;

    // get a platform
    cl::Platform::get(&platformList);

    checkErr(platformList.size()!=0 ? CL_SUCCESS : -1, "cl::Platform::get");
    std::cerr << "Platform number is: " << platformList.size() << std::endl;std::string platformVendor;

    platformList[0].getInfo((cl_platform_info)CL_PLATFORM_VENDOR, &platformVendor);
    std::cerr << "Platform is by: " << platformVendor << "\n";

    // get a context
    cl_context_properties cprops[3] = {CL_CONTEXT_PLATFORM, (cl_context_properties)(platformList[0])(), 0};
    cl::Context context(CL_DEVICE_TYPE_CPU, cprops, NULL, NULL, &err);
    checkErr(err, "Conext::Context()"); 


    // --------------------------

    // allocate memory on the host
	char * outH = new char[hw.length()+1];

	// send the pointer to the device to use as buffer
    cl::Buffer outCL(context, CL_MEM_WRITE_ONLY | CL_MEM_USE_HOST_PTR, hw.length()+1, outH, &err);
    checkErr(err, "Buffer::Buffer()");


    // --------------------------

    // get a specific device handle, based on the given context
	cl::vector<cl::Device> devices;
	// std::vector<cl::Device> devices;
    devices = context.getInfo<CL_CONTEXT_DEVICES>();
    checkErr(devices.size() > 0 ? CL_SUCCESS : -1, "devices.size() > 0");


    // --------------------------

    // read the kernel from disk
 	std::ifstream file("opencl_toolbox_kernels.cl");
    checkErr(file.is_open() ? CL_SUCCESS:-1, "opencl_toolbox_kernels.cl");

    // convert it to string
    std::string prog(std::istreambuf_iterator<char>(file), (std::istreambuf_iterator<char>()));

    // create a program using the source from the file
    cl::Program::Sources source(1, std::make_pair(prog.c_str(), prog.length()+1));
    cl::Program program(context, source);
    

    // build the program for all available devices
    err = program.build(devices,"");
    // err = clBuildProgram(program, 1, devices, "-g", NULL, NULL);

    checkErr(err, "Program::build()");


    // --------------------------

	// define the entry point kernel for this program
	cl::Kernel kernel(program, "hello", &err);
    checkErr(err, "Kernel::Kernel()");

    // set arguments
    err = kernel.setArg(0, outCL);
    checkErr(err, "Kernel::setArg()");


    // --------------------------

    // create a command queue for the device
    cl::CommandQueue queue(context, devices[0], 0, &err);
    checkErr(err, "CommandQueue::CommandQueue()");

    // enqueue the kernel on this queue
    cl::Event event;
    err = queue.enqueueNDRangeKernel(kernel, cl::NullRange, cl::NDRange(hw.length()+1), cl::NDRange(1, 1), NULL, &event);
    checkErr(err, "ComamndQueue::enqueueNDRangeKernel()");
*/
    return EXIT_SUCCESS;
}