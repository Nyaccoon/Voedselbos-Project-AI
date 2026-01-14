#pragma once
#define DEPTHAI_HAVE_OPENCV_SUPPORT
#ifdef _DEBUG
#pragma comment (lib, "depthai-cored.lib")
#pragma comment (lib, "depthai-resourcesd.lib")
#pragma comment (lib, "opencv_world4110d.lib")
#else
#pragma comment (lib, "./lib/depthai-core.lib")
#pragma comment (lib, "./lib/depthai-resources.lib")
#pragma comment (lib, "./lib/opencv_world4110.lib")
#endif

#include <thread>
#include <chrono>
#include "./lib/depthai/depthai.hpp"
#include "./lib/opencv2/opencv.hpp"

#include "./lib/depthai/pipeline/datatype/Tracklets.hpp"


typedef struct {
	double x;
	double y;
	double z;
	bool ripe;
} FruitPos;