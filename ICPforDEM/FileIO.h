#pragma once

#include <iostream>
#include <stdio.h>
#include <fstream>
#include <string>
#include <iostream>
#include <cmath>
#include <iomanip>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
//#include <pcl/registration/correspondence_rejection_trimmed.h>

using namespace std;

class FileIO
{
public:
	FileIO();
	~FileIO();
	void readASC(const char *file1, const char *file2);
	void print4x4Matrix(const Eigen::Matrix4d & matrix);
};

