#ifndef DEPTH_RENDER_CUH
#define DEPTH_RENDER_CUH

#include <cuda_runtime.h>
#include <cstdlib>
#include <stdio.h>
#include <vector>
#include <ctime>
#include <iostream>

#include "local_sensing/device_image.cuh"
#include "local_sensing/cuda_exception.cuh"
#include "local_sensing/helper_math.h"

using namespace std;

struct Parameter
{
	int point_number;
	float fx, fy, cx, cy;
	int width, height;
	float r[3][3];
	float t[3];
};

class DepthRender
{
	public:
		DepthRender();
		void set_para(float _fx, float _fy, float _cx, float _cy, int _width, int _height);
		~DepthRender();
		void set_data(vector<float> &cloud_data);
		void render_pose( double * transformation, int *host_ptr);
		
	private:
		int cloud_size;

		float3 *host_cloud_ptr;
		float3 *dev_cloud_ptr;
		bool has_devptr;

		Parameter parameter;
		Parameter* parameter_devptr;	
};

#endif