
/*
viveOri.c
Author: Samuel Love (built off information gained from example files from
OpenHMD github page
Year: 2020

Extracts IMU orientation data from HTC Vive headset and prints information to stdout
*/

// Includes
#include <openhmd.h>
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int main(int argc, char** argv)
{
	ohmd_context* headsets = ohmd_ctx_create();
	int devicesAvailable = ohmd_ctx_probe(headsets);
	if (devicesAvailable < 1)
	{
		printf("Headset not detected");
		return -1;
	}
	ohmd_device* vive = ohmd_list_open_device(headsets, 0);

	if(!vive){
		printf("Couldn't open device: %s\n", ohmd_ctx_get_error(headsets));
		return -1;
	}

	float orientationMatrix[4];
	char axis[4] = "wxyz";
	while(1)
	{
		ohmd_ctx_update(headsets); // Update headset object
		ohmd_device_getf(vive, OHMD_ROTATION_QUAT, orientationMatrix);
		usleep(10000);
		for(int i = 0; i<4; i++)
		{
			printf("Value at %c is %f", axis[i], orientationMatrix[i]);
			printf("\n");
			fflush(stdout); // Clear stdout to trigger a Python update
		}
	}
	ohmd_ctx_destroy(headsets);
}
