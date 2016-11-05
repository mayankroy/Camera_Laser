#include <pylon/PylonIncludes.h>
#include <pylon/gige/BaslerGigECamera.h>

using namespace Pylon;
using namespace Basler_GigECameraParams;

typedef struct 
	{
		CvPoint3D32f position;
		float orientation;
	}pos_orient;


//!!!!Z is always relative!!!!
typedef struct 
	{
		CvPoint3D32f position;
		float orientation;
		bool gripper;
	}pos_orient_gripper;

typedef struct 
	{
		CvPoint3D32f position;
		float orientation;
		bool standing;
	}pos_orient_standing;
	
typedef struct 
	{
		CvPoint3D32f position;
		float orientation;
		bool gripper;
		bool standing;
	}pos_orient_gripper_standing;


	
#define longest_edge  120//190 //90
#define shortest_edge 100 //50

typedef CBaslerGigECamera Camera_t;

void RunProfiler(double *x_k,double *y_k,double *z_k,double *a_k,double *b_k,double *c_k,double *calibflag,double (*arr)[1000][1280][8],int *ctr);