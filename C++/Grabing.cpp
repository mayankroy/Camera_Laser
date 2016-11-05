#include"stdafx.h"
#include <pylon/PylonIncludes.h>
#include <pylon/gige/BaslerGigECamera.h>
//#include <C:\Users\Pranav\Desktop\pcl_visualiser_with_menu_modular\Calibrate.h>
#include"opencv2\imgproc\imgproc.hpp"
#include"opencv2\imgproc\imgproc_c.h"
#include"cv.h"
#include"highgui.h"

using namespace cv;

using namespace Pylon;
using namespace Basler_GigECameraParams;
using namespace std;
typedef CBaslerGigECamera Camera_t;
struct MyContext
{
  // Define some application specific context information here
};

void ProcessImage(IplImage* cimg, unsigned char* pImage, int imageSizeX, int imageSizeY )
{

  // Do something with the image data
		
	
	for(int j=0;j<imageSizeY;j++){
		for(int i=0;i<imageSizeX;i++){			
			((uchar *)(cimg->imageData+j*cimg->widthStep))[i] = pImage[j*imageSizeX+i];			
		}
	}	

	
	//cvDestroyWindow("OUTPUT");
	

}

IplImage* GrabImage(){
	IplImage* cimg;
	PylonAutoInitTerm autoInitTerm;

	try
	{
		// Enumerate GigE cameras
		CTlFactory& TlFactory = CTlFactory::GetInstance();
		ITransportLayer *pTl = TlFactory.CreateTl( Camera_t::DeviceClass() );
		DeviceInfoList_t devices;
		if ( 0 == pTl->EnumerateDevices( devices ) )
		{
		  cerr << "No camera present!" << endl;
		  return cimg ;
		}
		// Create a camera object
		Camera_t Camera1 ( pTl->CreateDevice( devices[ 0 ] ) );

		// Open the camera object
		Camera1.Open();

		//Parameterize the camera

		//Mono8 pixel format
		Camera1.PixelFormat.SetValue(Basler_GigECameraParams::PixelFormat_Mono8);

		//Maximized AOI
		Camera1.OffsetX.SetValue(0);
		Camera1.OffsetY.SetValue(0);
		Camera1.Width.SetValue(Camera1.Width.GetMax());
		Camera1.Height.SetValue(Camera1.Height.GetMax());

		// Continuous mode, no external trigger used
		Camera1.TriggerSelector.SetValue( TriggerSelector_AcquisitionStart );
		Camera1.TriggerMode.SetValue( TriggerMode_Off );
		Camera1.AcquisitionMode.SetValue( AcquisitionMode_SingleFrame );

		// Configure exposure time and mode
		Camera1.ExposureMode.SetValue( ExposureMode_Timed);

		Camera1.ExposureAuto.SetValue(ExposureAuto_Off); //changed from ExposureAuto_Off to ExposureAuto_Continuous: tanya
		Camera1.ExposureTimeRaw.SetValue(2500 ); //removed for ExposureAuto_Continuous Mode3600

		// Configure Gain Raw mode
		//Camera1.GainAuto.SetValue(Basler_GigECameraParams::GainAuto_Once);
	

		//Get and open a stream grabber for camera 1
		IStreamGrabber* pGrabber1 = Camera1.GetStreamGrabber(0);
		CBaslerGigECamera::StreamGrabber_t StreamGrabber1(Camera1.GetStreamGrabber(0));
		StreamGrabber1.Open();

		// Parameterize the stream grabbers
		const int bufferSize = (int) Camera1.PayloadSize();
		StreamGrabber1.MaxBufferSize = bufferSize;
		StreamGrabber1.PrepareGrab();

		// Allocate and register image buffers, put them into the 
		// grabber's input queue for camera 1
		unsigned char* ppBuffers1;
		MyContext context1;
		StreamBufferHandle handles1;
		
	  ppBuffers1 = new unsigned char[bufferSize];
	  handles1 = StreamGrabber1.RegisterBuffer( ppBuffers1, bufferSize);
	  StreamGrabber1.QueueBuffer( handles1, &context1 );
		
	  GrabResult Result;
	  int flag=1;
	  char ch;
		
	  cimg = cvCreateImage(cvSize(Camera1.WidthMax(),Camera1.HeightMax()),IPL_DEPTH_8U,1);
	  while(flag){
		  // Start image acquisition
			Camera1.AcquisitionStart.Execute();
			// Wait for the grabbed image with a timeout of 3 seconds
			if ( StreamGrabber1.GetWaitObject().Wait( 3000 ))
			{
			// Get an item from the grabber's output queue
			if ( ! StreamGrabber1.RetrieveResult( Result ) )
			{
			 cerr << "Failed to retrieve an item from the output queue" << endl;
			 
			}
			if ( Result.Succeeded() )
			{
			  // Grabbing was successful. Process the image.
			  ProcessImage(cimg, (unsigned char*) Result.Buffer(), Result.GetSizeX(), Result.GetSizeY() );
			  //if(ch==27){
				  flag=0;
			  //}
			  
			}else 
			{
			  cerr << "Grab failed: " << Result.GetErrorDescription() << endl;
			  // The camera is in continuous mode, stop image acquisition 
			  Camera1.AcquisitionStop.Execute();
			  
			}
			// Requeue the buffer				
			StreamGrabber1.QueueBuffer( Result.Handle(), Result.Context() );

			}else
			{
			cerr << "timeout occurred when waiting for a grabbed image" 
				 << endl;		
			}
			// The camera is in continuous mode, stop image acquisition 
			Camera1.AcquisitionStop.Execute();

					
	  }
			
	    // Finished. Stop grabbing and do clean-up

		// Flush the input queue, grabbing may have failed
		StreamGrabber1.CancelGrab();

		// Consume all items from the output queue
		while ( StreamGrabber1.GetWaitObject().Wait(0) )
		{
		  StreamGrabber1.RetrieveResult( Result );
		  if ( Result.Status() == Canceled )
		  {
#if _DEBUG
			cout << "Got canceled buffer" << endl;
#endif
		  }
		}

		
		// Deregister and free buffers
		
	  StreamGrabber1.DeregisterBuffer(handles1);
	  delete [] ppBuffers1;
		
		// Clean up
		StreamGrabber1.FinishGrab();
		StreamGrabber1.Close();
		Camera1.Close();

		TlFactory.ReleaseTl( pTl );

	}catch( GenICam::GenericException &e )
	  {
		// Error handling
		cerr << "An exception occurred!" << endl << e.GetDescription() << endl;
		return cimg ;
	  }
	return cimg;
}

IplImage* GrabImage(int expo){
	IplImage* cimg;
	PylonAutoInitTerm autoInitTerm;

	try
	{
		// Enumerate GigE cameras
		CTlFactory& TlFactory = CTlFactory::GetInstance();
		ITransportLayer *pTl = TlFactory.CreateTl( Camera_t::DeviceClass() );
		DeviceInfoList_t devices;
		if ( 0 == pTl->EnumerateDevices( devices ) )
		{
		  cerr << "No camera present!" << endl;
		  return cimg ;
		}
		// Create a camera object
		Camera_t Camera1 ( pTl->CreateDevice( devices[ 0 ] ) );

		// Open the camera object
		Camera1.Open();

		//Parameterize the camera

		//Mono8 pixel format
		Camera1.PixelFormat.SetValue(Basler_GigECameraParams::PixelFormat_Mono8);

		//Maximized AOI
		Camera1.OffsetX.SetValue(0);
		Camera1.OffsetY.SetValue(0);
		Camera1.Width.SetValue(Camera1.Width.GetMax());
		Camera1.Height.SetValue(Camera1.Height.GetMax());

		// Continuous mode, no external trigger used
		Camera1.TriggerSelector.SetValue( TriggerSelector_AcquisitionStart );
		Camera1.TriggerMode.SetValue( TriggerMode_Off );
		Camera1.AcquisitionMode.SetValue( AcquisitionMode_SingleFrame );

		// Configure exposure time and mode
		Camera1.ExposureMode.SetValue( ExposureMode_Timed);

		Camera1.ExposureAuto.SetValue(ExposureAuto_Off); //changed from ExposureAuto_Off to ExposureAuto_Continuous: tanya
		Camera1.ExposureTimeRaw.SetValue(expo); //removed for ExposureAuto_Continuous Mode3600

		// Configure Gain Raw mode
		//Camera1.GainAuto.SetValue(Basler_GigECameraParams::GainAuto_Once);
	

		//Get and open a stream grabber for camera 1
		IStreamGrabber* pGrabber1 = Camera1.GetStreamGrabber(0);
		CBaslerGigECamera::StreamGrabber_t StreamGrabber1(Camera1.GetStreamGrabber(0));
		StreamGrabber1.Open();

		// Parameterize the stream grabbers
		const int bufferSize = (int) Camera1.PayloadSize();
		StreamGrabber1.MaxBufferSize = bufferSize;
		StreamGrabber1.PrepareGrab();

		// Allocate and register image buffers, put them into the 
		// grabber's input queue for camera 1
		unsigned char* ppBuffers1;
		MyContext context1;
		StreamBufferHandle handles1;
		
	  ppBuffers1 = new unsigned char[bufferSize];
	  handles1 = StreamGrabber1.RegisterBuffer( ppBuffers1, bufferSize);
	  StreamGrabber1.QueueBuffer( handles1, &context1 );
		
	  GrabResult Result;
	  int flag=1;
	  char ch;
		
	  cimg = cvCreateImage(cvSize(Camera1.WidthMax(),Camera1.HeightMax()),IPL_DEPTH_8U,1);
	  while(flag){
		  // Start image acquisition
			Camera1.AcquisitionStart.Execute();
			// Wait for the grabbed image with a timeout of 3 seconds
			if ( StreamGrabber1.GetWaitObject().Wait( 3000 ))
			{
			// Get an item from the grabber's output queue
			if ( ! StreamGrabber1.RetrieveResult( Result ) )
			{
			 cerr << "Failed to retrieve an item from the output queue" << endl;
			 
			}
			if ( Result.Succeeded() )
			{
			  // Grabbing was successful. Process the image.
			  ProcessImage(cimg, (unsigned char*) Result.Buffer(), Result.GetSizeX(), Result.GetSizeY() );
			  //if(ch==27){
				  flag=0;
			  //}
			  
			}else 
			{
			  cerr << "Grab failed: " << Result.GetErrorDescription() << endl;
			  // The camera is in continuous mode, stop image acquisition 
			  Camera1.AcquisitionStop.Execute();
			  
			}
			// Requeue the buffer				
			StreamGrabber1.QueueBuffer( Result.Handle(), Result.Context() );

			}else
			{
			cerr << "timeout occurred when waiting for a grabbed image" 
				 << endl;		
			}
			// The camera is in continuous mode, stop image acquisition 
			Camera1.AcquisitionStop.Execute();

					
	  }
			
	    // Finished. Stop grabbing and do clean-up

		// Flush the input queue, grabbing may have failed
		StreamGrabber1.CancelGrab();

		// Consume all items from the output queue
		while ( StreamGrabber1.GetWaitObject().Wait(0) )
		{
		  StreamGrabber1.RetrieveResult( Result );
		  if ( Result.Status() == Canceled )
		  {
#if _DEBUG
			cout << "Got canceled buffer" << endl;
#endif
		  }
		}

		
		// Deregister and free buffers
		
	  StreamGrabber1.DeregisterBuffer(handles1);
	  delete [] ppBuffers1;
		
		// Clean up
		StreamGrabber1.FinishGrab();
		StreamGrabber1.Close();
		Camera1.Close();

		TlFactory.ReleaseTl( pTl );

	}catch( GenICam::GenericException &e )
	  {
		// Error handling
		cerr << "An exception occurred!" << endl << e.GetDescription() << endl;
		return cimg ;
	  }
	return cimg;
}

void ProcessImage_usb( IplImage* inpimg ,IplImage* cimg, int imageSizeX, int imageSizeY )
{

  // Do something with the image data
	//cimg=cvCreateImage(cvSize(inpimg->width,inpimg->height),IPL_DEPTH_8U,1);
	cout<<inpimg->depth<<endl;	
	float temp;
	//int temp2=255;//pow(2.0,8)-1;
	for(int j=0;j<imageSizeY;j++){
		for(int i=0;i<imageSizeX;i++){
			//temp=((((uchar *)(inpimg->imageData+j*inpimg->widthStep))[i])/(pow(2,(inpimg->depth-1))))*255 ;	
			//temp=((((uchar *)(inpimg->imageData+j*inpimg->widthStep))[i])/(temp2))*255;
			temp=(((uchar *)(inpimg->imageData + j*inpimg->widthStep))[i*inpimg->nChannels + 0]*0.114+((uchar *)(inpimg->imageData + j*inpimg->widthStep))[i*inpimg->nChannels + 1]*0.587+((uchar *)(inpimg->imageData + j*inpimg->widthStep))[i*inpimg->nChannels + 2]*0.299);// /(temp2))*255;
			//temp=((uchar)(inpimg[j][i].b*0.114 + inpimg[j][i].g*0.587 + inpimg[j][i].r*0.299))/(temp2))*255;
			((uchar *)(cimg->imageData+j*cimg->widthStep))[i] = temp;
		}
	}	

	
	//cvDestroyWindow("OUTPUT");
	

}

IplImage* GrabImage_usb2(){
//IplImage* grey = 0;
IplImage* img;
IplImage* greyout;
CvCapture* capture = cvCaptureFromCAM(1); // capture from video device #0
//cvSetCaptureProperty(capture,CV_CAP_PROP_FRAME_WIDTH,1280);
//cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT,720);
//img->depth=IPL_DEPTH_8U;
if(!cvGrabFrame(capture)){              // capture a frame 
  printf("Could not grab a frame\n\7");
  exit(0);
}
img=cvRetrieveFrame(capture);
cvShowImage("first",img);
cvWaitKey(0);

IplImage* iplImg = cvQueryFrame( capture ); //added by tanya
cvShowImage("second",iplImg);
cvWaitKey(0);

// retrieve the captured frame

//cvCvtColor(img, grey, CV_BGR2GRAY);
//cvConvertImage(img, grey, 0);
greyout=cvCreateImage(cvSize(img->width,img->height),IPL_DEPTH_8U,1);
ProcessImage_usb(img, greyout, img->width, img->height);
cvReleaseCapture(&capture);
cvSaveImage("Trialimg.tif",greyout);

/* undistort Final Image */
// void undistort(InputArray src, OutputArray dst, InputArray cameraMatrix, InputArray distCoeffs, InputArray newCameraMatrix=noArray() ) //

double cameraMatrix[3][3]; //InputArray cameraMatrix
const Mat  CamMatx= Mat(3,3,CV_64F,cameraMatrix);
cameraMatrix[0][0]=2339.22;
cameraMatrix[0][1]=0.00;
cameraMatrix[0][2]=1247.86;
cameraMatrix[1][0]=0.00;
cameraMatrix[1][1]=2333.31;
cameraMatrix[1][2]=1041.13;
cameraMatrix[2][0]=0.00;
cameraMatrix[2][1]=0.00;
cameraMatrix[2][2]=-1.00;

double distCoeff[4][1]; // InputArray distCoeffs
const Mat  dCoeff= Mat(4,1,CV_64F,distCoeff);
distCoeff[0][0]=-0.181703;
distCoeff[1][0]=0.000102;
distCoeff[2][0]=-0.001169;
distCoeff[3][0]=-0.067033;

double newCameraMatrix[4][4]; //InputArray newCameraMatrix
const Mat  newCamMatx= Mat(4,4,CV_64F,newCameraMatrix);
newCameraMatrix[0][0]=2341.83;
newCameraMatrix[0][1]=18.78;
newCameraMatrix[0][2]=-1242.80;
newCameraMatrix[0][3]=6338.41;
newCameraMatrix[1][0]=2.31;
newCameraMatrix[1][1]=-2317.42;
newCameraMatrix[1][2]=-1076.03;
newCameraMatrix[1][3]=543383.69;
newCameraMatrix[2][0]=0.00;
newCameraMatrix[2][1]=-0.02;
newCameraMatrix[2][2]=1.00;
newCameraMatrix[2][3]=-266.05;

Mat greymat = Mat(greyout);
//cv::undistort(greymat, greymat, CamMatx, dCoeff,newCamMatx);

IplImage* greyout_new = new IplImage(greymat);
/*-----------------------*/
//return(greyout_new);
return greyout;
cvReleaseImage(&img);

}


IplImage* GrabImage_usb(){
//IplImage* grey = 0;
IplImage* img = 0;
IplImage* greyout;

CvCapture* capture = cvCaptureFromCAM(0); // capture from video device #0


//cvSetCaptureProperty(capture,CV_CAP_PROP_FRAME_WIDTH,1280);
//cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT,800);
//cvSetCaptureProperty(capture,CV_CAP_PROP_EXPOSURE,2000);
//cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT,800);


//img->depth=IPL_DEPTH_8U;
if(!cvGrabFrame(capture)){              // capture a frame 
  printf("Could not grab a frame\n\7");
  
  exit(0);
}



img=cvRetrieveFrame(capture);

IplImage* iplImg = cvQueryFrame( capture ); //added by tanya

// retrieve the captured frame

//cvCvtColor(img, grey, CV_BGR2GRAY);
//cvConvertImage(img, grey, 0);
greyout=cvCreateImage(cvSize(img->width,img->height),IPL_DEPTH_8U,1);
ProcessImage_usb(img, greyout, img->width, img->height);
cvReleaseCapture(&capture);
//cvSaveImage("Trialimg.tif",greyout);
return(greyout);
cvReleaseImage(&img);

}


IplImage* remove_distortion(IplImage* input)
{//this function removes distortion from an image

IplImage* output;

double cameraMatrix[3][3]; //InputArray cameraMatrix
const Mat  CamMatx= Mat(3,3,CV_64F,cameraMatrix);
cameraMatrix[0][0]=2339.22;
cameraMatrix[0][1]=0.00;
cameraMatrix[0][2]=1247.86;
cameraMatrix[1][0]=0.00;
cameraMatrix[1][1]=2333.31;
cameraMatrix[1][2]=1041.13;
cameraMatrix[2][0]=0.00;
cameraMatrix[2][1]=0.00;
cameraMatrix[2][2]=-1.00;

double distCoeff[4][1]; // InputArray distCoeffs
const Mat  dCoeff= Mat(4,1,CV_64F,distCoeff);
distCoeff[0][0]=-0.050676;
distCoeff[1][0]=5.007399;
distCoeff[2][0]=-0.003372;
distCoeff[3][0]=-0.003114;

double newCameraMatrix[3][4]; //InputArray newCameraMatrix
const Mat  newCamMatx= Mat(3,4,CV_64F,newCameraMatrix);
newCameraMatrix[0][0]=2341.83;
newCameraMatrix[0][1]=18.78;
newCameraMatrix[0][2]=-1242.80;
newCameraMatrix[0][3]=6338.41;
newCameraMatrix[1][0]=2.31;
newCameraMatrix[1][1]=-2317.42;
newCameraMatrix[1][2]=-1076.03;
newCameraMatrix[1][3]=543383.69;
newCameraMatrix[2][0]=0.00;
newCameraMatrix[2][1]=-0.02;
newCameraMatrix[2][2]=1.00;
newCameraMatrix[2][3]=-266.05;

 Mat greymat = Mat(input);
 Mat greyout; 
cv::undistort(greymat, greyout, CamMatx, dCoeff,newCamMatx);

IplImage *srcimg = cvCreateImage(cvSize(input->width,input->height),input->depth,1);
srcimg=cvCloneImage(&(IplImage)greymat);

/*-----------------------*/
return(input);
}

IplImage* remove_distortion_hole(IplImage* input)
{//this funxtion removes distortion from an image

IplImage* output;

double cameraMatrix[3][3]; //InputArray cameraMatrix
const Mat  CamMatx= Mat(3,3,CV_64F,cameraMatrix);
cameraMatrix[0][0]=2272.559448;
cameraMatrix[0][1]=0.00;
cameraMatrix[0][2]=1222.239603;
cameraMatrix[1][0]=0.00;
cameraMatrix[1][1]=1222.239603;
cameraMatrix[1][2]=1006.894116 ;
cameraMatrix[2][0]=0.00;
cameraMatrix[2][1]=0.00;
cameraMatrix[2][2]=-0.988630;

double distCoeff[4][1]; // InputArray distCoeffs
const Mat  dCoeff= Mat(4,1,CV_64F,distCoeff);
distCoeff[0][0]=-0.151239;
distCoeff[1][0]=-0.429654;
distCoeff[2][0]=-0.000935;
distCoeff[3][0]=-0.001337;

double newCameraMatrix[3][4]; //InputArray newCameraMatrix
const Mat  newCamMatx= Mat(3,4,CV_64F,newCameraMatrix);
newCameraMatrix[0][0]=2246.95;
newCameraMatrix[0][1]=66.54;
newCameraMatrix[0][2]=-1266.96;
newCameraMatrix[0][3]=12761.30;
newCameraMatrix[1][0]=17.35;
newCameraMatrix[1][1]=-2258.02;
newCameraMatrix[1][2]=-1059.17;
newCameraMatrix[1][3]=512737.93;
newCameraMatrix[2][0]=0.02;
newCameraMatrix[2][1]=-0.02;
newCameraMatrix[2][2]=0.99;
newCameraMatrix[2][3]=-268.30;

 Mat greymat = Mat(input);
 Mat greyout; 
cv::undistort(greymat, greyout, CamMatx, dCoeff,newCamMatx);

IplImage *srcimg = cvCreateImage(cvSize(input->width,input->height),input->depth,1);
srcimg=cvCloneImage(&(IplImage)greymat);

/*-----------------------*/
return(input);
}