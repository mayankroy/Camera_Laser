//   GetProfiles_Ethernet.cpp: demo-application for using the LLT.dll
//
//   Version 3.0.0.0
//
//   Copyright 2009
// 
//   Sebastian Lueth
//   MICRO-EPSILON Optronic GmbH
//   Lessingstrasse 14
//   01465 Dresden OT Langebrueck
//   Germany
//---------------------------------------------------------------------------

#include "stdafx.h"
#include <iostream>
#include <conio.h>
#include "InterfaceLLT_2.h"
#include "GetProfiles_Ethernet.h"

using namespace std;

#include <time.h>       /* clock_t, clock, CLOCKS_PER_SEC */
clock_t tOld;
clock_t tNew;
float fTime;
float arr[1000][1280][8];
//int ctr=0;


FILE* fpi ;
int file_cnt=0;
char profile_file[100];


struct Point{
				double x;
				double z;
} ;

void RunProfiler(double *x_k,double *y_k,double *z_k,double *a_k,double *b_k,double *c_k,double *calibflag,double (*arr)[1000][1280][8], int *ctr)
{
  std::vector<unsigned int> vuiEthernetInterfaces(MAX_INTERFACE_COUNT);
  std::vector<DWORD> vdwResolutions(MAX_RESOULUTIONS);
  unsigned int uiEthernetInterfaceCount = 0;
  //unsigned int uiShutterTime = 11;
  //unsigned int uiIdleTime = 989;
  unsigned int uiShutterTime = 1000;
  unsigned int uiIdleTime = 0;
  unsigned int uiLaserPowerON = 1;
  unsigned int uiLaserPowerOFF = 0;

  bool bLoadError;
  int iRetValue;
  bool bON=true;
  bool bOK = true;
  bool bConnected = false;
  m_uiResolution = 0;

  //Creating a LLT-object
  //The LLT-Object will load the LLT.dll automaticly and give us a error if ther no LLT.dll
  m_pLLT = new CInterfaceLLT("LLT.dll", &bLoadError);

  if(bLoadError)
  {
    cout << "Error loading LLT.dll \n";

    //Wait for a keyboard hit
    while(!_kbhit()) {}

    //Deletes the LLT-object
    delete m_pLLT;
    //return -1;
  }

  //Test if the LLT.dll supports GetLLTType (Version 3.0.0.0 or higher)
  if(m_pLLT->m_pFunctions->CreateLLTDevice == NULL)
  {
    cout << "Please use a LLT.dll version 3.0.0.0 or higher! \n";
  }
  else
  {
    //Create a Firewire Device
    if(m_pLLT->CreateLLTDevice(INTF_TYPE_ETHERNET))
      cout << "CreateLLTDevice Ethernet OK \n";
    else
      cout << "Error during CreateLLTDevice\n";

    //Gets the available interfaces from the scanCONTROL-device
    iRetValue = m_pLLT->GetDeviceInterfacesFast(&vuiEthernetInterfaces[0], (unsigned int)vuiEthernetInterfaces.size());
    
    if(iRetValue == ERROR_GETDEVINTERFACES_REQUEST_COUNT)
    {
      cout << "There are more or equal than " << vuiEthernetInterfaces.size() << " scanCONTROL connected \n";
      uiEthernetInterfaceCount = vuiEthernetInterfaces.size();
    }
    else if(iRetValue < 0)
    {
      cout << "A error occured during searching for connected scanCONTROL \n";
      uiEthernetInterfaceCount = 0;
    }
    else
    {
      uiEthernetInterfaceCount = iRetValue;
    }

    if(uiEthernetInterfaceCount == 0)
      cout << "There is no scanCONTROL connected \n";
    else if(uiEthernetInterfaceCount == 1)
      cout << "There is 1 scanCONTROL connected \n";
    else
      cout << "There are " << uiEthernetInterfaceCount << " scanCONTROL's connected \n";

    if(uiEthernetInterfaceCount >= 1)
    {
      cout << "\nSelect the device interface " << vuiEthernetInterfaces[0] << "\n";
      if((iRetValue = m_pLLT->SetDeviceInterface(vuiEthernetInterfaces[0], 0)) < GENERAL_FUNCTION_OK)
      {
        OnError("Error during SetDeviceInterface", iRetValue);
        bOK = false;
      }
       
      if(bOK)
      {
        cout << "Connecting to scanCONTROL\n";
        if((iRetValue = m_pLLT->Connect()) < GENERAL_FUNCTION_OK)
        {
          OnError("Error during Connect", iRetValue);
          bOK = false;
        }
        else
          bConnected = true;
      }

      if(bOK)
      {
        cout << "Get scanCONTROL type\n";
        if((iRetValue = m_pLLT->GetLLTType(&m_tscanCONTROLType)) < GENERAL_FUNCTION_OK)
        {
          OnError("Error during GetLLTType", iRetValue);
          bOK = false;
        }
        
        if(iRetValue == GENERAL_FUNCTION_DEVICE_NAME_NOT_SUPPORTED)
        {
          cout << "Can't decode scanCONTROL type. Please contact Micro-Epsilon for a newer version of the LLT.dll.\n";
        }

        if(m_tscanCONTROLType >= scanCONTROL27xx_25 && m_tscanCONTROLType <= scanCONTROL27xx_xxx)
        {
          cout << "The scanCONTROL is a scanCONTROL27xx\n\n";
        }
        else if(m_tscanCONTROLType >= scanCONTROL26xx_25 && m_tscanCONTROLType <= scanCONTROL26xx_xxx)
        {
          cout << "The scanCONTROL is a scanCONTROL26xx\n\n";
        }
        else if(m_tscanCONTROLType >= scanCONTROL29xx_25 && m_tscanCONTROLType <= scanCONTROL29xx_xxx)
        {
          cout << "The scanCONTROL is a scanCONTROL29xx\n\n";
        }
        else
        {
          cout << "The scanCONTROL is a undefined type\nPlease contact Micro-Epsilon for a newer SDK\n\n";
        }

        cout << "Get all possible resolutions\n";
        if((iRetValue = m_pLLT->GetResolutions(&vdwResolutions[0], vdwResolutions.size())) < GENERAL_FUNCTION_OK)
        {
          OnError("Error during GetResolutions", iRetValue);
          bOK = false;
        }

        m_uiResolution = vdwResolutions[0];
      }
      
      if(bOK)
      {
        cout << "Set resolution to " << m_uiResolution << "\n";
        if((iRetValue = m_pLLT->SetResolution(m_uiResolution)) < GENERAL_FUNCTION_OK)
        {
          OnError("Error during SetResolution", iRetValue);
          bOK = false;
        }
      }

      if(bOK)
      {
        cout << "Set trigger to internal\n";
        if((iRetValue = m_pLLT->SetFeature(FEATURE_FUNCTION_TRIGGER, 0x00000000)) < GENERAL_FUNCTION_OK)
        {
          OnError("Error during SetFeature(FEATURE_FUNCTION_TRIGGER)", iRetValue);
          bOK = false;
        }
      }

      if(bOK)
      {
        cout << "Profile config set to PROFILE\n";
        if((iRetValue = m_pLLT->SetProfileConfig(PROFILE)) < GENERAL_FUNCTION_OK)
        {
          OnError("Error during SetProfileConfig", iRetValue);
          bOK = false;
        }
      }


      if(bOK)
      {
        cout << "Set shutter time to " << uiShutterTime << "\n";
        if((iRetValue = m_pLLT->SetFeature(FEATURE_FUNCTION_SHUTTERTIME, uiShutterTime)) < GENERAL_FUNCTION_OK)
        {
          OnError("Error during SetFeature(FEATURE_FUNCTION_SHUTTERTIME)", iRetValue);
          bOK = false;
        }
      }

      if(bOK)
      {
        cout << "Set idle time to " << uiIdleTime << "\n";
        if((iRetValue = m_pLLT->SetFeature(FEATURE_FUNCTION_IDLETIME, uiIdleTime)) < GENERAL_FUNCTION_OK)
        {
          OnError("Error during SetFeature(FEATURE_FUNCTION_IDLETIME)", iRetValue);
          bOK = false;
        }
      }

	  //Laser Power ON
	  if(bOK)
      {
        cout << "Set laser power to " << uiLaserPowerON << "\n";
        if((iRetValue = m_pLLT->SetFeature(FEATURE_FUNCTION_LASERPOWER, uiLaserPowerON)) < GENERAL_FUNCTION_OK)
        {
          OnError("Error during SetFeature(FEATURE_FUNCTION_LASERPOWER)", iRetValue);
          bOK = false;
        }
      }

      if(bOK)
      {
		cout<<"value of calib flag is"<<*calibflag<<endl;
        GetProfiles_Ethernet(x_k,y_k,z_k,a_k,b_k,c_k,calibflag,arr,ctr);
		cout<<"no. of profiles = "<<*ctr<<endl;
      }
	  if(bON)
      {
        cout << "Set laser power to " << uiLaserPowerOFF << "\n";
        if((iRetValue = m_pLLT->SetFeature(FEATURE_FUNCTION_LASERPOWER, uiLaserPowerOFF)) < GENERAL_FUNCTION_OK)
        {
          OnError("Error during SetFeature(FEATURE_FUNCTION_LASERPOWER)", iRetValue);
          bOK = false;
        }
      }

      if(bConnected)
      {
        cout << "Disconnect the scanCONTROL\n";
        if((iRetValue = m_pLLT->Disconnect()) < GENERAL_FUNCTION_OK)
        {
          OnError("Error during Disconnect", iRetValue);
        }
      }
    }
  }

  //Deletes the LLT-object
  delete m_pLLT;

  //Wait for a keyboard hit
  //while(!_kbhit()) {}
  //return 0;
}

//void GetProfiles_Ethernet()
//{
//  int iRetValue;
//   unsigned int uiLaserPowerON = 0;
//  bool bOK=true;
//  std::vector<double> vdValueX(m_uiResolution);
//  std::vector<double> vdValueZ(m_uiResolution);
//  //Resize the profile buffer to the maximal profile size
//  std::vector<unsigned char> vucProfileBuffer(m_uiResolution*4+16);
//
//  cout << "\nDemonstrate the profile transfer via poll function\n";
//
//  cout << "Gets the type of the scanCONTROL (measurement range)\n";
//
//  cout << "Enable the measurement\n";
//  if(bOK)
//      {
//        cout << "Set laser power to " << uiLaserPowerON << "\n";
//        if((iRetValue = m_pLLT->SetFeature(FEATURE_FUNCTION_LASERPOWER, uiLaserPowerON)) < GENERAL_FUNCTION_OK)
//        {
//          OnError("Error during SetFeature(FEATURE_FUNCTION_LASERPOWER)", iRetValue);
//          bOK = false;
//        }
//      }
//  if((iRetValue = m_pLLT->TransferProfiles(NORMAL_TRANSFER, true)) < GENERAL_FUNCTION_OK)
//  {
//    OnError("Error during TransferProfiles", iRetValue);
//    return;
//  }
//
//  //Sleep for a while to warm up the transfer
//  Sleep(100);
//
//  //Gets 1 profile in "polling-mode" and PURE_PROFILE configuration
//  if((iRetValue = m_pLLT->GetActualProfile(&vucProfileBuffer[0], (unsigned int)vucProfileBuffer.size(), PURE_PROFILE, NULL))
//      != vucProfileBuffer.size())
//  {
//    OnError("Error during GetActualProfile", iRetValue);
//    return;
//  }
//  cout << "Get profile in polling-mode and PURE_PROFILE configuration OK \n";
//
//  cout << "Converting of profile data from the first reflection\n";
//  iRetValue = m_pLLT->ConvertProfile2Values(&vucProfileBuffer[0], m_uiResolution, PURE_PROFILE, m_tscanCONTROLType,
//    0, true, NULL, NULL, NULL, &vdValueX[0], &vdValueZ[0], NULL, NULL);
//  if(((iRetValue & CONVERT_X) == 0) || ((iRetValue & CONVERT_Z) == 0))
//  {
//    OnError("Error during Converting of profile data", iRetValue);
//    return;
//  }
//
//  DisplayProfile(&vdValueX[0], &vdValueZ[0], m_uiResolution);
//
//  cout << "\n\nDisplay the timestamp from the profile:";
//  DisplayTimestamp(&vucProfileBuffer[m_uiResolution*4]);
//
//  cout << "Disable the measurement\n";
//  if((iRetValue = m_pLLT->TransferProfiles(NORMAL_TRANSFER, false)) < GENERAL_FUNCTION_OK)
//  {
//    OnError("Error during TransferProfiles", iRetValue);
//    return;
//  }
//}


void GetProfiles_Ethernet(double *x_k,double *y_k,double *z_k,double *a_k,double *b_k,double *c_k,double *calibflag,double (*arr)[1000][1280][8], int *ctr)
{
  int iRetValue;
  std::vector<double> vdValueX(m_uiResolution);
  std::vector<double> vdValueZ(m_uiResolution);
  //Resize the profile buffer to the maximal profile size
  std::vector<unsigned char> vucProfileBuffer(m_uiResolution*4+16);
  
  tOld = clock();

  //cout << "\nDemonstrate the profile transfer via poll function\n";

  //cout << "Gets the type of the scanCONTROL (measurement range)\n";

 // cout << "Enable the measurement\n";
  //change SHOT_TRANSFER to NORMAL for data transfer mode 18/06/2015
  if((iRetValue = m_pLLT->TransferProfiles(NORMAL_TRANSFER, true)) < GENERAL_FUNCTION_OK)
  {
    OnError("Error during TransferProfiles", iRetValue);
    return;
  }

  //Sleep for a while to warm up the transfer
  Sleep(100);

  //for (int scan=0;*calibflag!=15;scan++)
for (*ctr=0;*calibflag!=6;(*ctr)++)

{
  //cout<<"profile number is"<<*ctr<<endl;
  //Gets 1 profile in "polling-mode" and PURE_PROFILE configuration
  if((iRetValue = m_pLLT->GetActualProfile(&vucProfileBuffer[0], (unsigned int)vucProfileBuffer.size(), PURE_PROFILE, NULL))
      != vucProfileBuffer.size())
  {
    OnError("Error during GetActualProfile", iRetValue);
    return;
  }
  //cout << "Get profile in polling-mode and PURE_PROFILE configuration OK \n";

  //cout << "Converting of profile data from the first reflection\n";
  iRetValue = m_pLLT->ConvertProfile2Values(&vucProfileBuffer[0], m_uiResolution, PURE_PROFILE, m_tscanCONTROLType,
    0, true, NULL, NULL, NULL, &vdValueX[0], &vdValueZ[0], NULL, NULL);
  if(((iRetValue & CONVERT_X) == 0) || ((iRetValue & CONVERT_Z) == 0))
  {
    OnError("Error during Converting of profile data", iRetValue);
    return;
  }

  DisplayProfile(&vdValueX[0], &vdValueZ[0], m_uiResolution,x_k,y_k,z_k,a_k,b_k,c_k,calibflag,arr,ctr); Sleep(12);
  }
  //cout << "\n\nDisplay the timestamp from the profile:";
  DisplayTimestamp(&vucProfileBuffer[m_uiResolution*4]);

  cout << "Disable the measurement\n";
  //change SHOT_TRANSFER to NORMAL for data transfer mode 18/06/2015
  if((iRetValue = m_pLLT->TransferProfiles(NORMAL_TRANSFER, false)) < GENERAL_FUNCTION_OK)
  {
    OnError("Error during TransferProfiles", iRetValue);
    return;
  }
  
}


//Displaying the error text
void OnError(const char* szErrorTxt, int iErrorValue)
{
  char acErrorString[200];

  cout << szErrorTxt << "\n";
  if(m_pLLT->TranslateErrorValue(iErrorValue, acErrorString, sizeof(acErrorString)) >= GENERAL_FUNCTION_OK)
    cout << acErrorString << "\n\n";
}

//Displays one profile
//void DisplayProfile(double *pdValueX, double *pdValueZ, unsigned int uiResolution)
//{
//  size_t tNumberSize;
//  
//  Point * points;
//points = new Point [uiResolution];
///*
//  double * x;
//x = new double [uiResolution];
//
//double * z;
//z = new double [uiResolution];
//*/
//  //double x[uiResolution],z[uiResolution];
//
//  for(unsigned int i=0; i<uiResolution; i++)
//  {
//    //Prints the X- and Z-values
//    tNumberSize = Double2Str(*pdValueX).size();
//
//	points[i].x=*pdValueX++;
//    cout << "\r" << "Profiledata: X = " << points[i].x;
//
//    for(; tNumberSize<8; tNumberSize++)
//    {
//      cout << " ";
//    }
//
//	tNumberSize = Double2Str(*pdValueZ).size();
//
//	points[i].z=*pdValueZ++;
//
//	cout << " Z = " << points[i].z;
//    for(; tNumberSize<8; tNumberSize++)
//    {
//      cout << " ";
//    }
//    
//    //Somtimes wait a short time (only for display)
//    if(i%8 == 0)
//    {
//      Sleep(10);
//    }
//  }
//  //return points;
//}

//Displays the timestamp
void DisplayProfile(double *pdValueX, double *pdValueZ, unsigned int uiResolution,double *x_k,double *y_k,double *z_k,double *a_k,double *b_k,double *c_k,double *calibflag,double (*arr)[1000][1280][8], int *ctr)
{
  //size_t tNumberSize;
  
  //fpi = fopen("data_19.txt","a");
  // Code for creating laser data file and reading
 // sprintf(profile_file,"data_%d.txt",file_cnt);
	//fpi = fopen(profile_file,"a") ;
	//cout<<"inside run profiler"<<endl;
	/*if(scan==1)
	{
	++file_cnt;
	}*/
	// File creation ends here

  //ofstream myfile;
  //myfile.open ("example.txt");
//for (int scan=0;scan<50;scan++)

 //save to file
 // for(unsigned int i=0; i<uiResolution; i++)
 // {
	//double xValue = *pdValueX++;
	//double zValue = *pdValueZ++;
	//fprintf(fpi,"%lf %lf %lf\n",xValue,*x_k,zValue);
 // }
 // fclose(fpi);

	//Display on screen
	
	//tNew = clock();
	//cout<<*x_k<<endl;
	for (unsigned int j = 0; j <1280; j++)
			{
				double xValue = *pdValueX++;
				double zValue = *pdValueZ++;
				//double xValue = *pdValueX++;
	//double zValue = *pdValueZ++;
	//printf("%lf %lf %lf\n",xValue,*x_k,zValue);
				//cout << "(" << xValue << ", "<<(((float)(tNew - tOld))/CLOCKS_PER_SEC)<<" , " << zValue << "), ";
				(*arr)[*ctr][j][0]=xValue;
				//(*arr)[*ctr][j][1]=*x_k;
				//(*arr)[*ctr][j][2]=zValue;
				(*arr)[*ctr][j][1]=zValue;
				(*arr)[*ctr][j][2]=*x_k;
				
				(*arr)[*ctr][j][3]=*y_k;
				(*arr)[*ctr][j][4]=*z_k;
				(*arr)[*ctr][j][5]=*a_k;
				(*arr)[*ctr][j][6]=*b_k;
				(*arr)[*ctr][j][7]=*c_k;

			}

	/*if(fTime<(((float)(tNew - tOld))/CLOCKS_PER_SEC))
					fTime = (((float)(tNew - tOld))/CLOCKS_PER_SEC);
				*/

				
	/*cout << (((float)(tNew - tOld))/CLOCKS_PER_SEC);
			cout << "\n"<<fTime<<endl;;*/
			//tOld = tNew;
			//m_pLLT->Timestamp2TimeAndCount(pucContainerBuffer1 + m_uiResolution * 64 - 16, NULL, NULL, &uiProfileCounter);

//			m_pLLT->Timestamp2TimeAndCount(pucContainerBuffer1 + m_uiResolution * 64 - 16, &pTimeShutterOpen,&pTimeShutterClose, &uiProfileCounter);

//			cout << "Profile count from the timestamp: " << pTimeShutterOpen << "\n\n";
}

void DisplayTimestamp(unsigned char *pucTimestamp)
{
  double dShutterOpen, dShutterClose;
  unsigned int uiProfileCount;

  //Decode the timestamp
  m_pLLT->Timestamp2TimeAndCount(pucTimestamp, &dShutterOpen, &dShutterClose, &uiProfileCount);
  cout << "\nShutterOpen: " << dShutterOpen << " ShutterClose: " << dShutterClose << "\n";
  cout << "ProfileCount: " << uiProfileCount << "\n";
  cout << "\n";
}

//Convert a double value to a string
std::string Double2Str(double dValue)
{
  std::ostringstream NewStreamApp;
  NewStreamApp << dValue;

  return NewStreamApp.str();
}
