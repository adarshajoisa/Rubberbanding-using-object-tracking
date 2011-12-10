#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <math.h>
#include <iostream>

//definitions of ROI values
#define centerX 270
#define centerY 210
#define ROISTARTX 310
#define ROISTARTY 230
#define ROIENDX 330
#define ROIENDY 250
#define ROIHEIGHT 20
#define ROIWIDTH 20

struct centroidposition
{
  int x;
  int y;
};

  centroidposition pos1[10];
  centroidposition pos2[10];
  int curtop1	 = 0;
  int curtop2 = 0;
  const int qmaxlen = 10;
  int fullcount = 0, partialcount = 0;

using namespace std;


void InRangeS(IplImage * img, CvScalar hsv_min, CvScalar hsv_max, IplImage* imgThreshed, centroidposition* pos, int curtop)
{
      IplImage * imgHSV = 0;
      imgHSV = cvCloneImage(img);
      cvCvtColor(imgHSV, imgHSV, CV_BGR2HSV);
//   imgThreshed = cvCreateImage(cvGetSize(imgHSV), 8, 1);
  if(imgHSV == NULL || imgThreshed == NULL)
  {
    cout<<"Null pointer encountered. Aborting..."<<endl;
    exit(1);
  }
  
  //Get the previous direction of object movement.
      int sidedirection = 0, updirection = 0;
      int xIncrement = 0, yIncrement = 0;
      if( pos[(curtop + 4 ) % 10].x > pos[curtop].x + 5)
      {
	sidedirection = -1;
	xIncrement = (pos[(curtop + 4) % 10].x - pos[curtop].x ) / 3;
      }
      
      else if( pos[(curtop + 4 ) % 10].x < pos[curtop].x - 5)
      {
	sidedirection = 1;
	xIncrement = (pos[curtop].x - pos[(curtop + 4) % 10].x) / 3;
      }
      
      else 
      {
	sidedirection = 0;
	xIncrement = 0;
      }

      
      
      if( pos[(curtop + 4 ) % 10].y > pos[curtop].y + 5)
      {
	yIncrement = (pos[(curtop + 4) % 10].y - pos[curtop].y ) / 3;
	updirection = 1;
      }
      
      else if( pos[(curtop + 4 ) % 10].y < pos[curtop].y - 5)
      {
	yIncrement = (pos[curtop].y - pos[(curtop + 4) % 10].y) / 3;
	updirection = -1;
      }
      
      else 
      {
	updirection = 0;
	yIncrement = 0;
      }
  
  
  //Set ROI for thresholding based on the previous position and direction of movement
      int ROIx;
      if( sidedirection == 0)
	ROIx = pos[curtop].x - (xIncrement - 100);
      else if(sidedirection == -1)
	ROIx = pos[curtop].x - (xIncrement - 100);
      else
	ROIx = pos[curtop].x - (xIncrement - 100);
      
      if(ROIx < 0)
	ROIx = 0;
      if(ROIx > 440)
	ROIx = 440;
      
      
      int ROIy;
      if( updirection == 0)
	ROIy = pos[curtop].y - (yIncrement - 100);
      else if(updirection == -1)
	ROIy = pos[curtop].y - (yIncrement - 100);
      else
	ROIy = pos[curtop].y - (yIncrement - 100);
      
      if(ROIy < 0)
	ROIy = 0;
      if(ROIy > 280)
	ROIy = 280;
      
      cvSetImageROI( imgHSV, cvRect(ROIx, ROIy, 200, 200));

  //Initialize the threshold image ( imgThreshed ) to black
  for(int y = 0; y < imgThreshed->height; y++)
  {
    uchar* tempthreshptr = (uchar*) ( imgThreshed->imageData + y * imgThreshed->widthStep );
    for(int x = 0; x < imgThreshed->width; x++)
      tempthreshptr[x] = 0x00;
  }
  
  
  //Threshold the ROI and write to imgThreshed
  int whitecount = 0;
  for( int y=ROIy; y<(ROIy + 150); y++ ) 
  {
    uchar* ptr = (uchar*) ( imgHSV->imageData + y * imgHSV->widthStep );
    uchar* threshptr = (uchar*) ( imgThreshed->imageData + y * imgThreshed->widthStep );
    for( int x=ROIx; x<(ROIx + 150); x++ ) 
    {
      int prevline = x - imgHSV->width;
      if( prevline < 0 )
	prevline = 0;
      if( (ptr[3*x] > hsv_min.val[0] && ptr[3*x] < hsv_max.val[0]) && 
	  (ptr[3*x + 1] > hsv_min.val[1] && ptr[3*x + 1] < hsv_max.val[1]) &&
	  (ptr[3*x + 2] > hsv_min.val[2] && ptr[3*x + 2] < hsv_max.val[2]))
      {
	//threshold
	threshptr[x] = 0xff;
	whitecount++;
      }
      else
      {
	threshptr[x] = 0x00;
      }
    }
  }
  
  if(whitecount <  50 )
  {
    whitecount = 0;
    for( int y=ROIy; y<(ROIy + 150); y++ ) 
    {
      uchar* ptr = (uchar*) ( imgHSV->imageData + y * imgHSV->widthStep );
      uchar* threshptr = (uchar*) ( imgThreshed->imageData + y * imgThreshed->widthStep );
      for( int x=ROIx; x<(ROIx + 150); x++ ) 
      {
	int prevline = x - imgHSV->width;
	if( prevline < 0 )
	  prevline = 0;
	if( (ptr[3*x] > (hsv_min.val[0] - 5) && ptr[3*x] < (hsv_max.val[0] + 5)) && 
	    (ptr[3*x + 1] > (hsv_min.val[1] - 10) && ptr[3*x + 1] < (hsv_max.val[1] + 10)) &&
	    (ptr[3*x + 2] > (hsv_min.val[2] - 10) && ptr[3*x + 2] < (hsv_max.val[2] + 10)))
	{
	  //threshold
	  threshptr[x] = 0xff;
	  whitecount++;
	}
	else
	{
	  threshptr[x] = 0x00;
	}
      }
    }
  }
  
  cvResetImageROI(imgHSV);
//   cout<<whitecount<<endl;

  //If object was detected in this ROI, return. Else threshold the entire image.
  if(whitecount > 50)
  {
    partialcount++;
    return;
  }
  else
  {
    fullcount++;
    for( int y=0; y<imgThreshed->height; y++ ) 
    {
      uchar* ptr = (uchar*) ( imgHSV->imageData + y * imgHSV->widthStep );
      uchar* threshptr = (uchar*) ( imgThreshed->imageData + y * imgThreshed->widthStep );
      for( int x=0; x<imgThreshed->width; x++ ) 
      {
	int prevline = x - imgThreshed->width;
	if( prevline < 0 )
	  prevline = 0;
	if( (ptr[3*x] > hsv_min.val[0] && ptr[3*x] < hsv_max.val[0]) && 
	    (ptr[3*x + 1] > hsv_min.val[1] && ptr[3*x + 1] < hsv_max.val[1]) &&
	    (ptr[3*x + 2] > hsv_min.val[2] && ptr[3*x + 2] < hsv_max.val[2]))
	{
	  //threshold
	  threshptr[x] = 0xff;
	}
	else if( ( threshptr[x-1] == 0xff  || threshptr[prevline] == 0xff)
		  &&
		  ( (ptr[3*x] > hsv_min.val[0] - 5 && ptr[3*x] < hsv_max.val[0] + 5) && 
		    (ptr[3*x + 1] > hsv_min.val[1] - 10 && ptr[3*x + 1] < hsv_max.val[1] + 10) &&
		    (ptr[3*x + 2] > hsv_min.val[2] - 10 && ptr[3*x + 2] < hsv_max.val[2] + 10)))
	{
	  //threshold
	  threshptr[x] = 0xff;
	}
	else
	{
	  threshptr[x] = 0x00;
	}
      }
    }
  }
  cvReleaseImage(&imgHSV);
}



IplImage* GetThresholdedImage(IplImage* img, CvScalar hsv_min, CvScalar hsv_max, centroidposition * pos, int curtop)
{
  IplImage * imgHSV = 0;
  imgHSV = cvCloneImage(img);
  cvCvtColor(imgHSV, imgHSV, CV_BGR2HSV);
  IplImage* imgThreshed = cvCreateImage(cvGetSize(imgHSV), 8, 1);
  cvSmooth( imgHSV, imgHSV, CV_BLUR, 5, 0, 0, 0 );
//   cvInRangeS(imgHSV, hsv_min, hsv_max, imgThreshed);
  InRangeS(imgHSV, hsv_min, hsv_max, imgThreshed, pos, curtop);
  return imgThreshed;
}



void getHSVRanges(IplImage * frame, CvScalar *HSVranges)
{
  
      cvSetImageROI(frame, cvRect(ROISTARTX, ROISTARTY, ROIHEIGHT, ROIWIDTH));
    
    IplImage *sub_img = cvCreateImageHeader(cvSize( ROIHEIGHT, ROIWIDTH ), frame->depth, frame->nChannels);
    sub_img->origin = frame->origin;
    sub_img->widthStep = frame->widthStep;
    sub_img->imageData = frame->imageData +
			 ROISTARTY * frame->widthStep +
			 ROISTARTX * frame->nChannels;
    
    CvScalar hsv_avg, hsv_sdv;
    
    //Get the average and standard deviation of HSV values for the recognized object
    cvAvgSdv(frame, &hsv_avg, &hsv_sdv, 0);
    
	    
	    int Hbuckets[256];
	    int Sbuckets[256];
	    int Vbuckets[256];
	    int Hbucketindex = 0;
	    int Sbucketindex = 0;
	    int Vbucketindex = 0;

	    for(int i = 0; i < 256; i++)
	    {
	      Hbuckets[i] = 0;
	      Sbuckets[i] = 0;
	      Vbuckets[i] = 0;
	    }
	    for( int y=0; y<sub_img->height; y++ ) 
	    {
	      uchar* ptr = (uchar*) ( sub_img->imageData + y * sub_img->widthStep );
	      for( int x=0; x<sub_img->width; x++ ) 
	      {
		Hbucketindex = (int)ptr[3*x];
		Hbuckets[Hbucketindex] ++;
		Sbucketindex = (int)ptr[3*x + 1];
		Sbuckets[Sbucketindex] ++;
		Vbucketindex = (int)ptr[3*x + 2];
		Vbuckets[Vbucketindex] ++;
	      }
	    }
	    
	    int Hmaxbucket = 0;
	    int Hmaxval = 0;
	    int Smaxbucket = 0;
	    int Smaxval = 0;
	    int Vmaxbucket = 0;
	    int Vmaxval = 0;
	    for(int i = 0; i < 256; i++)
	    {
// 	      cout<<Hbuckets[i]<<" ";
	      if(Hbuckets[i] > Hmaxval)
	      {
		Hmaxval = Hbuckets[i];
		Hmaxbucket = i;
	      }
	    }
	    
	    for(int i = 0; i < 256; i++)
	    {
// 	      cout<<Sbuckets[i]<<" ";
	      if(Sbuckets[i] > Smaxval)
	      {
		Smaxval = Sbuckets[i];
		Smaxbucket = i;
	      }
	    }
	    
	    for(int i = 0; i < 256; i++)
	    {
// 	      cout<<Vbuckets[i]<<" ";
	      if(Vbuckets[i] > Vmaxval)
	      {
		Vmaxval = Vbuckets[i];
		Vmaxbucket = i;
	      }
	    }
	
//Code to find the longest sequence of non-zero pixel values

	    int Hseqcount = 0, Hmaxseqcount = 0;
	    int Hindex = 0;
	    int Hmaxindex = 0;
	    int Hflag = 0;
	    
	    for(int i = 0; i < 256; i++)
	    {
	      if(Hbuckets[i] == 0)
	      {
		if(Hflag == 0)
		  Hflag = 1;
		else if(Hflag == 1)
		{
		  if(Hseqcount > Hmaxseqcount)
		  {
		    Hmaxseqcount = Hseqcount;
		    Hmaxindex = Hindex;
		    
		  }
		  Hseqcount = 0;
		}
	      }
	      
	      else if(Hbuckets[i] > 0)
	      {
		if(Hseqcount == 0)
		  Hindex = i;
		Hflag = 0;
		Hseqcount++;
	      }
	    }
	    
	    if(Hseqcount > Hmaxseqcount)
	    {
	      Hmaxseqcount = Hseqcount;
	      Hmaxindex = Hindex;
	      
	    }
	    int Hrangestart = Hmaxindex;
	    int Hrangeend = Hmaxindex + Hmaxseqcount;
// 	    cout<<"HStart : "<<Hrangestart<<" ; End : "<<Hrangeend<<endl;
	    
	    int Htotalrange = Hrangeend - Hrangestart;
	    int Hn = (Htotalrange / 10) + 1;
	    int Hrangebuckets[Hn];
	    for(int i = 0; i < Hn; i++ )
	      Hrangebuckets[i] = 0;
	    int Hrangebucketcount = 0;
	    int j = 0;
	    for(int i = Hrangestart; i <= Hrangeend; i++)
	    {
	      Hrangebuckets[j] += Hbuckets[i];
	      Hrangebucketcount++;
	      if(Hrangebucketcount == 9)
	      {
		j++;
		Hrangebucketcount = 0;
	      }
	    }
	    
	    int Hmaxrangebucket = 0;
	    int Hmaxrangestart = 0;
	    for(int i = 0; i < Hn; i++)
	    {
	      if(Hrangebuckets[i] > Hmaxrangebucket)
	      {
		Hmaxrangebucket = Hrangebuckets[i];
		Hmaxrangestart = i;
	      }
	    }
	    
	    
	    
	    
	    int Sseqcount = 0, Smaxseqcount = 0;
	    int Sindex = 0;
	    int Smaxindex = 0;
	    int Sflag = 0;
	    for(int i = 0; i < 256; i++)
	    {
	      if(Sbuckets[i] == 0)
	      {
		if(Sflag == 0)
		{
		  Sflag = 1;
		}
		else if(Sflag == 1)
		{
		  if(Sseqcount > Smaxseqcount)
		  {
		    Smaxseqcount = Sseqcount;
		    Smaxindex = Sindex;
		    
		  }
		  Sseqcount = 0;
		}
	      }
	      
	      else if(Sbuckets[i] > 0)
	      {
		if(Sseqcount == 0)
		{
		  Sindex = i;
		}
		Sflag = 0;
		Sseqcount++;
	      }
	    }
	    
	    if(Sseqcount > Smaxseqcount)
	    {
	      Smaxseqcount = Sseqcount;
	      Smaxindex = Sindex;
	      
	    }
	    
	    int Srangestart = Smaxindex;
	    int Srangeend = Smaxindex + Smaxseqcount;
// 	    cout<<"SStart : "<<Srangestart<<" ; End : "<<Srangeend<<endl;
	    
	    int Stotalrange = Srangeend - Srangestart;
	    int Sn = (Stotalrange / 10) + 1;
	    int Srangebuckets[Sn];
	    for(int i = 0; i < Sn; i++ )
	      Srangebuckets[i] = 0;
	    int Srangebucketcount = 0;
	    j = 0;
	    for(int i = Srangestart; i <= Srangeend; i++)
	    {
	      Srangebuckets[j] += Sbuckets[i];
	      Srangebucketcount++;
	      if(Srangebucketcount == 9)
	      {
		j++;
		Srangebucketcount = 0;
	      }
	    }
	    
	    int Smaxrangebucket = 0;
	    int Smaxrangestart = 0;
	    for(int i = 0; i < Sn; i++)
	    {
	      if(Srangebuckets[i] > Smaxrangebucket)
	      {
		Smaxrangebucket = Srangebuckets[i];
		Smaxrangestart = i;
	      }
	    }
	    
	    int Vseqcount = 0, Vmaxseqcount = 0;
	    int Vindex = 0;
	    int Vmaxindex = 0;
	    int Vflag = 0;
	    for(int i = 0; i < 256; i++)
	    {
	      if(Vbuckets[i] == 0)
	      {
		if(Vflag == 0)
		  Vflag = 1;
		else if(Vflag == 1)
		{
		  if(Vseqcount > Vmaxseqcount)
		  {
		    Vmaxseqcount = Vseqcount;
		    Vmaxindex = Vindex;
		  }
		  Vseqcount = 0;
		}
	      }
	      
	      else if(Vbuckets[i] > 0)
	      {
		if(Vseqcount == 0)
		  Vindex = i;
		Vflag = 0;
		Vseqcount++;
	      }
	    }
	    
	    if(Vseqcount > Vmaxseqcount)
	    {
	      Vmaxseqcount = Vseqcount;
	      Vmaxindex = Vindex;
	    }
	    
	    int Vrangestart = Vmaxindex;
	    int Vrangeend = Vmaxindex + Vmaxseqcount;
// 	    cout<<"VStart : "<<Vrangestart<<" ; End : "<<Vrangeend<<endl;
	    
	    int Vtotalrange = Vrangeend - Vrangestart;
	    int Vn = (Vtotalrange / 10) + 1;
	    int Vrangebuckets[Vn];
	    for(int i = 0; i < Vn; i++ )
	      Vrangebuckets[i] = 0;
	    int Vrangebucketcount = 0;
	    j = 0;
	    for(int i = Vrangestart; i <= Vrangeend; i++)
	    {
	      Vrangebuckets[j] += Vbuckets[i];
	      Vrangebucketcount++;
	      if(Vrangebucketcount == 9)
	      {
		j++;
		Vrangebucketcount = 0;
	      }
	    }
	    
	    int Vmaxrangebucket = 0;
	    int Vmaxrangestart = 0;
	    for(int i = 0; i < Vn; i++)
	    {
	      if(Vrangebuckets[i] > Vmaxrangebucket)
	      {
		Vmaxrangebucket = Vrangebuckets[i];
		Vmaxrangestart = i;
	      }
	    }
	    
    cvResetImageROI(frame);
    double avg_val[4], sdv_val[4];
    for(int i = 0; i < 4; i++)
    {
      avg_val[i] = hsv_avg.val[i];
      sdv_val[i] = hsv_sdv.val[i];
    }
        
     // Values 20,100,100 to 30,255,255 working perfect for ping pong ball at around 6pm
     //Set min and max values for the object to be tracked
    double min[4];
    min[0] = Hrangestart; //*/avg_val[0] - hsv_sdv.val[0];
    min[1] = Srangestart;// - 50; //*/avg_val[1] - hsv_sdv.val[1] -50;
    min[2] = Vrangestart;// - 25; //*/avg_val[2] - hsv_sdv.val[2] - 50;
    min[3] = avg_val[3] - hsv_sdv.val[3];
    if(min[1] < 0)
      min[1] = 0.00;
    if(min[2] < 0)
      min[2] = 0.00;
    
    
    
    HSVranges[0] = cvScalar( min[0], min[1], min[2], min[3] );
//     CvScalar hsv_min = cvScalar( 0, 200, 200, 0 );
      
      
    double max[4];
    max[0] = Hrangeend; //*/avg_val[0] + hsv_sdv.val[0];
    max[1] = 255;//Srangeend;// + 50; //*/avg_val[1] + hsv_sdv.val[1] + 50; 
    max[2] = 255;//Vrangeend;// + 50; //*/avg_val[2] + hsv_sdv.val[2] + 50;
    max[3] = avg_val[3] + hsv_sdv.val[3];
    if(max[1] > 255)
      max[1] = 255.00;
    if(max[2] > 255)
      max[2] = 255.00;
    HSVranges[1] = cvScalar( max[0], max[1], max[2], max[3] );
    
}



int main()
{
  CvCapture* capture = 0;
  capture = cvCaptureFromCAM(0);	
  
  

  int framecount = 0;
  
  if(!capture)
  {
    printf("Could not start capturing from camera...\n");
    return -1;
  }
  
  
  
    cvNamedWindow("video");
    IplImage* frame = 0;
    IplImage* RGBframe = 0;
    while(true)
    {
      frame = cvQueryFrame(capture);
//       IplImage* frame = cvCreateImage(cvGetSize(RGBframe), 8, 3);
//       cvCvtColor(frame, frame, CV_BGR2HSV);
      //drawing the yellow rectangle affects the color values in the ROI. So, drawing the rectangle just outside the ROI
      cvRectangle(frame, cvPoint(ROISTARTX - 1, ROISTARTY - 1), cvPoint(ROIENDX + 1, ROIENDY + 1), cvScalar(0, 255, 255));
      cvFlip(frame, frame, 1);
//       cvCvtColor(frame, frame, CV_HSV2BGR);
      cvShowImage("video", frame);
//       cvCvtColor(frame, frame, CV_BGR2HSV);
      cvFlip(frame, frame, 1);
      int input = cvWaitKey(1);
//       cout<<input<<endl;
      if((input % 256 )== 32)
	break;
    }
    
    //Now we got the frame with the object to be tracked in the rectangle. We need to get the HSV value range for the object.
    CvScalar HSVranges1[2];
    getHSVRanges(frame, HSVranges1);
 
    
    cout<<"1. HSV MIN: "<<HSVranges1[0].val[0]<<" "<<HSVranges1[0].val[1]<<" "<<HSVranges1[0].val[2]<<" "<<endl;
    cout<<"1. HSV MAX: "<<HSVranges1[1].val[0]<<" "<<HSVranges1[1].val[1]<<" "<<HSVranges1[1].val[2]<<" "<<endl<<endl;
    
     while(true)
    {
      frame = cvQueryFrame(capture);
//       IplImage* frame = cvCreateImage(cvGetSize(RGBframe), 8, 3);
//       cvCvtColor(frame, frame, CV_BGR2HSV);
      //drawing the yellow rectangle affects the color values in the ROI. So, drawing the rectangle just outside the ROI
      cvRectangle(frame, cvPoint(ROISTARTX - 1, ROISTARTY - 1), cvPoint(ROIENDX + 1, ROIENDY + 1), cvScalar(0, 255, 255));
      cvFlip(frame, frame, 1);
//       cvCvtColor(frame, frame, CV_HSV2BGR);
      cvShowImage("video", frame);
//       cvCvtColor(frame, frame, CV_BGR2HSV);
      cvFlip(frame, frame, 1);
      int input = cvWaitKey(1);
//       cout<<input<<endl;
      if((input % 256 )== 32)
	break;
    }
    
    //Now we got the frame with the object to be tracked in the rectangle. We need to get the HSV value range for the object.
    CvScalar HSVranges2[2];
    getHSVRanges(frame, HSVranges2);
 
        
    cout<<"2. HSV MIN: "<<HSVranges2[0].val[0]<<" "<<HSVranges2[0].val[1]<<" "<<HSVranges2[0].val[2]<<" "<<endl;
    cout<<"2. HSV MAX: "<<HSVranges2[1].val[0]<<" "<<HSVranges2[1].val[1]<<" "<<HSVranges2[1].val[2]<<" "<<endl<<endl;
 
  int left = 0, right = 0, up = 0, down = 0;
  char ch;
  sleep(1);

  while(true)
  {
    frame = cvQueryFrame(capture);
//     cvCvtColor(frame, frame, CV_BGR2HSV);
    framecount++;

    if(!frame)
      break;
    

    IplImage* imgThresh1 = GetThresholdedImage(frame, HSVranges1[0], HSVranges1[1], pos1, curtop1);
    IplImage* imgThresh2 = GetThresholdedImage(frame, HSVranges2[0], HSVranges2[1],pos2, curtop2);

    // Calculate the moments to estimate the position of the object
    CvMoments *moments1 = (CvMoments*)malloc(sizeof(CvMoments));
    cvMoments(imgThresh1, moments1, 1);

    // The actual moment values
    double moment101 = cvGetSpatialMoment(moments1, 1, 0);
    double moment011 = cvGetSpatialMoment(moments1, 0, 1);
    double area1 = cvGetCentralMoment(moments1, 0, 0);

    // Holding the last and current object positions
    static int posX1 = 100;
    static int posY1 = 100;

    int lastX1 = posX1;
    int lastY1 = posY1;

    posX1 = moment101/area1;
    posY1 = moment011/area1;
    
    pos1[curtop1].x = posX1;
    pos1[curtop1].y = posY1;
    curtop1 = (curtop1 + 1) % 10;
    
//     cout<<posX<<" "<<posY<<endl;
   
    if(posX1 <= 0)
      posX1 = lastX1;
    if(posY1 <= 0)
      posY1 = lastY1;
 
    //invert the sideways movement to get a mirror image of movement
    posX1 = 640 - posX1;		//the camera doesn't give a mirror image. We need a mirror image to get the left-right directions correct.

    
    
    
    
    CvMoments *moments2 = (CvMoments*)malloc(sizeof(CvMoments));
    cvMoments(imgThresh2, moments2, 1);

    // The actual moment values
    double moment102 = cvGetSpatialMoment(moments2, 1, 0);
    double moment012 = cvGetSpatialMoment(moments2, 0, 1);
    double area2 = cvGetCentralMoment(moments2, 0, 0);

    // Holding the last and current object positions
    static int posX2 = 100;
    static int posY2 = 100;

    int lastX2 = posX2;
    int lastY2 = posY2;

    posX2 = moment102/area2;
    posY2 = moment012/area2;
    
    pos2[curtop2].x = posX2;
    pos2[curtop2].y = posY2;
    curtop2 = (curtop2 + 1) % 10;
    
//     cout<<posX<<" "<<posY<<endl;
   
    if(posX2 <= 0)
      posX2 = lastX2;
    if(posY2 <= 0)
      posY2 = lastY2;
 
    //invert the sideways movement to get a mirror image of movement
    posX2 = 640 - posX2;		//the camera doesn't give a mirror image. We need a mirror image to get the left-right directions correct.

//     cout<<posX1<<" "<<posY1<<"     "<<posX2<<" "<<posY2<<endl;
    cvRectangle(frame, cvPoint(posX1, posY1), cvPoint(posX2, posY2), cvScalar(40, 10, 255));
    cvShowImage("thresh1", imgThresh1);
    cvShowImage("thresh2", imgThresh2);
    cvFlip(frame, frame, 1);
//     cvCvtColor(frame, frame, CV_HSV2BGR);
    cvShowImage("video", frame);
    cvFlip(frame, frame, 1);

    int c = cvWaitKey(10);

    if(c!=-1)
    {
      if( (c % 256) == 27 )
	break;
      if( (c % 256) == ' ' )
      {
	CvScalar TestHSVranges[2];
	getHSVRanges(frame, TestHSVranges);
	cout<<"HSV MIN: "<<TestHSVranges[0].val[0]<<" "<<TestHSVranges[0].val[1]<<" "<<TestHSVranges[0].val[2]<<" "<<endl;
	cout<<"HSV MAX: "<<TestHSVranges[1].val[0]<<" "<<TestHSVranges[1].val[1]<<" "<<TestHSVranges[1].val[2]<<" "<<endl<<endl;

// 	system("xdotool mousedown 1");
      }
//       else
//       {
// 	system("xdotool mouseup 1");
//       }
    }

    cvReleaseImage(&imgThresh1);
    cvReleaseImage(&imgThresh2);
    delete moments1;
    delete moments2;

  }
  cout<<partialcount<<" frames have been processed using the optimized thresholding."<<endl;
  cout<<fullcount<<" frames have been processed using full thresholding."<<endl;
  double percentage = (float)((float)(partialcount)/(float)(partialcount + fullcount)) * 100;
  cout<<percentage<<"% accuracy of position prediction."<<endl<<endl;
//   system("clear");
  cvReleaseCapture(&capture);
//   cvReleaseImage(&frame);
  cvDestroyAllWindows();
  return 0;
}