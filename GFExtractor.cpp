/*
 * JRGT.cpp
 *
 *  Created on: 16 juin 2011
 *      Author: raoui
 */


#include "JRGT.h"
#include <stdlib.h>
#include <cv.h>
#include <highgui.h>
#define pi 22/7

CvMat * kernel=0;
//typedef vector<CvMat*> Mat;
		CDisplayWindowPlots winEKF("Tracking - Extended Kalman Filter",450,400);


		typedef vector<string> Vec_place;
		typedef vector<Vec_place> Mat_place;

typedef vector<double> Vec;
typedef vector<Vec> Mat;

Mat_place mp;
int hasht[7][10];


JRGT::JRGT() {
	// TODO Auto-generated constructor stub
line_size=0;
}

JRGT::~JRGT() {
	// TODO Auto-generated destructor stub

}

IplImage* JRGT::descriptorGauss(IplImage *src){
	IplImage* dst = cvCreateImage( cvGetSize(src), IPL_DEPTH_8U, 3 );;
	 cvSmooth( src, dst,CV_GAUSSIAN,3, 0 );

	return dst;
}
:
vector<IplImage*> JRGT::load_image_db(){
IplImage *im;
	vector<IplImage*> ims;
	int i;
	char buffer[20];
	for (i=1;i<63;i++)
	{
		sprintf(buffer,  "rooms/im%d.png",i);
		im=cvLoadImagee(buffer,1);
		ims.push_back(im);

	}
return ims;

	}




double JRGT::euclidean(vector<string> v1,vector<double> v2){
	double dist=0.0;

	for(int i=0;i<v2.size();i++){
		double  n = atof (v1.at(i+3).c_str() );
		dist+=pow((n-v2.at(i)),2);
	}
		dist=sqrt(dist);
	return dist;
}



double JRGT::euclidean_double(vector<double> v1,vector<double> v2){
	double dist=0.0;


	for(int i=2;i<v1.size();i++){

		dist+=pow((v1.at(i)-v2.at(i)),2);
	}
		dist=sqrt(dist);
	return dist;
}



IplImage* JRGT::descriptorGabor(IplImage* im,float phase,float psi,float scale){

	    //CvFileStorage *fs = cvOpenFileStorage("kernel.xml",NULL,CV_STORAGE_WRITE);
	kernel = cvCreateMat(21,21,CV_32FC1);
	 IplImage* src = 0;
	  IplImage* src_f = 0;
	  IplImage* image = 0;
	  IplImage* dest = 0;
	  IplImage* dest_mag = 0;
	  IplImage* kernelimg=0;
	  IplImage* big_kernelimg=0;

	  int kernel_size =21;
	  int pos_var = 50;
	  int pos_w = 5;
	  int pos_phase = 0;
	  int pos_psi = 90;



	  int x,y;
	    float kernel_val;
	    float var = (float)pos_var/10;
	    float w = (float)pos_w/10;
	    //float phase = (float) pos_phase*CV_PI/180;
	    //float psi = CV_PI*pos_psi/180;
	    cvZero(kernel);
	    for (x = -kernel_size/2;x<=kernel_size/2; x++) {
	      for (y = -kernel_size/2;y<=kernel_size/2; y++) {
	        kernel_val = scale*exp( -((x*x)+(y*y))/(2*var))*cos( w*x*cos(phase)+w*y*sin(phase)+psi);
	        cvSet2D(kernel,y+kernel_size/2,x+kernel_size/2,cvScalar(kernel_val));
	     // cvSet2D(kernelimg,y+kernel_size/2,x+kernel_size/2,cvScalar(kernel_val/2+0.5));
	  //      printf("%f\n",kernel_val);
	      }
	    }
	    //cvWrite( fs, "kernel", kernel, cvAttrList(0,0) );
	    //cvReleaseFileStorage(&fs);
	dest= cvCreateImage(cvSize(im->width, im->height), IPL_DEPTH_8U ,3);

	    cvFilter2D(im, dest,kernel,cvPoint(-1,-1));
//	   cvShowImage("Process window",dest);
//	    cvResize(kernelimg,big_kernelimg);
	//    cvShowImage("Kernel",big_kernelimg);
	    //cvPow(dest,dest_mag,2);
	  //  cvPow(dest_mag,dest_mag,0.5);
	    //cvShowImage("Mag",dest_mag);
	//    cvWaitKey(0);
return dest;


}



vector<double> JRGT::node_matching(Mat_place mp,Vec vecteur){
JRGT gf;
	 vector<string> vec_1D;
vector<double>distances;


	 for (int j=0;j<line_size;j++){
	 vec_1D=mp.at(j);

distances.push_back(gf.euclidean(vec_1D,vecteur));
	 }
return distances;

}

Vec JRGT::globalComputationFeaturess(IplImage* image){
JRGT gf;
	float scale[5]={4.0,3.5,2.95,2.35,1.75};
		float central_freq[5]={0.05,0.08,0.14,0.22,0.33};
		float orient[4]={-pi/4,-pi/2,pi/4,pi/2};
		double tags[2][10]={{-1,-1,3,-4,3,-2,2,5,9},{1,4,5,8,7,4,2,6,8}};

	IplImage* measures;
	CvMat* M = cvCreateMat(2,100,CV_32FC1);

	Vec vecteur,vecteur0;

	IplImage* array;
	CvScalar MeanScalar;
	CvScalar StandardDeviationScalar;
	int k=0,ind=0;
	int i,j,im_i;
	vector<CvMat*>  global_features;


			array= gf.descriptorGauss(image);
				ind=0;
				for (i=0;i<5;i++)
					for(j=0;j<5;j++)
						for(k=0;k<4;k++){
							measures=descriptorGabor(array,central_freq[i],scale[j],orient[k]);
							cvAvgSdv(measures,&MeanScalar,&StandardDeviationScalar);
							vecteur.push_back(MeanScalar.val[0]);
							vecteur.push_back(StandardDeviationScalar.val[0]);
									ind++;

						}


return vecteur;


	}


int main(){

return 0;


}



