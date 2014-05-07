#include <iostream>
#include <vector.h>
#include <cv.h>
#include <fstream>
#include <highgui.h>
#include <iostream>
#include "Vision.h"
#include <string>
#include <eigen3/Eigen/Dense>
using namespace std;
using namespace Eigen;
typedef vector<string> Vec;
typedef vector<Vec> Mat;



/*


MatrixXd rotation_camera_robot(float thetak){
MatrixXd Rgr(3,3);
Rgr(0,0)=cos(thetak);
Rgr(1,0)=sin(thetak);
Rgr(0,1)=-sin(thetak);
Rgr(1,1)=cos(thetak);
return Rgr;
}


MatrixXd rotation_g_robot(float thetak){
MatrixXd Rgr(3,3);
Rgr(0,0)=cos(thetak);
Rgr(1,0)=sin(thetak);
Rgr(0,1)=-sin(thetak);
Rgr(1,1)=cos(thetak);
return Rgr;
}


MatrixXd Rcr(float theta){

MatrixXd R(3,3);
R(0,0)=cos(theta);
R(0,1)=sin(theta);
R(0,0)=0;
R(1,0)=-sin(theta);
R(0,0)=cos(theta);
R(0,0)=0;
R(0,0)=0;
R(0,0)=0;
R(0,0)=1;

 return R;


}

*/
/*
Eigen::VectorXd Prj2Dto3D(VectorXd X){
        Eigen::MatrixXd RW(3,3);
   
Eigen::VectorXd hRL(3),h(3);
int fku=1.95;
int fkv=1.95;
int u0=162;
int v0=125;
float u,v;
        RW(0,0)=0.1;
        RW(0,1)=0.1;
        RW(0,2)=0.1;
        RW(1,0)=0.1;
        RW(1,1)=0.1;
        RW(1,2)=0.1;
        RW(2,0)=0.1;
        RW(2,1)=0.1;
        RW(2,2)=0.1;
        hRL= RW*(fx-X);
        u=u0-fku*hRL(0)/hRL(2);
        v=v0-fkv*hRL(1)/hRL(2);
        h(0)=u;
        h(1)=v;
        return h;
    }
*/
/*
VectorXd position_feature_Camera(VectorXd X){}

VectorXd Pfc(3);
VectorXd Cpr(3);
Cpr(0)=tx;
Cpr(1)=ty;
Cpr(2)=tz;
VectorXd Gprk(3);
Gprk(0)=X(0);
Gprk(1)=X(1);
Gprk(2)=h;

MatrixXd R(3,3);
R=rotation_g_robot(thetak);
Pfc=Rcr(theta)*R.tr()*(Prj2Dto3D(X)-Gprk)+Gprk;
return Pfc;

}

*/
int Vision::BriscCompute(IplImage* imageStr){
        char* str_image;
	char *str_detector;
        str_image="Brisc_db.txt";
	str_detector="Brisc_detect.txt";
        int i;
        CvMat *image1=0;
           ofstream outputFile(str_image);
	   ofstream   outputDetector(str_detector);
         static CvScalar red_color[] ={0,0,255};
            image1 = cvCreateMat(imageStr->height, imageStr->width, CV_8UC1);
            cvCvtColor(imageStr, image1, CV_BGR2GRAY);
           CvSURFParams params;
          std::vector<cv::KeyPoint> keypointsA;
            cv::Mat descriptorsA;
            int Threshl=40;
            int Octaves=2;
            float PatternScales=1.0f;
            cv::BRISK  BRISKD(Threshl,Octaves,PatternScales);//initialize algoritm
            BRISKD.create("Feature2D.BRISK");
            BRISKD.detect(image1, keypointsA);
            BRISKD.compute(image1, keypointsA,descriptorsA);
            cv::Ptr<cv::FeatureDetector> detector = cv::Algorithm::create<cv::FeatureDetector>("Feature2D.BRISK");
            detector->detect(image1, keypointsA);
            cv::Ptr<cv::DescriptorExtractor> descriptorExtractor =cv::Algorithm::create<cv::DescriptorExtractor>("Feature2D.BRISK");
            descriptorExtractor->compute(image1, keypointsA, descriptorsA);
            for(int i=0;i<keypointsA.size();i++){
          outputDetector<<keypointsA[i].pt.x<<" "<<keypointsA[i].pt.y<<endl;
           outputFile<<descriptorsA.row(i)<<endl;
        }
outputFile <<endl;
return 0;       
}



void Vision::drawFeatures(IplImage *image,Vec detector,int dim){
   
int i;

//for(i=0;i<dim;i+=2)
  //  cvCircle(image, cvPoint(cvRound(detector.at(i)),cvRound(detector.at(i+1))), 0 ,CV_RGB(0, 255, 0),2); 

cvWaitKey(0);
}


MatrixXd Vision::clustering(int i){
const int ROWS=63;
const int COLS=10;
const int BUFFSIZE=10;
MatrixXd array(ROWS,COLS);
//int **array;
char buff[BUFFSIZE];

ifstream ifs("Brisc_db.txt");	
stringstream ss;

for(int row=0;row<ROWS-1;++row) {
ifs.getline(buff,BUFFSIZE);
ss<<buff;
for(int col=0;col<COLS-1;++col){
ss.getline(buff,15,',');
array(row,col)=atoi(buff);
 cout<<array(row,col)<<endl;
//**array=atoi(buff);
//(*array)++;
//cout<<array[row][col]<<"  ";
 ss<<"";
}
//array++;
ss.clear();
}
ifs.close();

return array;
}




  Mat Vision::clustering(){
    
     numberOfFeatures=0;
 int number_descriptorEntries; 
        ifstream ifs( "Brisc_db.txt" );
        string temp;
        char separateur=' ';
          Vec vecteur;
         Mat matrixe;
while( getline( ifs, temp ))
        {
          numberOfFeatures++;
            vecteur.clear();
            string::size_type stTemp = temp.find(separateur);
            number_descriptorEntries=0;
            while((stTemp != string::npos)&&(number_descriptorEntries<6))
            {
vecteur.push_back(temp.substr(0, stTemp-1));
                temp = temp.substr(stTemp + 1);
                stTemp = temp.find(separateur);            
            }           
            matrixe.push_back(vecteur);
            }
      return matrixe;
  }
    

IplImage* Vision::map_simulated(){
CvCapture* capture=cvCaptureFromCAM(CV_CAP_ANY);
if(!capture){
fprintf(stderr,"Erreur de capture");
getchar();
return NULL;
}
}

void Vision::featureDetector(char* str){
 string temp;
Vec vect;
 numberOfFeatures=0;
 char separateur=' ';
    ifstream ifs(str);
while(getline(ifs,temp)){

string::size_type stTemp = temp.find(separateur);
vect.push_back(temp.substr(0, stTemp-1));
vect.push_back(temp.substr(0, stTemp-1));
numberOfFeatures++;
}
}

int Vision::capture(){

    CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );
    if ( !capture ) {
        fprintf( stderr, "ERROR: capture is NULL \n" );
        getchar();
        return -1;
                 }
    // Create a window in which the captured images will be presented
    cvNamedWindow( "mywindow", CV_WINDOW_AUTOSIZE );
    // Show the image captured from the camera in the window and repeat
    while ( 1 ) {
        // Get one frame
        IplImage* frame = cvQueryFrame( capture );
        if ( !frame ) {
            fprintf( stderr, "ERROR: frame is null...\n" );
            getchar();
            break;
        }
        cvShowImage( "mywindow", frame );
        // Do not release the frame!

        if ( (cvWaitKey(10) & 255) == 's' ) {
            CvSize size = cvGetSize(frame);
            IplImage* img= cvCreateImage(size, IPL_DEPTH_16S, 1);
            img = frame;
             cvSaveImage("matteo.jpg",&img);
                                            }
     if ( (cvWaitKey(10) & 255) == 27 ) break;
    }
    // Release the capture device housekeeping
    cvReleaseCapture( &capture );
    cvDestroyWindow( "mywindow" );
    return 0;
}




