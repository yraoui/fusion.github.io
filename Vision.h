#include <iostream.h>
#include <cv.h>
#include <highgui.h>
#include <eigen3/Eigen/Dense>
#include <vector.h>
#include <string>


using namespace std;
using namespace Eigen;

typedef vector<string> Vec;
typedef vector<Vec> Mat;

class Vision{


public:
  int numberOfFeatures;
IplImage* imageLeft;
IplImage* imageRight;
IplImage* imageCenter;

Vision(IplImage* imagel,IplImage* imager,IplImage* imagec){
imageLeft=imagel;
imageRight=imager;
imageCenter=imagec;
}
MatrixXd clustering(int);

void featureDetector(char* str);
Vision(const Vision&);
~Vision(){}

IplImage* map_simulated();


void drawFeatures(IplImage *image,Vec detector,int dim);

void matching_euclidean_distance(float threshold){
Mat descriptorLeftImage, descriptorRightImage,descriptorCenterImage;
Vec descriptorLineLeft,descriptorLineRight;
int i,j;
MatrixXd match(10,6);

for(i=0;i<numberOfFeatures;i++)
	for(j=4;j<10;j++)
		match(i,j)=0;

vector<double>  descriptorLine;
float distanceBetweenDescriptors=0.0;
BriscCompute(imageLeft);
descriptorLeftImage=clustering();
BriscCompute(imageRight);
descriptorRightImage=clustering();
BriscCompute(imageCenter);
descriptorCenterImage=clustering();
for(i=0;i<10;i++){
descriptorLineLeft=descriptorLeftImage.at(i);
descriptorLineRight=descriptorRightImage.at(i);
	for(j=0;j<6;j++)
distanceBetweenDescriptors=distanceBetweenDescriptors+sqrt(((double)atof(descriptorLineLeft.at(j).c_str())-(double)atof(descriptorLineLeft.at(j).c_str()))*((double)atof(descriptorLineLeft.at(j).c_str())-(double)atof(descriptorLineLeft.at(j).c_str())));
distanceBetweenDescriptors=sqrt(distanceBetweenDescriptors);
if(distanceBetweenDescriptors<threshold)
match(i,j)++;
}
}

//Monocularv sterio
int capture();
int BriscCompute(IplImage* imageStr);
Mat clustering();
};


