#include "Utilities.cpp"
#include "Vision.h"
#include "robot.h"
#define pi 3.14
typedef vector<string> Vec;
typedef vector<Vec> Mat;

class Fusion :public Robot{
	
vector<int> tagx;
vector<int> tagy;
    
int *VectXr;
int *VectYr;
float *VectTheta;
int numberOfSteps;
Fusion(int ns){
 
VectorXd xrl(25),yrl(25),thetarl(25);
  int i;
  int VecXrforfusion[26]=  {50,  65 , 70 , 75  ,90 ,105, 110, 115 ,120 ,13,0 ,145 ,150,  145 ,130 ,115 ,110 ,105 , 90 , 75 , 70 , 65 , 50 , 35,  25 , 30 };
 int VecYrforfusion[26]=  {110, 115 ,130, 145, 150, 145 ,130 ,120, 110 ,110 ,105 , 90 ,  70,  65,  70  ,90 ,105 ,110 ,105 , 90 , 75 , 65 , 70 , 90, 105, 110};
  float VecThetaforfusion[26]={0  ,pi/4 , pi/2 , pi/4 ,  0 ,-pi/4 ,-pi/2 ,-pi/4 ,  0  , 0 ,-pi/4 ,-pi/2 ,-135*pi/180 ,pi , 135*pi/180 ,pi/2 ,135*pi/180 ,pi ,225*pi/180 ,270*pi/180 ,225*pi/180 ,pi ,135*pi/180 , pi/2 , pi/4 ,  0};
  numberOfSteps=ns;
  for(i=0;i<26;i++){
      xrl[i]=VecXrforfusion[i];
      yrl[i]=VecYrforfusion[i];
      thetarl[i]=VecThetaforfusion[i];
}
VectXr=VecXrforfusion;
VectYr=VecYrforfusion;
VectTheta=VecThetaforfusion;
}
Fusion(Fusion& a){}


void init_RFID_tags(){	
int i,N=10,x,y;
for(i=0,x=10,y=20;i<N,x<100,y<200;i++,x=+20,y+=40)
{
tagx[i]=x;
tagy[i]=y;
}

}


void fuse(VectorXd X){
VectorXd X_predicted(3);
Utilities U;
Vision V;

int *x_pass,*y_pass; float *theta_pass;
x_pass=VectXr;
y_pass=VectYr;
theta_pass=VectTheta;
X_predicted(0)=*x_pass;
X_predicted(1)=*y_pass;
X_predicted(2)=*theta_pass;
MatrixXd cloud;

cloud=cloud_particles(X_predicted);

VectorXd weights;
weights=compute_weight(cloud_particles,tagx,tagy);

MatrixXd array(ROWS,COLS);
Mat   descriptor;
Vec v_desc;
Vec x;
int k=0;
int i;
Vec tempom;
IplImage* scene;
IplImage* image_r=cvLoadImage("images.jpg");
IplImage* image_l=cvLoadImage("ResPiro.jpg");
IplImage* image_c=cvLoadImage("ResPiro.jpg");
Vision *myWork=new Vision(image_r,image_l,image_c);

myWork->numberOfFeatures=60;
myWork->BriscCompute(image_l);
array=myWork->clustering(2);

}

void interpolate(MatrixXd X2interpolate, int size2attain){
//to do		
MatrixXd X_interpolted(size2attain,3);
 int i=0,ix,iy,ith;

 vector<int>  a,b;


int size=(int)(size2attain-X2interpolate.size())/X2interpolate.size();
vector<int>  test;
 vector<int>   interX1,interX2;
 vector<int>  variableX,variableY,variableThe;
vector<int>::iterator itX;

while(i++<X2interpolate){

  interX1.at(0)=X2interpolate(i,0);
  interX1.at(1)=X2interpolate(i,1);
  interX1.at(2)=X2interpolate(i,2);

  interX2.at(0)=X2interpolate(i+1,0);
  interX2.at(1)=X2interpolate(i+1,1);
  interX2.at(2)=X2interpolate(i+1,2);

  cx=(interX2.at(0)-interX1.at(0)/size;
  cy=(interX2.at(1)-interX1.at(1)/size;
  ctheta=(interX2.at(2)-interX1.at(2)/size;

     variableX.at(0)= interX1.at(0);
     variableY.at(0)=interX1.at(0);
     variableThe.at(0)=interX1.at(0);
 ix=0;
     while( variableX.at(ix++)<interX2.at(0))
       variableX.at(ix)=variableX.at(ix)+cx;
     iy=0;
     while(variableY.at(iy++)<interX2.at(1))
       variableY.at(iy)=variableY.at(iy)+cy;
     ith=0;
     while(variableThe.at(ith++)<interX2.at(2))
       variableThe.at(ith)=variableThe.at(ith)+ctheta;





}

}
/*
Utilities(float *x_pass,float *y_pass,float *theta_pass){

VectorXd noise(3);
X(0)=X(0)+noise(0);
X(1)=X(1)+noise(1);
X(2)=X(2)+noise(2);
v.BriscCompute();
M=v.clustering();

Matrixxd mat_features(100,3);
VectorXd V;
int i,j;
for(i=0;i<100;i++)
for(j=0;j<6;j++){
V=M.at(i);
mat_features(i,j)=V(j);
}
struct distance *d;
d=observation(X);


Vision v;
Mat M;
}
*/


struct distance* observation(VectorXd X){

struct distance{
float x;
float suivant;
};

int i,j;
struct distance *d;
for(i=0;i<Vision.Numberoffeatures;i++)
	for(j=0;j<6;j++){
	*d->x=*d->x +(X(0)-mat_features(i,j));
	d->x=d->x->suivant;	
}
Mat &decriptor=d;
list<Vec>::iterator it;

list<Vec>   L;
int i;
for(it=L.begin(),i=0;it!=L.end();++it,++i){
it=descriptor.at(i);
}

return d;
}


};
	





