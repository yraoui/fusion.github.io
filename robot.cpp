#include <iostream.h>
#include <string.h>
#include <cv.h>
#include <highgui.h>
#include <math.h>
#include  "sterio_interpretation.cpp"
#include <eigen3/Eigen/Dense>
#include "Robot.h"
#include "RFID.h"
//#include "Vision.h"
#include "Utilities.cpp"
#define pi 3.14

static float numberOfParticle=100;


void Robot::test(){
  SterioTests st;
  vector<IplImage*>  image_Database;
  MatrixXd L(10,10);
  image_Database=st.load_image_db();
}

Robot::Robot(int ns){
                int i;
                int VecXr[26]=  {50,  65 , 70 , 75  ,90 ,105, 110, 115 ,120 ,13,0 ,145 ,150,  145 ,130 ,115 ,110 ,105 , 90 , 75 , 70 , 65 , 50 , 35,  25 , 30 };
                int VecYr[26]=  {110, 115 ,130, 145, 150, 145 ,130 ,120, 110 ,110 ,105 , 90 ,  70,  65,  70  ,90 ,105 ,110 ,105 , 90 , 75 , 65 , 70 , 90, 105, 110};
                float VecTheta[26]={0  ,pi/4 , pi/2 , pi/4 ,  0 ,-pi/4 ,-pi/2 ,-pi/4 ,  0  , 0 ,-pi/4 ,-pi/2 ,-135*pi/180 ,pi , 135*pi/180 ,pi/2 ,135*pi/180 ,pi ,225*pi/180 ,270*pi/180 ,225*pi/180 ,pi ,135*pi/180 , pi/2 , pi/4 ,  0};
                numberOfSteps=ns;
                for(i=0;i<26;i++){
                xrl.push_back(VecXr[i]);
                yrl.push_back(VecYr[i]);
                thetarl.push_back(VecTheta[i]);}

}



void Robot::PioneerModelView(int index){
char* choice_robot={"pioneer1.jpg"};
IplImage *imshow=cvLoadImage(choice_robot);
//cvShowImage("model view",imshow);
}

VectorXd Robot::prediction(VectorXd X){   
VectorXd Xs(3);       
VectorXf noise = VectorXf::Random(3);
Xs(0)=X(0)+10*noise(0);
Xs(1)=X(1)+10*noise(1);
Xs(2)=X(2)+10*noise(2);
return Xs;
}

VectorXd Robot::distance_computation(VectorXd Xrobot,MatrixXd cloud_precedent)
    {
      VectorXd X_precedent(4);
        int i,j,min_i;
        float min=0.0;
        for(i=0;i<numberOfParticle;i++){
          if (min> ( sqrt((Xrobot(0)-cloud_precedent(i,0))*(Xrobot(0)-cloud_precedent(j,0))+(Xrobot(1)-cloud_precedent(i,1))*(Xrobot(1)-cloud_precedent(i,1)))));
                  
              {
                  min=( sqrt((Xrobot(0)-cloud_precedent(i,0))*(Xrobot(0)-cloud_precedent(i,0))+(Xrobot(1)-cloud_precedent(i,1))*(Xrobot(1)-cloud_precedent(i,1))));
                  
                  min_i=i;   
              }        
        }
        
        X_precedent(0)=cloud_precedent(min_i,0);
        X_precedent(1)=cloud_precedent(min_i,1);
        X_precedent(2)=cloud_precedent(min_i,2);
        X_precedent(3)=0.0;
        return X_precedent;
    }
    


bool isInSector(float x_center,float y_center,float angle_offset,float angle, float x,float y){

VectorXd first_line(3),second_line(3);
first_line(0)=sin(angle_offset);
first_line(1)=-cos(angle_offset);
first_line(2)=-x_center*sin(angle_offset)+y_center*cos(angle_offset);
second_line(0)=sin(angle_offset+angle);
second_line(1)=-cos(angle_offset+angle);
second_line(2)=-x_center*sin(angle_offset+angle)+y_center*cos(angle_offset+angle);


if((x*first_line(0)+y*first_line(1)+first_line(2)<0)&&(x*second_line(0)+y*second_line(1)+second_line(2)<0));
else return false;


}




vector<int> RFID::Rfid_map(VectorXd X){

        int i,j,l=0;
vector<int> map;
float angle_offset=0.12;
float angle=1.12;
bool v;
for(i=0;i<9;i++)
map.push_back(0);

Eigen::MatrixXd tag(82,2);
        for(i=0;i<180;i+=20)
                for(j=0;j<180;j+=20){
                        tag(l,0)=i;
                        tag(l,1)=j;
                        l++;
                }
l=0;
        for(i=0;i<180;i+=20)
                for(j=0;j<180;j+=20){
if((v=isInSector( tag(l,0), tag(l,1),angle_offset,angle, X(0),X(1)))==true){
l++;
map.at(l)++;
}
}
return map;
}


void RFID::finger_print(){
VectorXd g(numberOfParticle);
int i,j,l=0;
Eigen::MatrixXd tag(82,2);
        for(i=0;i<180;i+=20)
                for(j=0;j<180;j+=20){
                        tag(l,0)=i;
                        tag(l,1)=j;
                        l++;
                }


for(i=0;i<numberOfParticle;i++)
  g(i)=0;

}




// compute the weights of the particles at each robot position




/*
VectorXd Robot::compute_weight(MatrixXd cloud_particles,vector<int> tagx,vector<int> tagy){
float d=0.0,angle_offset=3.14,angle=2.1;
vector<float> *likelihood;

int i;

for(i=0;i<numberOfParticle;i++)

vector<MatrixXd*>  X;
MatrixXd X_t(numberOfParticle,3);
int i=j=k=0;
while(i<X->size()){
X_t=*X[i];

for(j=0;j<numberOfParticle;j++){
  for(k=0;k<numberOfTags;k++)
{
d=sqrt((X_t(j,0)-tagx[k])*(X_t(j,0)-tagx[k])+(X_t(j,1)-tagy[k])*(X_t(j,1)-tagy[k]));
if((isInSector(tag(0),tag(1),angle_offset,angle, x(j,0), x(j,1))==false)){

if (d<10)
    *likelihood[j]*=0.7;
  else if(d>10 && d<60)
    *likelihood[j]*=0.9;
else
    *likelihood[j]*=0.5;
    }
likelihood++;
}
}
i++;
}
}


*/






VectorXd Robot::compute_weight(MatrixXd cloud_particles,VectorXd tag){
float d=0.0,likelihood=0.0,normalisation,x=0.0,y=0.0,angle_offset=3.14,angle=2.1;
int i;
VectorXd weight(numberOfParticle);
for(i=0;i<numberOfParticle;i++)
weight(i)=1.0;


for(i=0;i<numberOfParticle;i++)
{
x=cloud_particles(i,0);
y=cloud_particles(i,1);
d=sqrt((x-tag(0))*(x-tag(0))+(y-tag(1))*(y-tag(1)));
if((isInSector(tag(0),tag(1),angle_offset,angle, x, y)==false)){

if (d<10)
    likelihood=0.7;
  else if(d>10 && d<60)
    likelihood=0.9;
else
    likelihood=0.5;
    }
    
weight(i)=weight(i)*likelihood;
}
float s=0;
for(i=0;i<numberOfParticle;i++)
s=s+weight(i);

normalisation=1/s;

weight=weight*normalisation;

return weight;
}


MatrixXd Robot::cloud_particles(VectorXd X_predicted){  
VectorXf noise ;
MatrixXd X_i(100,3);
int i;
for(i=0;i<numberOfParticle;i++){
     noise = VectorXf::Random(3);
     X_i(i,0)=X_predicted(0)+noise(0);
     X_i(i,1)=X_predicted(1)+noise(1);
     X_i(i,2)=X_predicted(2)+noise(2);
 }
 return X_i;  
}



VectorXd predict_feature(VectorXd X, float p, float q, float alpha){
VectorXd X_prime(3),Xp(3);
X_prime(0)=(X(0)-p)*cos(alpha)-(X(2)-q)*sin(alpha);
X_prime(1)=X(1);
X_prime(2)=(X(0)-p)*sin(alpha)+(X(2)-q)*cos(alpha);
Xp=X_prime;
return Xp;
}








Eigen::VectorXd Prj2Dto3D(VectorXd X, VectorXd fx){
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



IplImage* map_simulated(){
CvCapture* capture=cvCaptureFromCAM(CV_CAP_ANY);
if(!capture){
fprintf(stderr,"Erreur de capture");
getchar();
return NULL;
}

IplImage* frame=cvQueryFrame(capture);

if(!frame){
fprintf(stderr,"Erreur");
getchar();
return NULL;
}

else
return frame;
}





/*
int robot::capture(){

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



*/



/*

void Robot::Gauss_Newton(){
MatrixXd measured(3,2);
measured(0,0)=11;
measured(0,1)=22;
measured(1,0)=11;
measured(1,1)=22;
measured(2,0)=1;
measured(2,1)=22;
MatrixXd J(3,3),J1(3,3);
float delta;
VectorXd Xs(3);
VectorXd X_prime=predict_feature(X,p,q,alpha);
J(0,0)=-cos(delta);
J(1,0)=0.0;
J(2,0)=-sin(delta);
J(0,1)=0.0;
J(1,1)=1.0;
J(2,1)=0.0;
J(0,2)=-sin(delta);
J(1,2)=0.0;
J(2,2)=cos(delta);

Xs=Prj2Dto3D(X);

J1=(J.transpose())*J;
J=J1.inverse()*J.transpose()*(X-X_prime);

r=measured-stenope(X_prime)


}

*/

void Robot::PioneerSpecifications(){

  string size;
  size= "50 cm or 19,7 in";
  string computer;

computer= "64-bit Dual Core 1.8 GHz Atom industrial PC 2GB DDR3 RAM Wireless Ethernet 3x USB 2.0 2x RS-232 serial 2x gigabit ethernet Audio input and output, speakers included";


string weight;
weight="60 kg (132 lbs)";

string os;
os="Ubuntu Linux or Windows 7";
string max_speed;

max_speed="1,8 m/s";

//char **features={"Autonomous Navigation and Mapping Software","SICK S300 Laser Scanner","Joystick (used for Mappi\\
//","Speakers & Voice Synthesis Software","Pioneer Software Development Kit"};
string optional_features[4]={"Digital Pan/Tilt/Zoom Camera","Robotic Arms","Pan Tilt Positioning Unit","GPS Systems"};

cout<<size <<" "<<endl;
cout<<computer <<" "<<endl;
cout<<weight <<" "<<endl;
cout<<os <<" "<<endl;
cout<<max_speed <<" "<<endl;
//for(int i=0;i<5;i++)
//cout<<features[i]<<" "<<endl;
for(int i=0;i<4;i++)
cout<<optional_features[i]<<" "<<endl;
}


/*
int main(){
Utilities gu,utili;
int i;
float s1=0.0,s2=0.0,s3=0.0;
Robot *r= new Robot(25);
MatrixXd cloud_partic(100,3);
VectorXd motionStep(3),X_next(3);
cout<<r->numberOfSteps;
VectorXd Xrobot(r->numberOfSteps),Yrobot(r->numberOfSteps),AngleRobot(r->numberOfSteps);
VectorXd weight(100);
VectorXd X(3),tag(2),X_pred(3);
MatrixXd u(r->numberOfSteps,3);

for(i=0;i<r->numberOfSteps-1;i++){
u(i,0)=Xrobot(i+1)-Xrobot(i);
u(i,1)=Yrobot(i+1)-Yrobot(i);
u(i,2)=AngleRobot(i+1)-AngleRobot(i);
}
for(i=0;i<1;i++){
X(0)=r->xrl.at(i);
X(1)=r->yrl.at(i);
X(2)=r->thetarl.at(i);
motionStep(0)=u(i,0);
motionStep(1)=u(i,1);
motionStep(2)=u(i,2);
X_next=utili.tcomp(X,motionStep);
X_pred=r->prediction(X_next);
cloud_partic=r->cloud_particles(X_pred);
cout<<" Xpred: "<<X_pred;
cout<<endl;
cout<<" cloud_partic: "<<cloud_partic;

}
return 0;
}

*/
/*
//}
//IplImage *image_r=cvLoadImage("ResPiro.jpg");
//IplImage* image_l=cvLoadImage("left.jpg");
//IplImage* image_c=cvLoadImage("center.jpg");
//Vision myWork(image_r,image_r,image_r);
//IplImage *scene;
//int c=0,v;
//while(1& c++<2){
//scene=map_simulated();
//myWork.BriscCompute(image_r);
//}


//tag(0)=1.1;
//tag(1)=2.1;
//weight=r->compute_weight(cloud_partic,tag);
/*


for(i=0;i<numberOfParticle;i++)
{
s1=s1+weight(i)*cloud_partic(i,0);
s2=s2+weight(i)*cloud_partic(i,1);
s3=s3+weight(i)*cloud_partic(i,2);
}

s1=s1/numberOfParticle;
s2=s2/numberOfParticle;
s3=s3/numberOfParticle;
bool b_lr,b_lc,b_cr;
Mat   descriptor_r,descriptor_l,descriptor_c; 
Vec v_desc;
int i;
IplImage* image_r=cvLoadImage("right.jpg");
IplImage* image_l=cvLoadImage("left.jpg");
IplImage* image_c=cvLoadImage("center.jpg");
Vision *myWork=new Vision(image_r,image_l,image_c);
myWork->BriscCompute(image_r);
desc_r=myWork->clustering();
myWork->BriscCompute(image_l);
desc_l=myWork->clustering();
myWork->BriscCompute(image_l);
desc_c=myWork->clustering();
b_lr=gu.matching(desc_l,desc_r);
b_lc=gu.matching(desc_l,desc_c);
b_cr=gu.matching(desc_c,desc_r);
*/

//return 0;

//}





