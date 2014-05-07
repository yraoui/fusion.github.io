#include <iostream.h>
#include <string.h>
#include <cv.h>
#include <highgui.h>
#include <math.h>
#include  "sterio_interpretation.cpp"
#include <eigen3/Eigen/Dense>
static float numberOfParticle=100;


void test(){
  SterioTests st;
  vector<IplImage*>  image_Database;
  MatrixXd L(10,10);
  image_Database=st.load_image_db();

}

void PioneerModelView(int index){
char* choice_robot={"pioneer1.jpg"};
IplImage *imshow=cvLoadImage(choice_robot);
//cvShowImage("model view",imshow);
}

VectorXd prediction(VectorXd X){   
VectorXd Xs(3);       
VectorXf noise = VectorXf::Random(3);
Xs(0)=X(0)+10*noise(0);
Xs(1)=X(1)+10*noise(1);
Xs(2)=X(2)+10*noise(2);
return Xs;
}



VectorXd distance_computation(VectorXd Xrobot,MatrixXd cloud_precedent)
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




vector<int> Rfid_map(VectorXd X){

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


void finger_print(){
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

VectorXd compute_weight(MatrixXd cloud_particles,VectorXd tag){
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


MatrixXd cloud_particles(VectorXd X_predicted){  
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

void start_t(){
// true state
VectorXd X(3),tag(2);
int i;
float s1=0.0,s2=0.0,s3=0.0;
X(0)=1.1;
X(1)=1.2;
X(2)=3.1;
VectorXd weight(numberOfParticle);
VectorXd X_pred(3);
MatrixXd cloud_partic(100,3);
X_pred=prediction(X);


cloud_partic=cloud_particles(X_pred);

tag(0)=1.1;
tag(1)=2.1;

weight=compute_weight(cloud_partic,tag);

for(i=0;i<numberOfParticle;i++)
{
s1=s1+weight(i)*cloud_partic(i,0);
s2=s2+weight(i)*cloud_partic(i,1);
s3=s3+weight(i)*cloud_partic(i,2);
}

s1=s1/numberOfParticle;
s2=s2/numberOfParticle;
s3=s3/numberOfParticle;
}

void PioneerSpecifications(){

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

int main(){


PioneerSpecifications();
start_t();
  return 0;
}





