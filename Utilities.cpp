
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <vector.h>
using namespace std;
using namespace Eigen;


typedef vector<string> Vec;
typedef vector<Vec> Mat;

class Utilities{
float *x;
float *y;
public:
float *theta;
float *moyenne_robot;
float *std_robot;
int size;
public:
Utilities(){}
Utilities(float *x_pass,float *y_pass,float *theta_pass){
x=x_pass;
y=y_pass;
theta=theta_pass;
}
Utilities(Utilities &);

void mean(){
int i;
moyenne_robot[0]=moyenne_robot[1]=moyenne_robot[2]=0.0;
for(i=0;i<size;i++){
moyenne_robot[0]+=x[i];
moyenne_robot[1]+=y[i];
moyenne_robot[2]+=theta[i];
}
moyenne_robot[0]=moyenne_robot[0]/size;
moyenne_robot[1]=moyenne_robot[1]/size;
moyenne_robot[2]=moyenne_robot[2]/size;
}

void standard_deviat(){
int i;
std_robot[0]=std_robot[1]=std_robot[2]=0.0;
for(i=0;i<size;i++){
std_robot[0]+=(x[i]-moyenne_robot[0])*(x[i]-moyenne_robot[0]);
std_robot[1]+=(y[i]-moyenne_robot[1])*(y[i]-moyenne_robot[1]);
std_robot[2]+=(theta[i]-moyenne_robot[2])*(theta[i]-moyenne_robot[2]);
}
}

MatrixXd covariance(float *vect_0,float *vect_1){

MatrixXd cov(3,3);
cov(0,0)=std_robot[0]*std_robot[0];
cov(0,1)=cov(1,0)=std_robot[0]*std_robot[1];
cov(0,2)=cov(2,0)=std_robot[0]*std_robot[2];
cov(1,1)=std_robot[0]*std_robot[1];
cov(1,2)=cov(2,1)=std_robot[0]*std_robot[2];
cov(2,2)=std_robot[2]*std_robot[2];

return cov;

}


VectorXd tcomp(VectorXd X,VectorXd U){
        X(0)=X(0)+U(0)*cos(X(2))+U(1)*sin(X(2));
        X(1)=X(1)-U(0)*sin(X(2))+U(1)*cos(X(2));
        X(2)=X(2)+U(2);
        return X;
}

VectorXd tcompI(VectorXd X,VectorXd U){
        X(0)=X(0)+U(0)*cos(X(2))-U(1)*sin(X(2));
        X(1)=X(1)+U(0)*sin(X(2))+U(1)*cos(X(2));
        X(2)=X(2)+U(2);
        return X;
}


};



class GeometriTransforms:public Utilities{

public: 
int number_of_features;
int  number_of_descriptor_elements;
GeometriTransforms(int  N,int M):Utilities(){number_of_features=N;number_of_descriptor_elements=M;}
GeometriTransforms(const GeometriTransforms &g):Utilities(){}
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
VectorXd predict_feature(VectorXd X, float p, float q, float alpha){
VectorXd X_prime(3),Xp(3);
X_prime(0)=(X(0)-p)*cos(alpha)-(X(2)-q)*sin(alpha);
X_prime(1)=X(1);
X_prime(2)=(X(0)-p)*sin(alpha)+(X(2)-q)*cos(alpha);
Xp=X_prime;
return Xp;
}
MatrixXd Rcrr(float theta){
MatrixXd R(3,3);
R(0,0)=cos(theta);
R(0,1)=sin(theta);
R(0,2)=0;
R(1,0)=-sin(theta);
R(1,1)=cos(theta);
R(1,2)=0;
R(2,0)=0;
R(2,1)=0;
R(2,2)=1;
return R;
}

Eigen::VectorXd Prj2Dto3D(VectorXd X){
Eigen::MatrixXd RW(3,3);
VectorXd fx(3);
fx(0)=10;
fx(1)=11;
fx(2)=11;   
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
VectorXd position_feature_Camera(VectorXd X){
VectorXd Pfc(3);
VectorXd Cpr(3);
float tx=11.0,ty=11.4,tz=11.9,h=11.2,thetak=10.0;
Cpr(0)=tx;
Cpr(1)=ty;
Cpr(2)=tz;
VectorXd Gprk(3);
Gprk(0)=X(0);
Gprk(1)=X(1);
Gprk(2)=h;
MatrixXd R(3,3),Rc(3,3);
R=rotation_g_robot(thetak);
Rc=Rcrr(thetak);
Pfc=Rc*R.transpose()*(Prj2Dto3D(X)-Gprk)+Gprk;
return Pfc;
}

  /*
bool matching(Mat desc1,Mat desc2){
float threshold=50.0;

float mean_of_distance_values;
Vec vect1,vect2;
MatrixXd distance(number_of_features,number_of_features);
float d=0.0;
int i,j,k;
for(i=0;i<number_of_features;i++)
    for(j=0;i<number_of_features;j++)
{

vect1=desc1.at(i);
vect2=desc2.at(j);
d=0.0;
for(k=0;k<number_of_descriptor_elements;k++){
string s1=vect1.at(k);
string s2=vect2.at(k);
char* ii=(char*)s1;
d=d+(vect1.at(k)-vect2.at(k));

}

d=sqrt(d);
distance(i,j)=d;
}

mean_of_distance_values=distance.mean();

if(mean_of_distance_values <threshold)
    return true;
else return false;


}

{}

  */

/*
J(0,0)=cos(delta);
J(0,1)=0;
J(0,2)=sin(delta);
J(1,0)=-sin(delta);
J(1,1)=0;
J(1,2)=cos(delta);
J(2,0)=-p*sin(delta)-q*cos(delta);
J(2,1)=0;
J(2,2)=p*cos(delta)-q*sin(delta);
*/

/*
VectorXd f(VectorXd *x1, VectorXd *x2){
VectorXd X_prime(3),Xp(3);
 X_prim(0)=(*x1(0)-*x2(0))*cos(*x2(2))-(*x1(2)-*x2(1))*sin(*x2(2));
X_prime(1)=*x1(1);
 X_prime(2)=(*x1(0)-*x2(0))*sin(*x2(2))+(*x1(2)-*x2(1))*cos(*x2(2));
Xp=X_prime;
return Xp;
}

*/

/*
VectorXd gauss_newton_minimization(VectorXd X){
Mat descriptor;
Vision v;
MatrixXd J(3,3);

J(0,0)=cos(delta);
J(0,1)=0;
J(0,2)=sin(delta);
J(1,0)=-0;
J(1,1)=1;
J(1,2)=0;
J(2,0)=-sin(delta);
J(2,1)=0;
J(2,2)=cos(delta);

VectorXd yi(3);

// ground thruth

yi(0)=11;
yi(1)=22;
yi(3)=11;

J1=(J.transpose())*J;
J=J1.inverse()*J.transpose();
r=yi-f(estimated_robot_position,predicted_feature_position);
estimated_feature_position=estimated_feature_position- J*r;

return estimated_feature_position;
}
*/
 

};
