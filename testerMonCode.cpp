

#include <eigen3/Eigen/Dense>
#include <vector.h>

using namespace std;
using namespace Eigen; 

int main()
{

int size2attain;
size2attain=60;
MatrixXd X2interpolate(size2attain,3); 


int ii;
for(ii=0;ii<size2attain;ii++){
X2interpolate(ii,0)=3;
X2interpolate(ii,1)=3;
X2interpolate(ii,2)=3;
}



MatrixXd X_interpolted(size2attain,3);
 int i=0,ix=0,iy=0,ith=0;
int cx,cy;int ctheta;
 vector<int>  a,b;

int size=(int)((size2attain-2)/2);

i=0;
cout<<size;

vector<int>  test;
 vector<int>   interX1,interX2;
 vector<int>  variableX,variableY,variableThe;
vector<int>::iterator itX;
 vector<int> vector_regenerated;

while(i<X2interpolate.size()-4){
  interX1.at(0)=X2interpolate(i,0);
  interX1.at(1)=X2interpolate(i,1);
  interX1.at(2)=X2interpolate(i,2);
  interX2.at(0)=X2interpolate(i+1,0);
  interX2.at(1)=X2interpolate(i+1,1);
  interX2.at(2)=X2interpolate(i+1,2);

i+=2;
  cx=(interX2.at(0)-interX1.at(0))/size;
  cy=(interX2.at(1)-interX1.at(1))/size;
  ctheta=(interX2.at(2)-interX1.at(2))/size;

     variableX.at(0)= interX1.at(0);
     variableY.at(0)=interX1.at(0);
     variableThe.at(0)=interX1.at(0);
 
     while( variableX.at(ix++)<interX2.at(0))
       variableX.at(ix)=variableX.at(ix)+cx;

     while(variableY.at(iy++)<interX2.at(1))
       variableY.at(iy)=variableY.at(iy)+cy;
 
    while(variableThe.at(ith++)<interX2.at(2))
       variableThe.at(ith)=variableThe.at(ith)+ctheta;	 

i++;
}

return 0;
}




