#include <eigen3/Eigen/Dense>
#include <vector.h>
using namespace Eigen;
using namespace std;
class Robot{
public:
vector<int> xrl;
vector<int> yrl;
vector<int> thetarl;
int numberOfSteps;

Robot(int);  
void test();
void PioneerModelView(int index);
VectorXd prediction(VectorXd X);
VectorXd distance_computation(VectorXd Xrobot,MatrixXd cloud_precedent);
VectorXd compute_weight(MatrixXd cloud_particles,VectorXd tag);
MatrixXd cloud_particles(VectorXd X_predicted);
void PioneerSpecifications();
friend void start_t();
friend void motor(int,MatrixXd,int);
friend void analog(int port);

};



