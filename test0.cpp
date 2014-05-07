#include<iostream>
#include <fstream>
#include <sstream>

using namespace std;
const int ROWS=170;
const int COLS=15;
const int BUFFSIZE=800;

int main(){
int array[ROWS][COLS];
char buff[BUFFSIZE];
ifstream ifs("Brisc_db.txt");
stringstream ss;

for(int row=0;row<ROWS;++row) {
ifs.getline(buff,BUFFSIZE);
ss<<buff;
for(int col=0;col<4;++col){
ss.getline(buff,5,',');
array[row][col]=atoi(buff);
cout<<array[row][col]<<"  ";
 ss<<"";
}
 /*
ss<<"";
 ss.clear();
for(int row=0;row<ROWS;row++)
for(int col=0;col<COLS;col++)
cout<<array[row][col]<<"  ";
 */
 ss.clear();

}
ifs.close();

}
