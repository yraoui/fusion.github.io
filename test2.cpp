#include<iostream>

using namespace std;

int main(){

string s1;
string s2="bonjour";
string s3("les amis");
string s5;
string s4(s3,4,4);
s4.reserve(40);
s4.data();
s5=s4+s2+s3;
char buf[100]="ghghg";
s5.copy(buf,sizeof(buf));
cout<<buf;
int x=stoi("22");
cout<<x;
}






