//
//  monocularv.h
//  
//
//  Created by Younès RAOUI on 02/03/13.
//
//
#include <iostream>
#include <vector.h>
#include <cv.h>
#include <fstream>
#include <highgui.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <string>
using namespace std;
using namespace Eigen;
typedef vector<string> Vec;
typedef vector<Vec> Mat;


class Monocularv{

public:    
    Monocularv(){}
 int line_size;        
 int colomn_size;
    static int size_surf;
    
    vector<IplImage*> load_image_db(){
        IplImage *im;
        vector<IplImage*> ims;
        int i;
        char buffer[20];
        for (i=1;i<16;i++)
        {
            sprintf(buffer,  "rooms/im%d.png",i);
            im=cvLoadImage(buffer,1);
            
            ims.push_back(im);
            
        }
        return ims;
        
    }

IplImage** load_3_images(){
    IplImage **threeclop;
    threeclop[0]=cvLoadImage("top.jpg");
    threeclop[1]=cvLoadImage("left.jpg");
    threeclop[2]=cvLoadImage("right.jpg");
    return threeclop;

  }


int bRIScComputation(){
        vector<IplImage*> image_scenes=load_image_db();
        int i;
        CvMat *image1=0;
           ofstream outputFile("surf_db.txt");
         static CvScalar red_color[] ={0,0,255};
        IplImage* image;
    
        for(i=0;i<6;i++){
            image=image_scenes.at(i);
            image1 = cvCreateMat(image->height, image->width, CV_8UC1);
            cvCvtColor(image, image1, CV_BGR2GRAY);
            CvSURFParams params;
          //  cv::Mat GrayA =cv::imread(image1);
          //  cv::Mat GrayB =cv::imread(PimB);
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
          outputFile<<keypointsA[i].pt.x<<" "<<keypointsA[i].pt.y<<" ";
           outputFile<<descriptorsA.row(i)<<endl;
            }
            
        }
outputFile <<endl;


return 0;
        
}


/*
    
    int extract_SURF(int surf){
        vector<IplImage*> image_scenes=load_image_db();
        int i;
        CvMat *image1=0;
           ofstream outputFile("surf_db.txt");
	   //    ofstream outputFile("surf_db.txt", ios::out | ios::trunc);// ouverture en 

        static CvScalar red_color[] ={0,0,255};
        IplImage* image;
        
        for(i=0;i<10;i++){
            image=image_scenes.at(i);

        image1 = cvCreateMat(image->height, image->width, CV_8UC1);
            cvCvtColor(image, image1, CV_BGR2GRAY);

        CvSeq* descriptors=0,*keypoints=0;
        
CvSURFParams params;


 params = cvSURFParams(500, 1); // MAX_SURF_POINTS = 1000 

            params.hessianThreshold=1000;
             params.nOctaveLayers=10;
             params.nOctaves=10;
             params.upright = false;
             

        CvMemStorage* stor = cvCreateMemStorage(0);
        
	//   CvSURFParams params=cvSURFParams(500,0);
        cvExtractSURF(image,0,&keypoints,&descriptors,stor,params,0);

            cout<<descriptors->total;
	           
for(CvSeq* c=descriptors;c!=NULL;c=c->h_next){
    
    for(int i=0;i<c->total;++i){

        if(i==0){
float* r1=(float*)cvGetSeqElem(keypoints,i);
CvPoint center;
int radius;
//center.x=cvRound(r1->pt.x);
//center.y=cvRound(r1->pt.y);
     outputFile <<r1<<" ";//<<" "<<center.y<<" ";
                
        }
CvPoint *p=(CvPoint*)cvGetSeqElem(c,i);
outputFile <<p->x<<" "<<p->y<<" ";
}
outputFile <<endl;
}
}
return 0;
        
        }
*/

  Mat clustering(){
   
        ifstream ifs( "surf_db.txt" );
        string temp;
        char separateur=' ';
        vector<string> vecteur;
         Mat matrixe;
      line_size=0;
	 colomn_size=0;
while( getline( ifs, temp ) )
            
        {
            vecteur.clear();
            string::size_type stTemp = temp.find(separateur);
            while(stTemp != string::npos)
            {
	  colomn_size++ ;

int test;
    if((temp.substr(0, 1).compare("[")!=0) ) { 
vecteur.push_back(temp.substr(0, stTemp-1));
}

                temp = temp.substr(stTemp + 1);
                stTemp = temp.find(separateur);            
            }           
            matrixe.push_back(vecteur);
            }

            line_size++;
                
      return matrixe;
  }
    
    /* TO DO
    void quantization(){
    kmeans(const Mat& samples, int clusterCount, Mat& labels, TermCriteria termcrit, int attempts, int flags, Mat* centers)
        
    }*/
    
};


int main(){
    
    Monocularv  cv;
    //cv.extract_SURF();
    //Mat v;
//v=cv.clustering();
//cout<<v.at(0).at(0);
    return 0;
    
}








