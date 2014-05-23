#ifndef GAITAN_FOOT_H
#define GAITAN_FOOT_H

#include <iostream>
#include <libgaitan/shape.h>
#include <libgaitan/box.h>  
#include <Eigen/Dense>
#include <vector>
#include <pcl/common/common_headers.h>

// TODO : surcharger les operation << et =
//

using namespace Eigen;

namespace gaitan
{

  
  class Foot
  {  
    
    public:
      enum footSide { UNKNOWN=0, LEFT=1, RIGHT=2 };
    
	  private:	
         
    
     footSide label; 
     
     // pose
     double tX;
     double tY;
     double tZ;
     double rX;
     double rY;
     double rZ;
     
     //toe
     double toeX;
     double toeY;
     double toeZ;
     
     //length
     double length;
     
     //volume
     double volume;
     
    public:
    
      
     Foot();
     Foot(const double &xmeas, const double &ymeas, const double &zmeas);
     ~Foot();
     
     void print();
     static void autoLabel(Foot &foot1, Foot &foot2);
     
     inline void setLabel(const footSide & x){this->label=x;}
     inline void setTX(const double & x){this->tX=x;}
     inline void setTY(const double & y){this->tY=y;}
     inline void setTZ(const double & z){this->tZ=z;}
     inline void setRX(const double & theta){this->rX=theta;}
     inline void setRY(const double & theta){this->rY=theta;}
     inline void setRZ(const double & theta){this->rZ=theta;}
     inline void setToeX(const double & x){this->toeX=x;}
     inline void setToeY(const double & y){this->toeY=y;}
     inline void setToeZ(const  double &z){this->toeZ=z;}
     
     
     inline footSide getLabel(){return this->label;}
     inline double getTX(){return this->tX;}
     inline double getTY(){return this->tY;}
     inline double getTZ(){return this->tZ;}
     inline double getRX(){return this->rX;}
     inline double getRY(){return this->rY;}
     inline double getRZ(){return this->rZ;}
     
     inline double getToeX(){return this->toeX;}
     inline double getToeY(){return this->toeY;}
     inline double getToeZ(){return this->toeZ;}
     
     inline double getLength(){return this->length;}
     inline void setLength(const double & l){this->length=l;}

     inline double getVolume(){return this->volume;}
     inline void setVolume(const double & l){this->volume=l;}
     
     
     
      
     // linear prediction based on constant velocity assumption 
     int predict (const double &xpred, const double &ypred, const double &zpred);
     
     // update the current state with measure
     int update ( const double &xmeas, const double &ymeas, const double &zmeas);  
     int update ( Foot & foot);  
     int update(Box & box);
    
     // compute the distance between the two feet
     double distance(Foot &  otherFoot);
     
     // compute the min distance between two feet
     int whichFoot (Foot & foot1,Foot&foot2); 
    
    
  
    
    
  };
}
#endif
   
