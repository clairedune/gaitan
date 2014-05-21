#ifndef GAITAN_FOOT_H
#define GAITAN_FOOT_H

#include <iostream>
#include <libgaitan/shape.h>
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
	  private:	
    
     std::string label;
     
     
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
     
    public:
    
      
     Foot();
     ~Foot();
     
     inline void setLabel(const std::string& label){this->label=label;}
     inline void setTX(const & x){this->tX=x;}
     inline void setTY(const & y){this->tY=y;}
     inline void setTZ(const & z){this->tZ=z;}
     inline void setRX(const & theta){this->rX=theta;}
     inline void setRY(const & theta){this->rY=theta;}
     inline void setRZ(const & theta){this->rZ=theta;}
     inline void setToeX(const & x){this->toeX=x;}
     inline void setToeY(const & y){this->toeY=y;}
     inline void setToeZ(const & z){this->toeZ=z;}
     
     
     inline std::string getLabel(){return this->label;}
     inline double getTX(){return this->tX;}
     inline double getTY(){return this->tY;}
     inline double getTZ(){return this->tZ;}
     inline double getRX(){return this->rX;}
     inline double getRY(){return this->rY;}
     inline double getRZ(){return this->rZ;}
     
     inline double getToeX(){return this->toeX;}
     inline double getToeY(){return this->toeY;}
     inline double getToeZ(){return this->toeZ;}
     
     int print();
     
      
     // linear prediction based on constant velocity assumption 
     int predict (const double &xpred, const double &ypred, const double &zpred);
     
     // update the current state with measure
     int measure (const double &xmeas, const double &ymeas, const double &zmeas);  
    
     // compute the distance between the two feet
     double distance(const Foot &  otherFoot);
     
     // compute the min distance between two feet
     int whichFoot (const Foot & foot1, const Foot&foot2); 
    
    
  
    
    
  };
}
#endif
   
