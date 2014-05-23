
#include <libgaitan/foot.h>
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <pcl/common/common_headers.h>

   
using namespace Eigen;

namespace gaitan
{
     Foot::Foot():label(UNKNOWN),tX(0),tY(0),tZ(0),rX(0),rY(0), rZ(0),
     toeX(0),toeY(0),toeZ(0),length(0.25)
     {return;}

     Foot::Foot(const double &x, const double &y, const double &z):
     label(UNKNOWN),tX(x),tY(y),tZ(z),rX(0),rY(0),rZ(0),
     toeX(0),toeY(0),toeZ(0),length(0.25)
     {return;}
     
     
     Foot::~Foot(){return;}

     // assuming that the left foot has the biggest y
     void Foot::autoLabel(Foot &foot1, Foot &foot2)
     {
       
       if(foot1.getTY()>foot2.getTY()) // left
        {
           foot1.setLabel(LEFT);
           foot2.setLabel(RIGHT);
        }
      else
        {
           foot1.setLabel(RIGHT);
           foot2.setLabel(LEFT);
        }
       
     } 

      // linear prediction based on constant velocity assumption and 
      // translational motion
     int Foot::predict (const double &xpred, const double &ypred, const double &zpred)
     {
       // v(t) = (x(t)-x(t-1))/dt
       // x(t+1)=x(t)+v(t)*dt= x(t)+ dt*(x(t)-x(t-1))/dt = 2x(t)-x(t-1)
       this->tX = 2*this->tX-xpred;
       this->tY = 2*this->tY-ypred;
       this->tZ = 2*this->tZ-zpred;
       
       return 1;
     }
     
     // print information
     void Foot::print()
     {
        std::cout << "----------" <<  std::endl;
      
        if (this->label==Foot::LEFT)
           std::cout << "Foot " << "left" << std::endl;
    
        if (this->label==Foot::RIGHT)
           std::cout << "Foot " << "right" << std::endl;
    
    
        std::cout << "----------" <<  std::endl;
        std::cout << "x: \t"<<this->tX  << std::endl;  
        std::cout << "y: \t"<<this->tY  << std::endl;  
        std::cout << "z: \t"<<this->tZ  << std::endl;  
        std::cout << "rx: \t"<<this->rX  << std::endl;  
        std::cout << "ry: \t"<<this->rY  << std::endl;  
        std::cout << "rz: \t"<<this->rZ  << std::endl; 
        std::cout << "toex: \t"<<this->toeX  << std::endl;  
        std::cout << "toey: \t"<<this->toeY  << std::endl;  
        std::cout << "toez: \t"<<this->toeZ  << std::endl;   
        
        return;
       
     }
     
     // update the current state with measure
     int Foot::update (const double &xmeas, const double &ymeas, const double &zmeas)
     {
          this->tX = xmeas;  
          this->tY = ymeas;  
          this->tZ = zmeas;  
          
          return 1;
     }
     // update the current state with measure
     int Foot::update (Foot & foot)
     {
          this->tX = foot.getTX();  
          this->tY = foot.getTY();  
          this->tZ = foot.getTZ();  
          this->rX = foot.getRX();  
          this->rY = foot.getRY();  
          this->rZ = foot.getRZ();  
          
          this->toeX = foot.getToeX();  
          this->toeY = foot.getToeY();  
          this->toeZ = foot.getToeZ();  
          
          this->length = foot.getLength();
          
      
          
          return 1;
     }
     
    int Foot::update(Box & box)
     {
          float x,y,z;
          box.center(x,y,z);
          this->tX = x;  
          this->tY = y;  
          this->tZ = z;  
//          this->rX = foot.getRX();  
//          this->rY = foot.getRY();  
//          this->rZ = foot.getRZ();  
            
          this->toeX = x;  
          this->toeY = y;  
          this->toeZ = box.getMinZ();  
          
          this->length = box.getMaxX()-box.getMinX();
          
      
          
          return 1;
     }
     
     // compute the distance between the two feet
     double Foot::distance(Foot &  otherFoot)
     {
         double dist(0);
         
        // dist  = (this->tX-otherFoot.getTX())*(this->tX-otherFoot.getTX());
         dist = (this->tY-otherFoot.getTY())*(this->tY-otherFoot.getTY());
         //dist += (this->tZ-otherFoot.getTZ())*(this->tZ-otherFoot.getTZ());
         
         return (sqrt(dist));
       
      }
     
     // compute the min distance between two feet
     //  return 1 if it is foot 1 and 2 if it is foot 2
     int Foot::whichFoot (Foot & foot1,Foot&foot2)
     {
       double dist1 (distance(foot1));
       double dist2 (distance(foot2));
       
       if(dist1<=dist2) return 1;
       else return 2;
       
     } 
   
}
