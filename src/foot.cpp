
#include <libgaitan/box.h>
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <pcl/common/common_headers.h>

   
using namespace Eigen;

namespace gaitan
{
     Foot::Foot():label("no label"),tX(0),tY(0),tZ(0),rX(0),rY(0), rZ(0),
     toeX(0),toeY(0),toeZ(0),length(0.25)
     {return;}

      // linear prediction based on constant velocity assumption 
     int Foot::predict (const double &xpred, const double &ypred, const double &zpred)
     {
       // v(t) = (x(t)-x(t-1))/dt
       // x(t+1)=x(t)+v(t)*dt= x(t)+ dt*(x(t)-x(t-1))/dt = 2x(t)-x(t-1)
       
       
       
     }
     
     // print information
     void Foot::print()
     {
         
        std::cout << "----------" <<  std::endl;
        std::cout << "Foot " << this->label << std::endl;
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
       
     }
     
     // update the current state with measure
     int Foot::measure (const double &xmeas, const double &ymeas, const double &zmeas)
     {
          this->tX = xmeas;  
          this->tY = ymeas;  
          this->tZ = zmeas;  
     }
     
     // compute the distance between the two feet
     double Foot::distance(const Foot &  otherFoot)
     {
       
      }
     
     // compute the min distance between two feet
     int Foot::whichFoot (const Foot & foot1, const Foot&foot2)
     {
       
       
     } 
   
}
