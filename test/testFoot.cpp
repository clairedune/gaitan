#include <iostream>
#include <libgaitan/foot.h>


using namespace std;
using namespace gaitan;

int main() {
   
  Foot* footLeft = new Foot(0.1,-1,0);
  Foot* footRight = new Foot(0.1,1,0);

  footLeft->print();
  footRight->print();
  Foot::autoLabel(*footRight,*footLeft);
  footLeft->print();
  footRight->print();
  
  
  std::cout << "dist = "<< footLeft->distance(*footRight) << std::endl;
  
  Foot newFoot(0.2,1,0);
  
  std::cout << "which foot? (1-left, 2-right): ...  " << 
                newFoot.whichFoot(*footLeft,*footRight) << std::endl;
  
  double xprec(0), yprec(-1), zprec(0) ;
  footLeft->predict(xprec,yprec,zprec);
  footLeft->print();


  int label;
  label =  newFoot.whichFoot(*footLeft,*footRight);  
  std::cout << "which foot? (1-left, 2-right): ...  " << label<<
                std::endl;
  if(label == 1)
      footLeft->update(newFoot);
  else 
      footRight->update(newFoot);
  
  //delete encoder;
  return 1;
}
