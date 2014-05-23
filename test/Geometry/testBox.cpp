/*!
 * Test the plane class 
 */

#include <iostream>

#include <Eigen/Dense>

#include <libgaitan/box.h>

using namespace gaitan;

int main() {
              
    Box box(-1.0,1,-1.0,-2,0,3);
    box.print();
     
    return 1;      
}
