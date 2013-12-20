// This file is a c++ wrapper of gnuplot

#include <vector>  
#include <cmath>  
#include <boost/tuple/tuple.hpp>

#include <gnuplot-iostream.h>


#include <libgaitan/encoder.h>


#ifndef PLOT_H
#define PLOT_H
namespace gaitan{
class Plot{  
        public :  
           Gnuplot gp;
           
 	   Plot();
           ~Plot();

           int plotEncoder(Encoder * encoder);        
};
}
#endif
