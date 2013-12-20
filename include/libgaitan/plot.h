// This file is a c++ wrapper of gnuplot
#ifndef GNUPLOT_H
#define GNUPLOT_H

#include <vector>
#include <cmath>  
#include <boost/tuple/tuple.hpp>
#include <gnuplot-iostream.h>
#include <libgaitan/encoder.h>


namespace gaitan{
class Plot{  
        public :  
           static int test();  
	   static int plotEncoder(Encoder*encoder);     
};
}
#endif
