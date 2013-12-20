// Compile it with:  
//   g++ -o example-vector example-vector.cc -lboost_iostreams -lboost_system -lboost_filesystem

#include <vector>  
#include <cmath>  
#include <boost/tuple/tuple.hpp>

#include <libgaitan/plot.h>
#include <libgaitan/encoder.h>
using namespace std;

namespace gaitan{

        int Plot::test()
	{
          Gnuplot gp;
	
	// Gnuplot vectors (i.e. arrows) require four columns: (x,y,dx,dy)  
	std::vector<boost::tuple<double, double, double, double> > pts_A;
	
	// You can also use a separate container for each column, like so:  
	std::vector<double> pts_B_x;  
	std::vector<double> pts_B_y;  
	std::vector<double> pts_B_dx;  
	std::vector<double> pts_B_dy;
	
	// Make the data  
	for(double alpha=0; alpha<1; alpha+=1.0/24.0) {  
		double theta = alpha*2.0*3.14159;  
		pts_A.push_back(boost::make_tuple(  
		cos(theta),  
		sin(theta), 
		-cos(theta)*0.1,  
		sin(theta)*0.1  
		));
		
		pts_B_x .push_back( cos(theta)*0.8);  
		pts_B_y .push_back( sin(theta)*0.8);  
		pts_B_dx.push_back( sin(theta)*0.1);  
		pts_B_dy.push_back(-cos(theta)*0.1);  
	}
	
	// Don't forget to put "\n" at the end of each line!  
	gp << "set xrange [-2:2]\nset yrange [-2:2]\n";  
	// '-' means read from stdin.  The send1d() function sends data to  
	// gnuplot's stdin.  
	gp << "plot '-' with vectors title 'pts_A', "  
	<< "'-' with vectors title 'pts_B'\n";  
	gp.send1d(pts_A);  
	gp.send1d(boost::make_tuple(pts_B_x, pts_B_y, pts_B_dx, pts_B_dy));  		
          return 0;
	}

	 int Plot::plotEncoder(Encoder*encoder)
	{
          Gnuplot gp;
	
	// Gnuplot vectors (i.e. arrows) require four columns: (x,y,theta)  
	std::vector<boost::tuple<double, double> > pts_X;
	
	// Make the data  
	for(int i=0; i<encoder->_data->_nbRow; i++) {  
		 
		pts_X.push_back(boost::make_tuple(  
		encoder->_data->_data[0][i],  
		encoder->_data->_data[1][i]
		));
	}
	//gp << "set xrange [1359736700:1359736900]\nset yrange [-2:2]\n";
	gp << "plot '-' with vectors title 'pts_X'\n" ;
	gp.send1d(pts_X);  
	  return 0;
	}
}
