// Compile it with:  
//   g++ -o example-vector example-vector.cc -lboost_iostreams -lboost_system -lboost_filesystem

#include <vector>  
#include <cmath>  
#include <boost/tuple/tuple.hpp>

#include <gnuplot-iostream.h>

#include <libgaitan/plot.h>
using namespace std;

namespace gaitan{

	Plot::Plot(){  
		
	}

        Plot::~Plot(){}

        int Plot::plotEncoder(Encoder* encoder)
	{
                if (encoder->_data->_nbRow>0&&encoder->_data->_nbCol>0){
			// Gnuplot vectors (i.e. arrows) require four columns: (time,x,y,theta)  
			std::vector<boost::tuple<double, double, double> > pts_odo;
			for (int i=0 ; i<encoder->_data->_nbRow ; i++){
				pts_odo.push_back(boost::make_tuple(  
				//encoder->_data->_data[i][0],  
				encoder->_data->_data[i][1], 
				encoder->_data->_data[i][2],  
				encoder->_data->_data[i][3]  
				));
			}
		
	
		// Don't forget to put "\n" at the end of each line!  
		//gp << "set xrange [0:2]\nset yrange [-2:2]\n";  
		// '-' means read from stdin.  The send1d() function sends data to  
		// gnuplot's stdin.  
		gp << "plot '-' with line title 'odometry'\n"; 
		gp.send1d(pts_odo);  
		//gp.send1d(boost::make_tuple(pts_B_x, pts_B_y, pts_B_dx, pts_B_dy));  
		return 0;
		}
		else return -1;
	}
}
