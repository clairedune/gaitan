/*
* This file is part of the OpenKinect Project. http://www.openkinect.org
*
* Copyright (c) 2010 individual OpenKinect contributors. See the CONTRIB file
* for details.
*
* This code is licensed to you under the terms of the Apache License, version
* 2.0, or, at your option, the terms of the GNU General Public License,
* version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
* or the following URLs:
* http://www.apache.org/licenses/LICENSE-2.0
* http://www.gnu.org/licenses/gpl-2.0.txt
*38
* If you redistribute this file in source form, modified or unmodified, you
* may:
* 1) Leave this header intact and distribute it under the same terms,
* accompanying it with the APACHE20 and GPL20 files, or
* 2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
* 3) Delete the GPL v2 clause and accompany it with the APACHE20 file
* In all cases you must keep the copyright notice intact and include a copy
* of the CONTRIB file.
*
* Binary distributions must follow the binary distribution requirements of
* either License.
*/

#if defined(__APPLE__)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif


// function for painting
void drawImage(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT) ;
        glBegin(GL_TRIANGLES);
		glVertex3f(-2, -2, -5.0);
		glVertex3f(2, 0.0, -5.0);		
		glVertex3f(0.0, 2, -5.0);
	glEnd();
	glutSwapBuffers();
}

//function for changing size 
void changeSize(int w, int h)
{
   	if(h==0) h=1;
   
   	float ratio = 1.0*w/h;       
   
   	//use projection matrix : define the viewing volume
   	glMatrixMode(GL_PROJECTION);

	//reset matrix : to init the viewing view
	glLoadIdentity();

        // viewport=entire windows
	glViewport(0,0,w,h);

	// perspective : field of view angle, ratio first point last point
	gluPerspective(45,ratio,1,100);

	// get back to the model view
        glMatrixMode(GL_MODELVIEW);
}



int main(int argc, char **argv) {
        glutInit(&argc, argv);
        glutInitDisplayMode(GLUT_DEPTH|GLUT_DOUBLE|GLUT_RGBA);
	glutInitWindowPosition(100,100);
	glutInitWindowSize(800,600);
        glutCreateWindow("Test read Image");

        // register call back
	glutDisplayFunc(drawImage);
        glutReshapeFunc(changeSize);

	glutMainLoop();	

        return 1;
}
