#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>


#include <Eigen/Core>
#include <Eigen/Geometry>

#include "arcball.hpp"

using namespace std;
using namespace Eigen;

/**
* \ingroup GLVisualization
 * Default constructor, it sets the ballRadius to 600
**/
Arcball::Arcball()
{  this->ballRadius=600;
   isRotating=false;
   width=height=0;
   reset();
}

/**
 * \ingroup GLVisualization
* Set width and height of the current windows, it's needed every time you resize the window
* \param w Width of the rendering window
* \param h Height of the rendering window
 **/
void Arcball::setWidthHeight(int w, int h)
{  width=w;
   height=h;
   ballRadius = min((int)(w/2), (int)(h/2));
}

/**
 * \ingroup GLVisualization
* Set the radius of the ball (a typical radius for a 1024x768 window is 600
* \param newRadius The radius of the spherical dragging area
 **/
void Arcball::setRadius(float newRadius)
{  ballRadius = newRadius;
}

/**
 * \ingroup GLVisualization
* Start the rotation. Use this method in association with the left click.
* Here you must give directly the coordinates of the mouse as the glut functions extract. This method supposes that the 0,0 is in the upper-left part of the screen
* \param _x Horizontal position of the mouse (0,0) = upperleft corner (w,h) = lower right
* \param _y Vertical position of the mouse (0,0) = upperleft corner (w,h) = lower right
 *
**/
void Arcball::startRotation(int _x, int _y)
{  int x = ( (_x)-(width/2) );
   int y = ((height/2)-_y);

   startRotationVector = convertXY(x,y);
   startRotationVector.normalize();

   currentRotationVector=  startRotationVector;
   isRotating = true;
}

/**
 * \ingroup GLVisualization
* Update the rotation. Use this method in association with the drag event.
* Here you must give directly the coordinates of the mouse as the glut functions extract. This method supposes that the 0,0 is in the upper-left part of the screen
* \param _x Horizontal position of the mouse (0,0) = upperleft corner (w,h) = lower right
* \param _y Vertical position of the mouse (0,0) = upperleft corner (w,h) = lower right
**/
void Arcball::updateRotation(int _x, int _y)
{  int x = ( (_x)-(width/2) );
   int y = ((height/2)-_y);

   currentRotationVector = convertXY(x,y);

   currentRotationVector.normalize();
}

/**
* \ingroup GLVisualization
* Apply the computed rotation matrix
* This method must be invoked inside the \code glutDisplayFunc() \endcode
*
**/
void Arcball::applyRotationMatrix()
{  if (isRotating)
   {  // Do some rotation according to start and current rotation vectors
      //cerr << currentRotationVector.transpose() << " " << startRotationVector.transpose() << endl;
      if ( ( currentRotationVector - startRotationVector).norm() > 1E-6 )
      {  Vector3d rotationAxis = currentRotationVector.cross(startRotationVector);
         rotationAxis.normalize();

         double val = currentRotationVector.dot(startRotationVector);
         val > (1-1E-10) ? val=1.0 : val=val ;
         double rotationAngle = acos(val) * 180.0f/(float)M_PI;

         // rotate around the current position
         applyTranslationMatrix(true);
         glRotatef(rotationAngle * 2, -rotationAxis.x(),  -rotationAxis.y(),-rotationAxis.z());
         applyTranslationMatrix(false);
      }
   }
   glMultMatrixf(startMatrix);
}

/**
 * \ingroup GLVisualization
* Stop the current rotation and prepare for a new click-then-drag event
*
**/
void Arcball::stopRotation()
{

   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();
   applyRotationMatrix();
   // set the current matrix as the permanent one
   glGetFloatv(GL_MODELVIEW_MATRIX, startMatrix);
   isRotating = false;
}


/**
* \ingroup GLVisualization
* Apply the translation matrix to the current transformation (zoom factor)
**/
void Arcball::applyTranslationMatrix(bool reverse)
{  float factor = (reverse?-1.0f:1.0f);
   float tx = transX + (currentTransX - startTransX)*TRANSLATION_FACTOR;
   float ty = transY + (currentTransY - startTransY)*TRANSLATION_FACTOR;
   glTranslatef(factor*tx,  factor*(-ty), 0);
}

/**
 * \ingroup GLVisualization
* Maps the mouse coordinates to points on a sphere, if the points lie outside the sphere, the z is 0, otherwise is \f$ \sqrt(r^2 - (x^2+y^2) ) \f$ where \f$ x,y \f$
* are the window centric coordinates of the mouse
* \param x Mouse x coordinate
* \param y Mouse y coordinate
**/
Vector3d Arcball::convertXY(int x, int y)
{

   int d = x*x+y*y;
   float radiusSquared = ballRadius*ballRadius;
   if (d > radiusSquared)
   {  return Vector3d((float)x,(float)y, 0 );
   }
   else
   {  return Vector3d((float)x,(float)y, sqrt(radiusSquared - d));
   }
}

/**
 * \ingroup GLVisualization
 * Reset the current transformation to the identity
**/
void Arcball::reset()
{  fov = INITIAL_FOV;
   // reset matrix
   memset(startMatrix, 0, sizeof(startMatrix));
   startMatrix[0] = 1;
   startMatrix[1] = 0;
   startMatrix[2] = 0;
   startMatrix[3] = 0;
   startMatrix[4] = 0;
   startMatrix[5] = 1;
   startMatrix[6] = 0;
   startMatrix[7] = 0;
   startMatrix[8] = 0;
   startMatrix[9] = 0;
   startMatrix[10] = 1;
   startMatrix[11] = 0;
   startMatrix[12] = 0;
   startMatrix[13] = 0;
   startMatrix[14] = 0;
   startMatrix[15] = 1;

   transX = transY = 0;
   startTransX = startTransY = currentTransX = currentTransY = 0;
}


const float Arcball::INITIAL_FOV = 30;
const float Arcball::TRANSLATION_FACTOR = 0.01f;

