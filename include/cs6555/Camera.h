/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

Camera class definition
------------------------------------------------------------------------------*/

#ifndef _CAMERA_H_
#define _CAMERA_H_

#include <cs6555/Math/Vector3.h>
#include <cs6555/Math/Matrix4.h>

//------------------------------------------------------------------------------

class Camera {

public:
    // Constructor
    Camera( void );

    // Destructor
    virtual ~Camera( void );

    // Projection
    void project( void );

    // Movement
    void track_left( const double& step );
    void track_right( const double& step );

    void dolly_in( const double& step );
    void dolly_out( const double& step );

    // Member Data
    Vector3 position;       // location of the camera
    Vector3 viewpoint;      // point where camera is focused

    Vector3 up;             // the up direction in world frame

    Matrix4 view;           // the view matrix
    Matrix4 perspective;    // the perspective matrix
    Matrix4 projection;     // the projection matrix
    Matrix4 projectionInv;  // the inverse projection matrix

    double near;            // distance to the near clipping plane
    double far;             // distance to the far clipping plane
    //double focusplane;

};

//------------------------------------------------------------------------------

#endif // _CAMERA_H
