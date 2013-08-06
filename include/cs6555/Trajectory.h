/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

Trajectory class definition

------------------------------------------------------------------------------*/

#ifndef TRAJECTORY_H
#define TRAJECTORY_H

//------------------------------------------------------------------------------
#include <vector>
#include <eigen3/Eigen/Dense>

#include <cs6555/Pose.h>
#include <cs6555/Math/Vector3.h>
#include <cs6555/Trajectory.h>
#include <cs6555/Math/CubicSpline.h>
#include <cs6555/Keyframe.h>
#include <cs6555/Constants.h>
#include <cs6555/Material.h>

//------------------------------------------------------------------------------

typedef std::vector<Keyframe> KeyframeList;

//------------------------------------------------------------------------------

typedef enum {
    TRAJECTORY_CUBIC_SPLINE
} ETrajectoryType;

//------------------------------------------------------------------------------

class Trajectory {
public:
    // Constructor
    Trajectory( void );

    // Destructor
    virtual ~Trajectory( void );

    // Keyframe Insertion and Queries
    void insert_keyframe( const int& frame, const Pose& pose );
    Keyframe& keyframe( const unsigned int& index );
    unsigned int keyframes( void );
    void clear( void );

    // Utilities
    bool construct_trajectory( void );  // Note: invoke after inserting all keyframes to automatically build a trajectory from the keyframes

    // Canned Trajectories
    void construct_line_trajectory( void );
    void construct_circle_trajectory( void );
    void construct_sine_trajectory( void );
    void construct_multisine_trajectory( void );
    void construct_immelmann_trajectory( void );
    void construct_doubleimmelmann_trajectory( void );
    void construct_loop_trajectory( void );

    void construct_pendulum_trajectory( const unsigned int& period_in_frames, const bool& forward = true, const Vector3& orientation = Vector3(0.0, PI, 0.0), const double& frequency = 2*PI, const double& amplitude = 1.0 );
    void construct_cyclic_circle_trajectory( const unsigned int& period_in_frames, const bool& forward = true, const Vector3& orientation = Vector3(0.0, PI, 0.0), const double& radius = 1.0, const Vector3& center = Vector3(0.0, 0.0, 0.0) );

    void construct_cyclic_sine_trajectory( void );

    void construct_random_trajectory( const unsigned int& num_keyframes );

    // Queries
    Vector3 position( const double& distance );
    Vector3 tangent( const double& distance );
    Vector3 normal( const double& distance );

    // Member Data
    std::vector<Pose> poses;                // the list of keyframed poses along the spline
    ControlPointList controlpoints;         // the list of control points composing the spline

    Eigen::MatrixXd spline_basis;           // the basis matrix for the spline
    std::vector<SdS> spline_arclength_map;  // the map of distances over the spline

    unsigned int prev_keyframe_id;      // the previous keyframe the trajectory passed through based on current position
    unsigned int next_keyframe_id;      // the next keyframe the trajectory will pass through based on current position

    bool cyclic;        // whether or not the trajectory is an infinite cycle
    bool reversible;    // whether or not the trajectory is reversible
    bool forward;       // whether or not the trajectory is being traversed forward or backward

    unsigned int frame_tic;      // the current frame tic that trajectory is at

    Material material;  // used when rendering the trajectory

    Matrix4 transform;
    double line_width;
private:
    KeyframeList m_keyframes;       // the list of keyframes
    unsigned int m_keyframe_count;  // the number of keyframes in the list

};

//------------------------------------------------------------------------------

#endif // TRAJECTORY_H
