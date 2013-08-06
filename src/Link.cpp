#include <cs6555/Link.h>

#include <assert.h>

#include <cs6555/Joint.h>

#include <cs6555/Math/EulerAngle.h>

//------------------------------------------------------------------------------
// Constructors
//------------------------------------------------------------------------------
/// Default Constructor
Link::Link( void ) {
    parent_joint = NULL;
    center_of_mass = Vector3( 0.0, 0.0, 0.0 );
    m_number_child_joints = 0;
    name = "";
}
//------------------------------------------------------------------------------
/// Name Constructor
Link::Link( const std::string& name ) {
    parent_joint = NULL;
    center_of_mass = Vector3( 0.0, 0.0, 0.0 );
    m_number_child_joints = 0;
    this->name = name;
}

//------------------------------------------------------------------------------
// Destructors
//------------------------------------------------------------------------------
/// Destructor
Link::~Link( void ) {

}

//------------------------------------------------------------------------------
// [Base Class] Geometry::Queries
//------------------------------------------------------------------------------
/// Geometry Type Query.  Implemented by inheritor.
EGeometryType Link::geometry_type( void ) {
    return GEOMETRY_TYPE_MESH;
}

//------------------------------------------------------------------------------
/// Transform a link and recursively descend to transform all children in the
/// kinematic chain.  Uses the custom matrix stack to keep the current
/// transform readily available.  Executed during the idle calculation
void Link::transform( std::stack<Matrix4>& matrixstack ) {
    // grab the current transform off the top of the matrixstack
    Matrix4 current_transform = matrixstack.top();
    for( unsigned int i = 0; i < child_joints(); i++ ) {
        Joint* joint = child_joint( i );

        Matrix4 F1, X1, J1, X2, F2;
        // where F1 is Frame1 transformation, X1 is inboard displacement from F1 COM to joint
        // J1 is the joint orientation and rotation, X2 is outboard displacement from joint to F2 COM
        // and F2 is the resultant Frame2 transformation

        F1 = current_transform;
        X1 = Matrix4::translationMatrix( joint->inboard_displacement );

        // probably a more elegant way to transform from the union of coordinate frames defined at the joint
        // but for now the frame_transformation is provided by the implementor
        J1 = joint->frame_transformation * EulerAngle::matrix4( EULER_ANGLE_XYZ, EULER_HANDED_RIGHT, joint->angle );
        X2 = Matrix4::translationMatrix( Vector3::invert(joint->outboard_displacement) );

        F2 = F1 * X1 * J1 * X2;

        joint->outboard_link->pose.transform = F2;

        // push this link's transform
        matrixstack.push( joint->outboard_link->pose.transform );

        // recurse to descend the hierarchy
        joint->outboard_link->transform( matrixstack );

        // pop this link's transform
        matrixstack.pop();
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
///
Joint* Link::child_joint( const unsigned int& index ) {
    assert( index < m_number_child_joints );
    return m_child_joints.at( index );
}

//------------------------------------------------------------------------------
///
unsigned int Link::child_joints( void ) {
    return m_number_child_joints;
}

//------------------------------------------------------------------------------
///
void Link::append_child_joint( Joint* j ) {
    m_child_joints.push_back( j );
    m_number_child_joints++;
}

//------------------------------------------------------------------------------
