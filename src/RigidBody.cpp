/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

RigidBody class implementation

------------------------------------------------------------------------------*/

#include <cs6555/RigidBody.h>

#include <assert.h>

#include <cs6555/Mesh.h>
#include <cs6555/MeshLoader.h>

//------------------------------------------------------------------------------
// Constructors
//------------------------------------------------------------------------------
/// Default Constructor
RigidBody::RigidBody( void ) {
    rigidbody_type = RIGIDBODY_DYNAMIC;
    coefficient_of_restitution = 1.0;
}

//------------------------------------------------------------------------------

RigidBody::RigidBody( const ERigidBodyType& type ) {
    rigidbody_type = type;
    coefficient_of_restitution = 1.0;
}

//------------------------------------------------------------------------------
// Destructor
//------------------------------------------------------------------------------
/// Destructor
RigidBody::~RigidBody( void ) {
    if( !m_shallow_copy && m_bv != NULL )
        delete m_bv;
}

/*
//------------------------------------------------------------------------------
// [Base Class] Geometry::Queries
//------------------------------------------------------------------------------
/// Geometry Type Query.  Implemented by inheritor.
EGeometryType RigidBody::geometry_type( void ) {
    return GEOMETRY_TYPE_UNKNOWN;
}
*/
//------------------------------------------------------------------------------
// [Base Class] Body::Queries
//------------------------------------------------------------------------------
/// Body Type Query.  Implemented by inheritor.
EBodyType RigidBody::body_type( void ) {
    return BODY_TYPE_RIGID;
}


void RigidBody::shallow_copy( RigidBody* rb ) {

    m_shallow_copy = true;
    m_origin = rb;

    copy_origin();
}

//------------------------------------------------------------------------------

void RigidBody::sphere( const double& mass, const double& radius ) {
    this->mass = mass;
    //Mesh* sphere = MeshLoader::sphere( radius, 32, 15 );
    Mesh* sphere = MeshLoader::sphere( radius, 12, 5 );
    insert( sphere );
    body_frame_inertial_tensor.identity();
    body_frame_inertial_tensor(0,0) = body_frame_inertial_tensor(1,1) = body_frame_inertial_tensor(2,2) = 2.0/5.0 * mass * radius * radius;
    body_frame_inertial_tensor_inverse = Matrix3::inverse( body_frame_inertial_tensor );
}

//------------------------------------------------------------------------------

void RigidBody::sphere( const double& mass, const double& radius, const unsigned int& segments, const unsigned int& slices ) {
    this->mass = mass;
    Mesh* sphere = MeshLoader::sphere( radius, segments, slices, NULL, ML_MATERIAL_RANDOM );
    insert( sphere );
    body_frame_inertial_tensor.identity();
    body_frame_inertial_tensor(0,0) = body_frame_inertial_tensor(1,1) = body_frame_inertial_tensor(2,2) = 2.0/5.0 * mass * radius * radius;
    body_frame_inertial_tensor_inverse = Matrix3::inverse( body_frame_inertial_tensor );
}

//------------------------------------------------------------------------------

void RigidBody::sphere( const double& mass, const double& radius, const unsigned int& segments, const unsigned int& slices, Material* material ) {
    this->mass = mass;
    Mesh* sphere = MeshLoader::sphere( radius, segments, slices, material, ML_MATERIAL_ASSIGN );
    insert( sphere );
    body_frame_inertial_tensor.identity();
    body_frame_inertial_tensor(0,0) = body_frame_inertial_tensor(1,1) = body_frame_inertial_tensor(2,2) = 2.0/5.0 * mass * radius * radius;
    body_frame_inertial_tensor_inverse = Matrix3::inverse( body_frame_inertial_tensor );
}

//------------------------------------------------------------------------------

void RigidBody::cube( const double& mass, const double& width, const double& height, const double& depth ) {
    this->mass = mass;
    Mesh* box = MeshLoader::box( width, height, depth );
    insert( box );
    body_frame_inertial_tensor.identity();
    body_frame_inertial_tensor(0, 0) = 1.0/12.0 * mass * (height * height + depth * depth);
    body_frame_inertial_tensor(1, 1) = 1.0/12.0 * mass * (width * width + depth * depth);
    body_frame_inertial_tensor(2, 2) = 1.0/12.0 * mass * (width * width + height * height);
    body_frame_inertial_tensor_inverse = Matrix3::inverse( body_frame_inertial_tensor );
}

//------------------------------------------------------------------------------

void RigidBody::cube( const double& mass, const double& width, const double& height, const double& depth, Material* material ) {
    this->mass = mass;
    Mesh* box = MeshLoader::box( width, height, depth, material, ML_MATERIAL_ASSIGN );
    insert( box );
    body_frame_inertial_tensor.identity();
    body_frame_inertial_tensor(0, 0) = 1.0/12.0 * mass * (height * height + depth * depth);
    body_frame_inertial_tensor(1, 1) = 1.0/12.0 * mass * (width * width + depth * depth);
    body_frame_inertial_tensor(2, 2) = 1.0/12.0 * mass * (width * width + height * height);
    body_frame_inertial_tensor_inverse = Matrix3::inverse( body_frame_inertial_tensor );
}

//------------------------------------------------------------------------------

void RigidBody::copy_origin( void ) {
    assert( m_origin != NULL );
    RigidBody* origin = static_cast<RigidBody*>( m_origin );

    rigidbody_type = origin->rigidbody_type;

    mass = origin->mass;
    coefficient_of_restitution = origin->coefficient_of_restitution;

    m_bv = origin->m_bv;

    //pose = origin->pose;
    pose = Pose( origin->pose );

    body_frame_inertial_tensor = origin->body_frame_inertial_tensor;
    body_frame_inertial_tensor_inverse = origin->body_frame_inertial_tensor_inverse;

    position = origin->position;
    rotation = origin->rotation;
    linear_momentum = origin->linear_momentum;
    angular_momentum = origin->angular_momentum;

    inertial_tensor_inverse = origin->inertial_tensor_inverse;
    linear_velocity = origin->linear_velocity;
    angular_velocity = origin->angular_velocity;

    force = origin->force;
    torque = origin->torque;

}

//------------------------------------------------------------------------------

void RigidBody::update_origin( void ) {
    assert( m_origin != NULL );
    RigidBody* origin = static_cast<RigidBody*>( m_origin );
    //origin->mass = this->mass;

    origin->body_frame_inertial_tensor = this->body_frame_inertial_tensor;
    origin->body_frame_inertial_tensor_inverse = this->body_frame_inertial_tensor_inverse;

    origin->position = this->position;
    origin->rotation = this->rotation;
    origin->linear_momentum = this->linear_momentum;
    origin->angular_momentum = this->angular_momentum;

    origin->inertial_tensor_inverse = this->inertial_tensor_inverse;
    origin->linear_velocity = this->linear_velocity;
    origin->angular_velocity = this->angular_velocity;

    origin->force = this->force;
    origin->torque = this->torque;

    //origin->pose = this->pose;
    origin->pose = Pose( this->pose );
}
