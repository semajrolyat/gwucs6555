/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

Renderer class implementation

------------------------------------------------------------------------------*/
#include <cs6555/Renderer.h>

#include <stdio.h>
#include <GL/glut.h>

#include <cs6555/Constants.h>


//------------------------------------------------------------------------------
// Utility Functions
//------------------------------------------------------------------------------

void Renderer::activate( const Material& material ) {
    glMaterialfv( GL_FRONT, GL_AMBIENT, (GLfloat*)material.ambient.arrayOpenGL() );
    glMaterialfv( GL_FRONT, GL_DIFFUSE, (GLfloat*)material.diffuse.arrayOpenGL() );
    glMaterialfv( GL_FRONT, GL_SPECULAR, (GLfloat*)material.specular.arrayOpenGL() );
    glMaterialfv( GL_FRONT, GL_EMISSION, (GLfloat*)material.emissive.arrayOpenGL() );
    glMaterialf( GL_FRONT, GL_SHININESS, (GLfloat)material.shininess );
}

//------------------------------------------------------------------------------
// Drawing Functions
//------------------------------------------------------------------------------
void Renderer::draw( Mesh* mesh ) {
    glBegin( GL_TRIANGLES );
    for( unsigned int poly_id = 0; poly_id < mesh->polygonCount( ); poly_id++ ) {
        // Select the current polygon
        Polygon* poly = mesh->polygon( poly_id );

        Vertex *v0, *v1, *v2;

        unsigned int verts = poly->numVertices( );
        if( verts < 3 ) continue;   // sanity check -> malformed poly & bad juju

        // the model is not tessellated, so have to tessellate for OpenGL
        // If poly is non-convex this won't work, but assume convex.

        // Select the first vertex as the root of all triangles in the poly
        v0 = mesh->vertex( poly->getVertex( 0 ) );

        // Iterate over the rest of the vertices in the polygon up to the n-1 vert
        for( unsigned int poly_vert = 1; poly_vert < verts - 1; poly_vert++ ) {

            // select the current vertex
            v1 = mesh->vertex( poly->getVertex( poly_vert ) );
            // and the next vertex (for n-1 case this will be n so closes the poly)
            v2 = mesh->vertex( poly->getVertex( poly_vert + 1 ) );

            glVertex3d( v0->position.x(), v0->position.y(), v0->position.z() );
            glVertex3d( v1->position.x(), v1->position.y(), v1->position.z() );
            glVertex3d( v2->position.x(), v2->position.y(), v2->position.z() );
        }
    }
    glEnd();
}

//------------------------------------------------------------------------------

void Renderer::draw( Motivator* motivator ) {
    glPushMatrix();
    glLoadIdentity();

    Matrix4 transform = Matrix4::translationMatrix( motivator->position );
    glLoadMatrixd( transform.arrayOpenGL() );
    Renderer::draw( (Mesh*)motivator );
    for( unsigned int geometry_id = 0; geometry_id < motivator->geometryCount( ); geometry_id++ ) {
        Mesh* mesh = static_cast<Mesh*>( motivator->geometry( geometry_id ) );
        Renderer::draw( mesh );
    }

    glPopMatrix();
}

/*
//------------------------------------------------------------------------------

void Renderer::draw( Point3* point ) {

}
*/
//------------------------------------------------------------------------------
/*
void Renderer::draw( PointCloud* pointcloud ) {
    glPushMatrix();

    glPointSize( pointcloud->count() );

    glBegin( GL_POINTS );

    std::vector<Point3*> points = pointcloud->points();
    for( std::vector<Point3*>::iterator it = points.begin(); it != points.end(); it++ ) {
        Point3* pt = *it;

        //glColor4d( (GLdouble)pt->color.red(), (GLdouble)pt->color.green(), (GLdouble)pt->color.blue(), (GLdouble)pt->color.alpha() );
        glColor3d( (GLdouble)pt->color.red(), (GLdouble)pt->color.green(), (GLdouble)pt->color.blue() );

        glVertex3d( (GLdouble)pt->position.x(), (GLdouble)pt->position.y(), (GLdouble)pt->position.z() );
    }
\
    glEnd();

    glPopMatrix();
}

//------------------------------------------------------------------------------

void Renderer::draw( ParticleSoup* soup ) {

    unsigned int n = soup->count();
    for( unsigned int i = 0; i < n; i++ ) {
        // Select the current point
        Point3* pt = soup->point( i );

        //glColor4d( (GLdouble)pt->color.red(), (GLdouble)pt->color.green(), (GLdouble)pt->color.blue(), (GLdouble)pt->color.alpha() );
        glColor3d( (GLdouble)pt->color.red(), (GLdouble)pt->color.green(), (GLdouble)pt->color.blue() );

        glVertex3d( (GLdouble)pt->position.x(), (GLdouble)pt->position.y(), (GLdouble)pt->position.z() );
    }

}
*/
//------------------------------------------------------------------------------

void Renderer::draw( Trajectory* trajectory ) {
    Eigen::MatrixXd M;

    activate( trajectory->material );
    M = trajectory->spline_basis;

    Vector3 vi, vi1;
    unsigned int n = trajectory->controlpoints.size();

    glLineWidth( trajectory->line_width );

    glPushMatrix();

    glLoadMatrixd( trajectory->transform.arrayOpenGL() );

    glBegin(GL_LINES);

    for( unsigned int i = 2; i < n-1; i++ ) {
        Eigen::MatrixXd C = CubicSpline::blend( M, trajectory->controlpoints, i );

        std::vector<Vector3> points;
        for( double u = 0.0; u < 1.0; u += 0.1 ) {
            points.push_back( CubicSpline::position( C, u ) );
        }
        points.push_back( CubicSpline::position( C, 1.0 ) );

        for( std::vector<Vector3>::iterator it = points.begin(); it != points.end(); it++ ) {
            if( it == points.begin() ) {
                vi = *it;
            } else {
                vi1 = *it;
                glVertex3d( vi.x(), vi.y(), vi.z() );
                glVertex3d( vi1.x(), vi1.y(), vi1.z() );
                vi = *it;
            }
        }
    }

    glEnd();

    glPopMatrix();
}

//------------------------------------------------------------------------------

void Renderer::draw( Body* body ) {

    glPushMatrix();

    activate( body->material );

    glLoadMatrixd( body->pose.transform.arrayOpenGL() );

    for( unsigned int geometry_id = 0; geometry_id < body->geometryCount( ); geometry_id++ ) {
        Mesh* mesh = static_cast<Mesh*>( body->geometry( geometry_id ) );
        Renderer::draw( mesh );
    }

    glPopMatrix();

}

//------------------------------------------------------------------------------

#include <cs6555/Joint.h>

void Renderer::draw( Link* link ) {

    activate( link->material );

    glLoadMatrixd( link->pose.transform.arrayOpenGL() );

    for( unsigned int geometry_id = 0; geometry_id < link->geometryCount( ); geometry_id++ ) {
        Mesh* mesh = static_cast<Mesh*>( link->geometry( geometry_id ) );
        draw( mesh );
    }

    for( unsigned int i = 0; i < link->child_joints(); i++ ) {
        Joint* joint = link->child_joint( i );
        draw( joint->outboard_link );
    }
}

//------------------------------------------------------------------------------

void Renderer::draw( ArticulatedBody* body ) {

    // By requirements can only have one push and pop of OpenGL's matrix stack
    // Therefore do that here then descend the hierarchy
    glPushMatrix();

    // recursive descent beginning with the root
    draw( body->root_link );

    glPopMatrix();

}

//------------------------------------------------------------------------------

void Renderer::draw( DeformableBody* body ) {

    // By requirements can only have one push and pop of OpenGL's matrix stack
    // Therefore do that here then descend the hierarchy
    glPushMatrix();

    draw( body->mesh );

    glPopMatrix();

}

//------------------------------------------------------------------------------

void Renderer::draw( Scene* scene ) {
    glPushMatrix();

    for( unsigned int i = 0; i < scene->geometryCount(); i++ ) {
        Geometry* geometry = scene->geometry( i );
        if( geometry->geometry_type() == GEOMETRY_TYPE_BODY ) {
            Body* body = static_cast<Body*>( geometry );
            if( body->body_type() == BODY_TYPE_RIGID ) {
                RigidBody* rbody = static_cast<RigidBody*>( body );

                glPushMatrix();

                glLoadMatrixd( rbody->pose.transform.arrayOpenGL() );

                for( unsigned int i = 0; i < rbody->geometryCount(); i++ ) {
                    Geometry* geometry = rbody->geometry( i );
                    if( geometry->geometry_type() == GEOMETRY_TYPE_MESH ) {
                        Mesh* mesh = static_cast<Mesh*>( geometry );
                        draw( mesh );
                    }
                }

                glPopMatrix();
            }
        } else if( geometry->geometry_type() == GEOMETRY_TYPE_MESH ) {
            Mesh* mesh = static_cast<Mesh*>( geometry );
            draw( mesh );
        }
    }

    glPopMatrix();
}

//------------------------------------------------------------------------------

void Renderer::draw( Particle* particle ) {

    glPointSize( 1.0 );

    glBegin( GL_POINTS );

    glColor3d( (GLdouble)particle->color.r(), (GLdouble)particle->color.g(), (GLdouble)particle->color.b() );

    glVertex3d( (GLdouble)particle->position.x(), (GLdouble)particle->position.y(), (GLdouble)particle->position.z() );

    glEnd();
}

//------------------------------------------------------------------------------

void Renderer::draw( ParticleCloud* cloud ) {
    glPushMatrix();

    for( unsigned int i = 0; i < cloud->geometryCount(); i++ ) {
        Geometry* g = cloud->geometry( i );
        if( g->geometry_type() == GEOMETRY_TYPE_PARTICLE ) {
            Particle* p = static_cast<Particle*>( g );

            if( p->particle_type() == PARTICLE_TYPE_CLOUD ) {
                /*
                glPushMatrix();
                glLoadIdentity();

                Matrix4 transform = Matrix4::translationMatrix( p->position );
                glLoadMatrixd( transform.arrayOpenGL() );
*/
                ParticleCloud* pc = static_cast<ParticleCloud*>( p );
                draw( (ParticleCloud*)pc );
/*
                glPopMatrix();
                */
            } else {
                draw( (Particle*)p );
            }
        } else if( g->geometry_type() == GEOMETRY_TYPE_MESH ) {
            draw( (Mesh*)g );
        }
    }

    glPopMatrix();
}

//------------------------------------------------------------------------------
