#ifndef _CONTACTEVENT_H_
#define _CONTACTEVENT_H_

//------------------------------------------------------------------------------

#include <cs6555/Math/Vector3.h>
#include <cs6555/Event.h>
#include <cs6555/RigidBody.h>

//------------------------------------------------------------------------------

typedef enum {
    CE_VERTEX_FACE,
    CE_EDGE_EDGE,
    CE_VERTEX_VERTEX,       // degenerate
    CE_VERTEX_EDGE          // degenerate
} EContactEventType;

//------------------------------------------------------------------------------

class ContactEvent : public Event {
public:
    ContactEvent( void ) { }
    ContactEvent( const ContactEvent& c ) {
        body1 = c.body1;
        body2 = c.body2;
        point = c.point;
        normal = c.normal;
        edge1 = c.edge1;
        edge2 = c.edge2;
        contact_event_type = c.contact_event_type;
    }
    ContactEvent( RigidBody* rb1, RigidBody* rb2 ) {
        body1 = rb1;
        body2 = rb2;
    }
    virtual ~ContactEvent( void ) { }

    virtual EEventType event_type( void ) { return EVENT_TYPE_CONTACT; }

    RigidBody*          body1;
    RigidBody*          body2;

    Vector3             point;
    Vector3             normal;
    Vector3             edge1;
    Vector3             edge2;

    EContactEventType   contact_event_type;
};

//------------------------------------------------------------------------------

#endif // _CONTACTEVENT_H_
