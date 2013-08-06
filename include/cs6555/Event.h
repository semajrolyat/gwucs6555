#ifndef _EVENT_H_
#define _EVENT_H_

//------------------------------------------------------------------------------

typedef enum {
    EVENT_TYPE_UNKNOWN,
    EVENT_TYPE_CONTACT
} EEventType;

//------------------------------------------------------------------------------

class Event {
public:
    Event( void ) { }
    virtual ~Event( void ) { }

    virtual EEventType event_type( void ) { return EVENT_TYPE_UNKNOWN; }
};

//------------------------------------------------------------------------------

#endif // _EVENT_H_
