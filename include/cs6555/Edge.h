#ifndef _EDGE_H_
#define _EDGE_H_

#include <assert.h>
#include <cs6555/Node.h>

class Edge {
public:
    Edge( void ) {
        node0 = NULL;
        node1 = NULL;
    }
    Edge( Node* node0, Node* node1 ) {
        this->node0 = node0;
        this->node1 = node1;
    }
    virtual ~Edge( void ) {
        node0 = NULL;
        node1 = NULL;
    }

    Node* node0;
    Node* node1;

    bool insert( Node* node ) {
        if( is_member( node ) )
            return true;
        if( node0 == NULL ) {
            node0 = node;
            return true;
        }
        if( node1 == NULL ) {
            node1 = node;
            return true;
        }
        return false;
    }

    Node* node( const unsigned int& i ) {
        switch( i ) {
        case 0:
            return node0;
        case 1:
            return node1;
        default:
            return NULL;
        }
    }

    bool is_member( Node* node ) {
        assert( node != NULL );

        if( node == node0 )
            return true;
        if( node == node1 )
            return true;
        return false;
    }
};

#endif // _EDGE_H_
