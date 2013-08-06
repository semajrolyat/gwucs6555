#ifndef _NODE_H_
#define _NODE_H_

#include <cs6555/Vertex.h>
#include <vector>

class Edge;

class Node {
public:
    Node( void ) { }
    virtual ~Node( void ) {
        vertex = NULL;
    }

    std::vector<Edge*> edges;

    bool insert( Edge* edge ) {
        for( std::vector<Edge*>::iterator it = edges.begin(); it != edges.end(); it++ ) {
            if( (*it) == edge )
                return false;
        }
        edges.push_back( edge );
        return true;
    }

    Vertex* vertex;
    double mass;
};

#endif // _NODE_H_
