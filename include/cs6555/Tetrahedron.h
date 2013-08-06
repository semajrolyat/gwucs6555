#ifndef _TETRAHEDRON_H_
#define _TETRAHEDRON_H_

#include <assert.h>
#include <vector>
#include <cs6555/Node.h>
#include <cs6555/Edge.h>

class Tetrahedron {
public:
    Tetrahedron( void ) { }
    virtual ~Tetrahedron( void ) { }


    bool insert( Node* node ) {
        assert( m_nodes.size() < 4 );

        for( std::vector<Node*>::iterator it = m_nodes.begin(); it != m_nodes.end(); it++ ) {
            if( (*it) == node )
                return false;
        }
        m_nodes.push_back( node );
        return true;
    }
    bool insert( Edge* edge ) {
        assert( m_edges.size() < 6 );

        for( std::vector<Edge*>::iterator it = m_edges.begin(); it != m_edges.end(); it++ ) {
            if( (*it) == edge )
                return false;
        }
        m_edges.push_back( edge );
        return true;
    }

private:
    unsigned int m_nodeCount;
    unsigned int m_edgeCount;

    std::vector<Node*> m_nodes;
    std::vector<Edge*> m_edges;

};

#endif // _TETRAHEDRON_H_
