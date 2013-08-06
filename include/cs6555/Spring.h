#ifndef _SPRING_H_
#define _SPRING_H_

#include <cs6555/Edge.h>

class Spring : public Edge {
public:
    Spring( void ) { }
    Spring( Node* node0, Node* node1, double kspring, double kdamping ) {
        this->node0 = node0;
        this->node1 = node1;
        spring_coefficient = kspring;
        damping_coefficient = kdamping;
    }

    virtual ~Spring( void ) { }

    double spring_coefficient;
    double damping_coefficient;
};

#endif // _SPRING_H_
