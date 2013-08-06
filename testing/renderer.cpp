#include <Renderer.h>

#include <map>
#include <string>

extern std::map<std::string,int> RendererSettings;

int main( int argc, char* argv[] ) {
    Renderer renderer;

    renderer.init( argc, argv );
    renderer.start();

    return 0;
}

