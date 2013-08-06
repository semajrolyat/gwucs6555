project( cs6555_test )
cmake_minimum_required(VERSION 2.8)

#find_package( cs6555 REQUIRED )

include_directories( ${CS6555_INCLUDE_DIRS} )

add_executable( simpleglut ${CMAKE_CURRENT_SOURCE_DIR}/simpleglut.cpp ${CS6555_HEADERS} ${CS6555_SOURCES} )
target_link_libraries( simpleglut ${CS6555_LIBRARIES} )
add_dependencies( simpleglut cs6555 )

add_executable( eigen ${CMAKE_CURRENT_SOURCE_DIR}/eigen.cpp ${CS6555_HEADERS} ${CS6555_SOURCES} )
target_link_libraries( eigen ${CS6555_LIBRARIES} )
add_dependencies( eigen cs6555 )

add_executable( vector3 ${CMAKE_CURRENT_SOURCE_DIR}/vector3.cpp ${CS6555_HEADERS} ${CS6555_SOURCES} )
target_link_libraries( vector3 ${CS6555_LIBRARIES} )
add_dependencies( vector3 cs6555 )

add_executable( vector4 ${CMAKE_CURRENT_SOURCE_DIR}/vector4.cpp ${CS6555_HEADERS} ${CS6555_SOURCES} )
target_link_libraries( vector4 ${CS6555_LIBRARIES} )
add_dependencies( vector4 cs6555 )

add_executable( matrix3 ${CMAKE_CURRENT_SOURCE_DIR}/matrix3.cpp ${CS6555_HEADERS} ${CS6555_SOURCES} )
target_link_libraries( matrix3 ${CS6555_LIBRARIES} )
add_dependencies( matrix3 cs6555 )

add_executable( matrix4 ${CMAKE_CURRENT_SOURCE_DIR}/matrix4.cpp ${CS6555_HEADERS} ${CS6555_SOURCES} )
target_link_libraries( matrix4 ${CS6555_LIBRARIES} )
add_dependencies( matrix4 cs6555 )

add_executable( quaternion ${CMAKE_CURRENT_SOURCE_DIR}/quaternion.cpp ${CS6555_HEADERS} ${CS6555_SOURCES} )
target_link_libraries( quaternion ${CS6555_LIBRARIES} )
add_dependencies( quaternion cs6555 )

#add_executable( renderer ${CMAKE_CURRENT_SOURCE_DIR}/renderer.cpp ${CS6555_HEADERS} ${CS6555_SOURCES} )
#target_link_libraries( renderer ${CS6555_LIBRARIES} )
#add_dependencies( renderer cs6555 )

add_executable( cubicspline ${CMAKE_CURRENT_SOURCE_DIR}/cubicspline.cpp ${CS6555_HEADERS} ${CS6555_SOURCES} )
target_link_libraries( cubicspline ${CS6555_LIBRARIES} )
add_dependencies( cubicspline cs6555 )

add_executable( speed ${CMAKE_CURRENT_SOURCE_DIR}/speed.cpp ${CS6555_HEADERS} ${CS6555_SOURCES} )
target_link_libraries( speed ${CS6555_LIBRARIES} )
add_dependencies( speed cs6555 )

add_executable( camera ${CMAKE_CURRENT_SOURCE_DIR}/camera.cpp ${CS6555_HEADERS} ${CS6555_SOURCES} )
target_link_libraries( camera ${CS6555_LIBRARIES} )
add_dependencies( camera cs6555 )

add_executable( tangent ${CMAKE_CURRENT_SOURCE_DIR}/tangent.cpp ${CS6555_HEADERS} ${CS6555_SOURCES} )
target_link_libraries( tangent ${CS6555_LIBRARIES} )
add_dependencies( tangent cs6555 )

add_executable( pose ${CMAKE_CURRENT_SOURCE_DIR}/pose.cpp ${CS6555_HEADERS} ${CS6555_SOURCES} )
target_link_libraries( pose ${CS6555_LIBRARIES} )
add_dependencies( pose cs6555 )

add_executable( trajectory ${CMAKE_CURRENT_SOURCE_DIR}/trajectory.cpp ${CS6555_HEADERS} ${CS6555_SOURCES} )
target_link_libraries( trajectory ${CS6555_LIBRARIES} )
add_dependencies( trajectory cs6555 )

add_executable( pendulum ${CMAKE_CURRENT_SOURCE_DIR}/pendulum.cpp ${CS6555_HEADERS} ${CS6555_SOURCES} )
target_link_libraries( pendulum ${CS6555_LIBRARIES} )
add_dependencies( pendulum cs6555 )

add_executable( cycliccircle ${CMAKE_CURRENT_SOURCE_DIR}/cycliccircle.cpp ${CS6555_HEADERS} ${CS6555_SOURCES} )
target_link_libraries( cycliccircle ${CS6555_LIBRARIES} )
add_dependencies( cycliccircle cs6555 )

add_executable( hip ${CMAKE_CURRENT_SOURCE_DIR}/hip.cpp ${CS6555_HEADERS} ${CS6555_SOURCES} )
target_link_libraries( hip ${CS6555_LIBRARIES} )
add_dependencies( hip cs6555 )

add_executable( biped ${CMAKE_CURRENT_SOURCE_DIR}/biped.cpp ${CS6555_HEADERS} ${CS6555_SOURCES} )
target_link_libraries( biped ${CS6555_LIBRARIES} )
add_dependencies( biped cs6555 )

add_executable( humanoid ${CMAKE_CURRENT_SOURCE_DIR}/humanoid.cpp ${CS6555_HEADERS} ${CS6555_SOURCES} )
target_link_libraries( humanoid ${CS6555_LIBRARIES} )
add_dependencies( humanoid cs6555 )

add_executable( shapes ${CMAKE_CURRENT_SOURCE_DIR}/shapes.cpp ${CS6555_HEADERS} ${CS6555_SOURCES} )
target_link_libraries( shapes ${CS6555_LIBRARIES} )
add_dependencies( shapes cs6555 )

add_executable( physics ${CMAKE_CURRENT_SOURCE_DIR}/physics.cpp ${CS6555_HEADERS} ${CS6555_SOURCES} )
target_link_libraries( physics ${CS6555_LIBRARIES} )
add_dependencies( physics cs6555 )

#add_executable( loadply ${CMAKE_CURRENT_SOURCE_DIR}/loadply.cpp ${CS6555_HEADERS} ${CS6555_SOURCES} )
#target_link_libraries( loadply ${CS6555_LIBRARIES} )
#add_dependencies( loadply cs6555 )

#add_executable( vbo ${CMAKE_CURRENT_SOURCE_DIR}/vbo.cpp ${CS6555_HEADERS} ${CS6555_SOURCES} )
#target_link_libraries( vbo ${CS6555_LIBRARIES} )
#add_dependencies( vbo cs6555 )

#add_executable( boids ${CMAKE_CURRENT_SOURCE_DIR}/boids.cpp ${CS6555_HEADERS} ${CS6555_SOURCES} )
#target_link_libraries( boids ${CS6555_LIBRARIES} )
#add_dependencies( boids cs6555 )

add_executable( boids_simple ${CMAKE_CURRENT_SOURCE_DIR}/boids_simple.cpp ${CS6555_HEADERS} ${CS6555_SOURCES} )
target_link_libraries( boids_simple ${CS6555_LIBRARIES} )
add_dependencies( boids_simple cs6555 )

#lab5

#add_executable( pointcloud ${CMAKE_CURRENT_SOURCE_DIR}/pointcloud.cpp ${CS6555_HEADERS} ${CS6555_SOURCES} )
#target_link_libraries( pointcloud ${CS6555_LIBRARIES} )
#add_dependencies( pointcloud cs6555 )

#add_executable( springmasssystem ${CMAKE_CURRENT_SOURCE_DIR}/springmasssystem.cpp ${CS6555_HEADERS} ${CS6555_SOURCES} )
#target_link_libraries( springmasssystem ${CS6555_LIBRARIES} )
#add_dependencies( springmasssystem cs6555 )

#add_executable( deformablebody ${CMAKE_CURRENT_SOURCE_DIR}/deformablebody.cpp ${CS6555_HEADERS} ${CS6555_SOURCES} )
#target_link_libraries( deformablebody ${CS6555_LIBRARIES} )
#add_dependencies( deformablebody cs6555 )

#add_executable( deformablespace ${CMAKE_CURRENT_SOURCE_DIR}/deformablespace.cpp ${CS6555_HEADERS} ${CS6555_SOURCES} )
#target_link_libraries( deformablespace ${CS6555_LIBRARIES} )
#add_dependencies( deformablespace cs6555 )

#add_executable( patch ${CMAKE_CURRENT_SOURCE_DIR}/patch.cpp ${CS6555_HEADERS} ${CS6555_SOURCES} )
#target_link_libraries( patch ${CS6555_LIBRARIES} )
#add_dependencies( patch cs6555 )

add_executable( particle ${CMAKE_CURRENT_SOURCE_DIR}/particle.cpp ${CS6555_HEADERS} ${CS6555_SOURCES} )
target_link_libraries( particle ${CS6555_LIBRARIES} )
add_dependencies( particle cs6555 )

add_executable( explosion ${CMAKE_CURRENT_SOURCE_DIR}/explosion.cpp ${CS6555_HEADERS} ${CS6555_SOURCES} )
target_link_libraries( explosion ${CS6555_LIBRARIES} )
add_dependencies( explosion cs6555 )
