project( cs6555_ex )
cmake_minimum_required(VERSION 2.8)

#find_package( cs6555 REQUIRED )

include_directories( ${CS6555_INCLUDE_DIRS} )

add_executable( template ${CMAKE_CURRENT_SOURCE_DIR}/template.cpp ${CS6555_HEADERS} ${CS6555_SOURCES} )
target_link_libraries( template ${CS6555_LIBRARIES} )
add_dependencies( template cs6555 )

add_executable( lab0 ${CMAKE_CURRENT_SOURCE_DIR}/lab0.cpp ${CS6555_HEADERS} ${CS6555_SOURCES} )
target_link_libraries( lab0 ${CS6555_LIBRARIES} )
add_dependencies( lab0 cs6555 )

add_executable( lab1 ${CMAKE_CURRENT_SOURCE_DIR}/lab1.cpp ${CS6555_HEADERS} ${CS6555_SOURCES} )
target_link_libraries( lab1 ${CS6555_LIBRARIES} )
add_dependencies( lab1 cs6555 )

add_executable( lab2 ${CMAKE_CURRENT_SOURCE_DIR}/lab2.cpp ${CS6555_HEADERS} ${CS6555_SOURCES} )
target_link_libraries( lab2 ${CS6555_LIBRARIES} )
add_dependencies( lab2 cs6555 )

add_executable( lab3 ${CMAKE_CURRENT_SOURCE_DIR}/lab3.cpp ${CS6555_HEADERS} ${CS6555_SOURCES} )
target_link_libraries( lab3 ${CS6555_LIBRARIES} )
add_dependencies( lab3 cs6555 )

add_executable( lab4 ${CMAKE_CURRENT_SOURCE_DIR}/lab4.cpp ${CS6555_HEADERS} ${CS6555_SOURCES} )
target_link_libraries( lab4 ${CS6555_LIBRARIES} )
add_dependencies( lab4 cs6555 )

add_executable( lab5 ${CMAKE_CURRENT_SOURCE_DIR}/lab5.cpp ${CS6555_HEADERS} ${CS6555_SOURCES} )
target_link_libraries( lab5 ${CS6555_LIBRARIES} )
add_dependencies( lab5 cs6555 

configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/biplane.d biplane.d COPYONLY )
#configure_file( data/biplane.d UnitTest/biplane.d COPYONLY )

#configure_file( mesh_start_ascii.ply mesh_start_ascii.ply COPYONLY )
#configure_file( data/mesh_start_ascii.ply UnitTest/mesh_start_ascii.ply COPYONLY )

#configure_file( mesh_end_ascii.ply mesh_end_ascii.ply COPYONLY )
#configure_file( data/mesh_end_ascii.ply UnitTest/mesh_end_ascii.ply COPYONLY )

)
