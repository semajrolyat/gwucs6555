# For Debug Build :: -DCMAKE_BUILD_TYPE=Debug

project( cs6555 )
cmake_minimum_required(VERSION 2.8)

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules ${CMAKE_MODULE_PATH} )

find_package( GLUT REQUIRED )
find_package( OpenGL REQUIRED )
find_package( TIFF REQUIRED )
find_package( Eigen3 REQUIRED )
#find_package( Moby REQUIRED )
#find_package( RT REQUIRED )
#find_package( DL REQUIRED )
#find_package( Pthread REQUIRED )

set( CS6555_HEADERS 
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Math/CubicSpline.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Math/EulerAngle.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Math/LineSegment.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Math/Matrix3.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Math/Matrix4.h
    #${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Math/MatrixN.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Math/Quaternion.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Math/Vector3.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Math/Vector4.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/AABB.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Actor.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/ArticulatedBody.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Body.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Boid.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/BoundingSphere.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/BoundingVolume.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/BVH.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Camera.h
    #${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/CloudMaker.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Color.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Constants.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/ContactEvent.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/ControlPoint.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/DeformableBody.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/DeformableGeometry.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Edge.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Event.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Flock.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Geometry.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/GeometryMaker.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/GetPot
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Interpolation.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Joint.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Keyframe.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Link.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Material.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Math.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Mesh.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/MeshLoader.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Motivator.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Movie.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Node.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/OBB.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Particle.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/ParticleCloud.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/ParticleEmitter.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Patch.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/PerlinNoise.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Physics.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Point3.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/PointCloud.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/PointCube.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Point.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Polygon.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Pose.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Renderer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/RigidBody.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Scene.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Sensor.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Spring.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/SpringMassSystem.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Tetrahedron.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/tiff.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/tiffio.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/tiffvers.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Trajectory.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Trigon.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Utilities.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Vertex.h
    #${CMAKE_CURRENT_SOURCE_DIR}/include/cs6555/Viewport.h
)

set( CS6555_SOURCES 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Math/CubicSpline.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Math/EulerAngle.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Math/Matrix3.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Math/Matrix4.cpp
    #${CMAKE_CURRENT_SOURCE_DIR}/src/Math/MatrixN.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Math/Quaternion.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Math/Vector3.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Math/Vector4.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/AABB.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Actor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ArticulatedBody.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Boid.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/BoundingSphere.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/BoundingVolume.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/BVH.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Camera.cpp
    #${CMAKE_CURRENT_SOURCE_DIR}/src/CloudMaker.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/DeformableBody.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Flock.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Geometry.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/GeometryMaker.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Joint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Keyframe.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Link.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Mesh.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/MeshLoader.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/OBB.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Particle.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Patch.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Physics.cpp
    #${CMAKE_CURRENT_SOURCE_DIR}/src/PointCloud.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Polygon.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Pose.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Renderer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/RigidBody.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Scene.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Trajectory.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Trigon.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Vertex.cpp
    #${CMAKE_CURRENT_SOURCE_DIR}/src/Viewport.cpp
)

set( CS6555_INCLUDE_DIRS 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    #${MOBY_INCLUDE_DIR}
)

set( CS6555_LIBRARIES
    ${OPENGL_LIBRARIES}
    ${GLUT_LIBRARIES}
    ${TIFF_LIBRARIES}
    #${RT_LIBRARIES}
    #${DL_LIBRARIES}
    #${PTHREAD_LIBRARIES}
    #${MOBY_LIBRARIES}
)

include_directories( ${CS6555_INCLUDE_DIRS} )

add_subdirectory( testing )

add_subdirectory( examples )

