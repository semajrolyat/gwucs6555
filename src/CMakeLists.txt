set( CS6555Animator_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/Math/Vector3.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Math/Vector4.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Math/Matrix3.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Math/Matrix4.cpp
    #${CMAKE_CURRENT_SOURCE_DIR}/Math/MatrixN.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Math/Quaternion.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Math/CubicSpline.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Trajectory.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Math/EulerAngle.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Renderer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Vertex.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Polygon.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Geometry.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/GeometryMaker.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Mesh.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MeshLoader.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Actor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Pose.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Scene.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Keyframe.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Camera.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Joint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Link.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ArticulatedBody.cpp
    #lab 3
    ${CMAKE_CURRENT_SOURCE_DIR}/BoundingVolume.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/BoundingSphere.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/AABB.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/OBB.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/BVH.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Physics.cpp
    #${CMAKE_CURRENT_SOURCE_DIR}/Body.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/RigidBody.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/DeformableBody.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Patch.cpp
    #lab 4
    ${CMAKE_CURRENT_SOURCE_DIR}/Boid.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Flock.cpp
    #lab 5
    #${CMAKE_CURRENT_SOURCE_DIR}/PointCloud.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/CloudMaker.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Trigon.cpp
    PARENT_SCOPE
)

