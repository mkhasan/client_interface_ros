cmake_minimum_required(VERSION 2.8.3)
project(client_interface)


add_compile_options(-std=c++14)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -ggdb -Wall -D_GNU_SOURCE=1 -D_REENTRANT")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb -Wall -D_GNU_SOURCE=1 -D_REENTRANT")


find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	xsens_msgs
	my_location
	
	
	  
)


include_directories(
  include
  /usr/local/include/ImageMagick-7
  ${catkin_INCLUDE_DIRS}
)





 
find_library(MQTT mosquittopp HINTS "/usr/local/lib")
find_library(ACE ACE)
find_library(MYSQL_CONN mysqlcppconn)
find_library(MAGIC_HDRI Magick++-7.Q16HDRI HINTS "/usr/local/lib")


message("test is ${MYSQL_CONN}")

add_library(${PROJECT_NAME}
  src/client_interface.cpp src/shm_manager.cpp src/db_handler.cpp src/utils.cpp
)

add_library(speedo src/speedo.cpp src/parser.cpp src/client_handler.cpp src/client_acceptor.cpp src/queue.cpp src/dr_robot.cpp)
target_link_libraries(speedo ${MQTT} ${ACE})


catkin_package(
   INCLUDE_DIRS include 
   LIBRARIES client_interface
   CATKIN_DEPENDS roscpp rospy std_msgs xsens_msgs my_location
)
 
#message("*********************************** ${CATKIN_PACKAGE_BIN_DESTINATION} *************************************")
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(client_interface ${catkin_LIBRARIES} ${MQTT} ${ACE} ${MYSQL_CONN} ${MAGIC_HDRI})

add_executable(client_interface_node src/client_interface_node.cpp)
 
add_dependencies(client_interface_node ${client_interface_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(client_interface_node ${PROJECT_NAME} ${catkin_LIBRARIES})


add_dependencies(speedo ${catkin_EXPORTED_TARGETS})

add_executable(speedo_node src/speedo_node.cpp)
 
add_dependencies(speedo_node ${speedo_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(speedo_node speedo ${catkin_LIBRARIES})

