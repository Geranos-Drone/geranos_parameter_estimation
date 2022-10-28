

#########################----------------------------
cmake_minimum_required(VERSION 2.8.3)
project(geranos_parameter_estimation)
find_package(catkin_simple REQUIRED dynamic_reconfigure)



#find_package(doxygen_catkin REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
#add_doxygen()
add_definitions(-std=c++11 -fPIC -Werror -Wall -Wno-reorder -Wno-unknown-pragmas)


# add_message_files(
#   FILES
#   geranos_parameter_estimation.msg
#)
# generate_messages()

#############
# LIBRARIES #
#############

cs_add_library(geranos_parameter_estimation_lib
  src/geranos_parameter_estimator.cpp
)


cs_add_executable(mass_and_inertia_estimator_node
  src/geranos_parameter_estimator.cpp
)

cs_add_executable(mass_inertia_com_estimator_node
  src/geranos_full_parameter_estimator.cpp
)

cs_add_executable(mass_comz_inertia_estimator_node
  src/geranos_parameter_estimation_observable.cpp
)

cs_add_executable(mass_comz_estimator_node
  src/geranos_parameter_estimation_no_inertia.cpp
)

cs_add_executable(geranos_mass_jxy_comz_estimator_node
  src/geranos_mass_jxy_comz_estimator.cpp
)

cs_add_executable(test_client_node
  src/test_client.cpp
)

cs_add_executable(ground_truth_node
  src/ground_truth.cpp
)





include_directories(SYSTEM "${LIB_DIR}/Include")


#dynamic reconfigure:
#generate_dynamic_reconfigure_options(
#  cfg/EKF_Tuning.cfg
  #...
#)
#add_dependencies(mass_and_inertia_estimator_node ${geranos_parameter_estimation}_gencfg)

##########
# EXPORT #
##########
cs_install()
cs_export()
