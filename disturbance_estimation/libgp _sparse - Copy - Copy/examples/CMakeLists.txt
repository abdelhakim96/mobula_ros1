# libgp - Gaussian process library for Machine Learning
# Copyright (c) 2013, Manuel Blum <mblum@informatik.uni-freiburg.de>
# All rights reserved.

ADD_EXECUTABLE(gpdense gp_example_dense.cc)
TARGET_LINK_LIBRARIES(gpdense gp)

ADD_EXECUTABLE(gp_example1 gp_example1.cc test_functions.cpp)
TARGET_LINK_LIBRARIES(gp_example1 gp)

ADD_EXECUTABLE(gp_example2 gp_example2.cc test_functions.cpp)
TARGET_LINK_LIBRARIES(gp_example2 gp)

ADD_EXECUTABLE(gp_example_mass_change gp_example_mass_change.cc)
TARGET_LINK_LIBRARIES(gp_example_mass_change gp)

ADD_EXECUTABLE(gp_example_mass_change_history gp_example_mass_change_history.cc)
TARGET_LINK_LIBRARIES(gp_example_mass_change_history gp)
