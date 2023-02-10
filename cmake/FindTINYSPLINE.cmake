# ------------------------------------------------------------------------------------------------ #
#                                This file is part of CosmoScout VR                                #
# ------------------------------------------------------------------------------------------------ #

# SPDX-FileCopyrightText: German Aerospace Center (DLR) <cosmoscout@dlr.de>
# SPDX-License-Identifier: MIT

# Locate header.
find_path(TINYSPLINE_INCLUDE_DIR tinysplinecxx.h
    HINTS ${TINYSPLINE_ROOT_DIR}/include)

# Locate library.
find_library(TINYSPLINE_LIBRARY NAMES tinysplinecxx
    HINTS ${TINYSPLINE_ROOT_DIR}/lib ${TINYSPLINE_ROOT_DIR}/lib64)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(TINYSPLINE DEFAULT_MSG TINYSPLINE_INCLUDE_DIR TINYSPLINE_LIBRARY)

# Add imported target.
if(TINYSPLINE_FOUND)
    set(TINYSPLINE_INCLUDE_DIRS "${TINYSPLINE_INCLUDE_DIR}")

    if(NOT TINYSPLINE_FIND_QUIETLY)
        message(STATUS "TINYSPLINE_INCLUDE_DIRS ....... ${TINYSPLINE_INCLUDE_DIR}")
        message(STATUS "TINYSPLINE_LIBRARY ............ ${TINYSPLINE_LIBRARY}")
    endif()

    if(NOT TARGET tinyspline::tinyspline)
        add_library(tinyspline::tinyspline UNKNOWN IMPORTED)
        set_target_properties(tinyspline::tinyspline PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${TINYSPLINE_INCLUDE_DIRS}")

        set_property(TARGET tinyspline::tinyspline APPEND PROPERTY
            IMPORTED_LOCATION "${TINYSPLINE_LIBRARY}")
    endif()
endif()
