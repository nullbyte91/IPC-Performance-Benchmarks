find_path(OMNIORB4_INCLUDE_DIR
    NAMES omniORB4/CORBA.h
    PATHS /usr/include /usr/local/include
)

find_library(OMNIORB4_LIBRARY
    NAMES omniORB4
    PATHS /usr/lib /usr/local/lib
)

find_library(OMNITHREAD_LIBRARY
    NAMES omnithread
    PATHS /usr/lib /usr/local/lib
)

find_library(OMNIDYNAMIC4_LIBRARY
    NAMES omniDynamic4
    PATHS /usr/lib /usr/local/lib
)

set(OMNIORB4_INCLUDE_DIRS ${OMNIORB4_INCLUDE_DIR})
set(OMNIORB4_LIBRARIES ${OMNIORB4_LIBRARY} ${OMNITHREAD_LIBRARY} ${OMNIDYNAMIC4_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(omniORB4 DEFAULT_MSG OMNIORB4_INCLUDE_DIRS OMNIORB4_LIBRARIES)

mark_as_advanced(OMNIORB4_INCLUDE_DIRS OMNIORB4_LIBRARIES)
