# - Try to find Atlante
# Once done this will define
#  ATLANTE_FOUND - System has Atlante
#  ATLANTE_INCLUDE_DIRS - The Atlante include directories
#  ATLANTE_LIBRARIES - The libraries needed to use Atlante
#  ATLANTE_DEFINITIONS - Compiler switches required for using Atlante

find_package(PkgConfig)
pkg_check_modules(PC_ATLANTE QUIET atlante)
set(ATLANTE_DEFINITIONS ${PC_ATLANTE_CFLAGS_OTHER})

find_path(ATLANTE_INCLUDE_DIR atlante
          HINTS ${PC_ATLANTE_INCLUDEDIR} ${PC_ATLANTE_INCLUDE_DIRS} )

find_library(ATLANTE_LIBRARY NAMES atlante
             HINTS ${PC_ATLANTE_LIBDIR} ${PC_ATLANTE_LIBRARY_DIRS} )

set(ATLANTE_INCLUDE_DIR ${ATLANTE_INCLUDE_DIR}/atlante)

set(ATLANTE_LIBRARIES ${ATLANTE_LIBRARY} )
set(ATLANTE_INCLUDE_DIRS ${ATLANTE_INCLUDE_DIR} )

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set ATLANTE_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(Atlante  DEFAULT_MSG
                                  ATLANTE_LIBRARY ATLANTE_INCLUDE_DIR)

mark_as_advanced(ATLANTE_INCLUDE_DIR ATLANTE_LIBRARY )
