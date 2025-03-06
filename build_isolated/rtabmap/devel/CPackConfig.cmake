# This file will be configured to contain variables for CPack. These variables
# should be set in the CMake list file of the project before CPack module is
# included. The list of available CPACK_xxx variables and their associated
# documentation may be obtained using
#  cpack --help-variable-list
#
# Some variables are common to all generators (e.g. CPACK_PACKAGE_NAME)
# and some are specific to a generator
# (e.g. CPACK_NSIS_EXTRA_INSTALL_COMMANDS). The generator specific variables
# usually begin with CPACK_<GENNAME>_xxxx.


set(CPACK_BINARY_7Z "")
set(CPACK_BINARY_BUNDLE "")
set(CPACK_BINARY_CYGWIN "")
set(CPACK_BINARY_DEB "")
set(CPACK_BINARY_DRAGNDROP "")
set(CPACK_BINARY_FREEBSD "")
set(CPACK_BINARY_IFW "")
set(CPACK_BINARY_NSIS "")
set(CPACK_BINARY_NUGET "")
set(CPACK_BINARY_OSXX11 "")
set(CPACK_BINARY_PACKAGEMAKER "")
set(CPACK_BINARY_PRODUCTBUILD "")
set(CPACK_BINARY_RPM "")
set(CPACK_BINARY_STGZ "")
set(CPACK_BINARY_TBZ2 "")
set(CPACK_BINARY_TGZ "")
set(CPACK_BINARY_TXZ "")
set(CPACK_BINARY_TZ "")
set(CPACK_BINARY_WIX "")
set(CPACK_BINARY_ZIP "")
set(CPACK_BUILD_SOURCE_DIRS "/home/robond/ROS_Udacity/src/rtabmap;/home/robond/ROS_Udacity/build_isolated/rtabmap/devel")
set(CPACK_CMAKE_GENERATOR "Unix Makefiles")
set(CPACK_COMPONENTS_ALL "Unspecified;devel;runtime")
set(CPACK_COMPONENT_UNSPECIFIED_HIDDEN "TRUE")
set(CPACK_COMPONENT_UNSPECIFIED_REQUIRED "TRUE")
set(CPACK_GENERATOR "TBZ2")
set(CPACK_INSTALL_CMAKE_PROJECTS "/home/robond/ROS_Udacity/build_isolated/rtabmap/devel;RTABMap;ALL;/")
set(CPACK_INSTALL_PREFIX "/home/robond/ROS_Udacity/devel_isolated/rtabmap")
set(CPACK_MODULE_PATH "/home/robond/ROS_Udacity/src/rtabmap/cmake_modules;/usr/lib/cmake/vtk-6.2")
set(CPACK_NSIS_DISPLAY_NAME "RTABMap 0.20.7")
set(CPACK_NSIS_INSTALLER_ICON_CODE "")
set(CPACK_NSIS_INSTALLER_MUI_ICON_CODE "")
set(CPACK_NSIS_INSTALL_ROOT "$PROGRAMFILES")
set(CPACK_NSIS_PACKAGE_NAME "RTABMap 0.20.7")
set(CPACK_OUTPUT_CONFIG_FILE "/home/robond/ROS_Udacity/build_isolated/rtabmap/devel/CPackConfig.cmake")
set(CPACK_PACKAGE_CONTACT "matlabbe@gmail.com")
set(CPACK_PACKAGE_DEFAULT_LOCATION "/")
set(CPACK_PACKAGE_DESCRIPTION_FILE "/usr/local/share/cmake-3.14/Templates/CPack.GenericDescription.txt")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "RTAB-MAP is a Real-Time Appearance-Based Mapping approach.")
set(CPACK_PACKAGE_FILE_NAME "RTABMap-0.20.7-Linux")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "RTABMap 0.20.7")
set(CPACK_PACKAGE_INSTALL_REGISTRY_KEY "RTABMap 0.20.7")
set(CPACK_PACKAGE_NAME "RTABMap")
set(CPACK_PACKAGE_RELOCATABLE "true")
set(CPACK_PACKAGE_VENDOR "RTABMap project")
set(CPACK_PACKAGE_VERSION "0.20.7")
set(CPACK_PACKAGE_VERSION_MAJOR "0")
set(CPACK_PACKAGE_VERSION_MINOR "20")
set(CPACK_PACKAGE_VERSION_PATCH "7")
set(CPACK_RESOURCE_FILE_LICENSE "/home/robond/ROS_Udacity/src/rtabmap/LICENSE")
set(CPACK_RESOURCE_FILE_README "/usr/local/share/cmake-3.14/Templates/CPack.GenericDescription.txt")
set(CPACK_RESOURCE_FILE_WELCOME "/usr/local/share/cmake-3.14/Templates/CPack.GenericWelcome.txt")
set(CPACK_SET_DESTDIR "OFF")
set(CPACK_SOURCE_7Z "")
set(CPACK_SOURCE_CYGWIN "")
set(CPACK_SOURCE_GENERATOR "ZIP")
set(CPACK_SOURCE_IGNORE_FILES "\\.svn/;\\.settings/;/home/robond/ROS_Udacity/src/rtabmap/build/[a-zA-Z0-9_]+;~$;/home/robond/ROS_Udacity/src/rtabmap/bin/.*rtabmap;/home/robond/ROS_Udacity/src/rtabmap/bin/.*RTABMap;/home/robond/ROS_Udacity/src/rtabmap/bin/.*[tT]est;/home/robond/ROS_Udacity/src/rtabmap/bin/.*[eE]xample;/home/robond/ROS_Udacity/src/rtabmap/bin/.*uresourcegenerator;\\.DS_Store")
set(CPACK_SOURCE_OUTPUT_CONFIG_FILE "/home/robond/ROS_Udacity/build_isolated/rtabmap/devel/CPackSourceConfig.cmake")
set(CPACK_SOURCE_RPM "")
set(CPACK_SOURCE_TBZ2 "")
set(CPACK_SOURCE_TGZ "")
set(CPACK_SOURCE_TXZ "")
set(CPACK_SOURCE_TZ "")
set(CPACK_SOURCE_ZIP "")
set(CPACK_SYSTEM_NAME "Linux")
set(CPACK_TOPLEVEL_TAG "Linux")
set(CPACK_WIX_SIZEOF_VOID_P "8")

if(NOT CPACK_PROPERTIES_FILE)
  set(CPACK_PROPERTIES_FILE "/home/robond/ROS_Udacity/build_isolated/rtabmap/devel/CPackProperties.cmake")
endif()

if(EXISTS ${CPACK_PROPERTIES_FILE})
  include(${CPACK_PROPERTIES_FILE})
endif()
