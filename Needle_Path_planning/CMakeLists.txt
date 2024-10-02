cmake_minimum_required(VERSION 3.16.3...3.19.7 FATAL_ERROR)

project(Needle_Path_Planning)

#-----------------------------------------------------------------------------
# Extension meta-information
set(EXTENSION_HOMEPAGE "https://www.slicer.org/wiki/Documentation/Nightly/Extensions/Needle_Path_Planning")
set(EXTENSION_CATEGORY "Neurosurgery Path Planning")
set(EXTENSION_CONTRIBUTORS "Jiaheng Wang KCL")
set(EXTENSION_DESCRIPTION "This extension helps to find an optimal needle insertion path for brain surgery, considering cortex angles and avoiding blood vessels.")
set(EXTENSION_ICONURL "https://www.example.com/Slicer/Extensions/Needle_Path_Planning.png")
set(EXTENSION_SCREENSHOTURLS "https://www.example.com/Slicer/Extensions/Needle_Path_Planning/Screenshots/1.png")
set(EXTENSION_DEPENDS "NA") # Specified as a list or "NA" if no dependencies

#-----------------------------------------------------------------------------
# Extension dependencies
find_package(Slicer REQUIRED)
include(${Slicer_USE_FILE})

#-----------------------------------------------------------------------------
# Extension modules
add_subdirectory(Needle_Path_Planning)
## NEXT_MODULE

#-----------------------------------------------------------------------------
include(${Slicer_EXTENSION_GENERATE_CONFIG})
include(${Slicer_EXTENSION_CPACK})
