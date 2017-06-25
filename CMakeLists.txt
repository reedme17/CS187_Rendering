cmake_minimum_required (VERSION 2.8.3)
project(nori)

add_subdirectory(ext ext_build)

if (WIN32)
  SET (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /SAFESEH:NO")
  SET (CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /SAFESEH:NO")
  SET (CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} /SAFESEH:NO")
endif()

include_directories(
  # Nori include files
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  # tinyformat string formatting library
  ${TFM_INCLUDE_DIR}
  # Eigen linear algebra library
  ${EIGEN_INCLUDE_DIR}
  # OpenEXR high dynamic range bitmap library
  ${OPENEXR_INCLUDE_DIRS}
  # Intel Thread Building Blocks
  ${TBB_INCLUDE_DIR}
  # Pseudorandom number generator
  ${PCG32_INCLUDE_DIR}
  # PugiXML parser
  ${PUGIXML_INCLUDE_DIR}
  # Helper functions for statistical hypothesis tests
  ${HYPOTHESIS_INCLUDE_DIR}
  # GLFW library for OpenGL context creation
  ${GLFW_INCLUDE_DIR}
  # GLEW library for accessing OpenGL functions
  ${GLEW_INCLUDE_DIR}
  # NanoVG drawing library
  ${NANOVG_INCLUDE_DIR}
  # NanoGUI user interface library
  ${NANOGUI_INCLUDE_DIR}
  # Portable filesystem API
  ${FILESYSTEM_INCLUDE_DIR}
  ext
)

# The following lines build the main executable. If you add a source
# code file to Nori, be sure to include it in this list.
add_executable(nori

  # Header files
  include/nori/bbox.h
  include/nori/bitmap.h
  include/nori/block.h
  include/nori/bsdf.h
  include/nori/bvh.h
  include/nori/camera.h
  include/nori/color.h
  include/nori/common.h
  include/nori/dpdf.h
  include/nori/frame.h
  include/nori/integrator.h
  include/nori/emitter.h
  include/nori/mesh.h
  include/nori/object.h
  include/nori/parser.h
  include/nori/proplist.h
  include/nori/ray.h
  include/nori/rfilter.h
  include/nori/sampler.h
  include/nori/scene.h
  include/nori/timer.h
  include/nori/transform.h
  include/nori/vector.h
  include/nori/warp.h
  include/nori/texture.h
  include/nori/medium.h

  # Source code files
  src/bitmap.cpp
  src/block.cpp
  src/bvh.cpp
  src/common.cpp
  src/diffuse.cpp
  src/gui.cpp
  src/independent.cpp
  src/main.cpp
  src/mesh.cpp
  src/obj.cpp
  src/moveObj.cpp
  src/object.cpp
  src/parser.cpp
  src/perspective.cpp
  src/proplist.cpp
  src/render.cpp
  src/rfilter.cpp
  src/scene.cpp
  src/warp.cpp
  src/chi2test.cpp
  src/ttest.cpp
  
  # integrator
  src/normals.cpp
  src/ao.cpp
  src/direct.cpp
  src/direct_ems.cpp
  src/direct_mats.cpp
  src/direct_mis.cpp
  src/path_mis.cpp
  src/path_nee.cpp
  src/path_mats.cpp
  src/path_volumetric.cpp
  src/path_volumetric2.cpp

  # emitter
  src/point.cpp
  src/area.cpp
  src/distantsphere.cpp
  src/environmental_light.cpp

  # bsdf
  src/blinn.cpp
  src/phong.cpp
  src/microfacet.cpp
  src/dielectric.cpp
  src/mirror.cpp

  # special bsdf (for final)
  src/micro_dielectric.cpp
  src/anisotropic.cpp # did not work out 
  src/paper.cpp
  src/diffuse_texture.cpp
  src/specular_texture.cpp
  src/fresnel_texture.cpp
  src/blinn_texture.cpp

  # texture
  src/image_texture.cpp

  # medium
  src/homogeneous.cpp
)


# The following lines build the warping test application
add_executable(warptest
  include/nori/warp.h
  src/warp.cpp
  src/warptest.cpp
  src/object.cpp
  src/proplist.cpp
  src/common.cpp
  src/blinn.cpp
  src/phong.cpp
  src/microfacet.cpp
)

# The following lines build the tonemapper
add_executable(tonemapper
        include/nori/bitmap.h
        src/bitmap.cpp
        src/common.cpp
        src/hdrToLdr.cpp)

target_link_libraries(nori tbb_static pugixml IlmImf nanogui ${NANOGUI_EXTRA_LIBS})
target_link_libraries(warptest tbb_static nanogui ${NANOGUI_EXTRA_LIBS})
target_link_libraries(tonemapper IlmImf)

# vim: set et ts=2 sw=2 ft=cmake nospell:
