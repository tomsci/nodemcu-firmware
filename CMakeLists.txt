cmake_minimum_required(VERSION 3.5)

if(DEFINED ENV{EXTRA_COMPONENT_DIRS})
  # Turn space-separated, quote-aware environment var into CMake list
  separate_arguments(
    EXTRA_COMPONENT_DIRS UNIX_COMMAND "$ENV{EXTRA_COMPONENT_DIRS}")
endif()

function(extmod_declare_module confname)
  if (${CONFIG_NODEMCU_CMODULE_${confname}})
    # If the module is enabled in the menuconfig, add a
    # "-u <modname>_module_selected1" option to force the linker to include
    # the module. See components/core/include/module.h for further details on
    # how this works.
    message("Including external module ${confname}")
    target_link_libraries(${COMPONENT_LIB} "-u ${confname}_module_selected1")
  endif()
endfunction()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(nodemcu)

# Useful when tidying up code to catch all the warnings.
#idf_build_set_property(COMPILE_OPTIONS "-Werror" "-Werror=deprecated-declarations" APPEND)
