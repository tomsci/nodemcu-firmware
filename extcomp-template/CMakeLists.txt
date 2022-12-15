# If necessary, add items to the SRCS, INCLUDE_DIRS and/or PRIV_REQUIRES below
idf_component_register(
  SRCS "mymod.c"
  INCLUDE_DIRS "." "${CMAKE_CURRENT_BINARY_DIR}"
  PRIV_REQUIRES
    "base_nodemcu"
    "lua"
    "platform"
)

# The argument to this function must match the first argument in the
# NODEMCU_MODULE() macro in the .c file.
extmod_declare_module(MYMOD)
