# components/bc7215_ac_lib/CMakeLists.txt

set(EXT_LIB_PATH "./../../../../bc7215_ac_lib")

idf_component_register(
    SRCS "${EXT_LIB_PATH}/bc7215_lib.c" 
         "${EXT_LIB_PATH}/bc7215_ac_lib.c"
    INCLUDE_DIRS "${EXT_LIB_PATH}" 
                 "./../../main/include" # <-- where bc7215_lib_config.h located
)
