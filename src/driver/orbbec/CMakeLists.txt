add_library(OrbbecSDK SHARED IMPORTED GLOBAL)

set_target_properties(OrbbecSDK PROPERTIES
    IMPORTED_LOCATION
        "${CMAKE_SOURCE_DIR}/src/driver/orbbec/lib/libOrbbecSDK.so"
    INTERFACE_INCLUDE_DIRECTORIES
        "${CMAKE_SOURCE_DIR}/src/driver/orbbec/include"
)