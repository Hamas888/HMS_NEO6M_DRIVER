# HMS_NEO6M_DRIVER/CMakeLists.txt

set(HMS_NEO6M_DRIVER_VERSION 1.0.0)

# Check if we're building with Zephyr
if(DEFINED ZEPHYR_BASE)
    zephyr_include_directories(include)
    zephyr_library_sources(src/HMS_NEO6M_DRIVER.cpp)

# Check if we're building with ESP-IDF
elseif(IDF_PROJECT)
    idf_component_register(
        SRCS "src/HMS_NEO6M_DRIVER.cpp"
        INCLUDE_DIRS "include"
    )
    
# STM32 / generic CMake project
else()
    add_library(HMS_NEO6M_DRIVER INTERFACE)
    target_include_directories(HMS_NEO6M_DRIVER INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_compile_features(HMS_NEO6M_DRIVER INTERFACE cxx_std_17)
endif()