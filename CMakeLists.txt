
cmake_minimum_required(VERSION 2.8)

project(MQIO)

IF (UNIX)
        set(CMAKE_C_COMPILER "/usr/bin/clang")
        set(CMAKE_CXX_COMPILER "/usr/bin/clang++")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++ -I/usr/include/libcxxabi")
        set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -stdlib=libc++ -lc++abi -rpath=./")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
ENDIF()

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

include_directories(${PROJECT_SOURCE_DIR}/include) 
include_directories(${PROJECT_SOURCE_DIR}/include/rocket) 
include_directories(${PROJECT_SOURCE_DIR}/include/simone)
include_directories(${PROJECT_SOURCE_DIR}/include/http)
include_directories(${PROJECT_SOURCE_DIR}/src)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

set(ROCKETMQ_LIBRARIES librocketmq.a)

set(Trans "Trans")
file(GLOB TransFiles "src/*.cpp")
add_executable(${Trans} ${TransFiles})

if(MSVC)
    if(CMAKE_CONFIGURATION_TYPES STREQUAL "Release")
        set_target_properties( ${Trans} PROPERTIES LINK_FLAGS "/NODEFAULTLIB:LIBCMT" )
    else()
        set_target_properties( ${Trans} PROPERTIES LINK_FLAGS "/NODEFAULTLIB:LIBCMTD" )
    endif()
endif()

if (MSVC)
    if (BUILD_ROCKETMQ_SHARED)
        target_link_libraries (${Trans}  rocketmq_shared ${deplibs}
        ${Boost_LIBRARIES} ${OPENSSL_LIBRARIES} ${LIBEVENT_LIBRARIES} ${JSONCPP_LIBRARIES})
    else()
        target_link_libraries (${Trans}  rocketmq_static ${deplibs}
        ${Boost_LIBRARIES} ${OPENSSL_LIBRARIES} ${LIBEVENT_LIBRARIES} ${JSONCPP_LIBRARIES})
    endif()
else()
        target_link_libraries (${Trans} ${ROCKETMQ_LIBRARIES} -ldl -lc++ -lm -lz -lpthread) # -lrt
        target_link_libraries (${Trans} ${CMAKE_CURRENT_LIST_DIR}/lib/simone/libSimOneAPI.so)
        target_link_libraries (${Trans} ${CMAKE_CURRENT_LIST_DIR}/lib/simone/libHDMapModule.so)
        target_link_libraries (${Trans} ${CMAKE_CURRENT_LIST_DIR}/lib/simone/libSSD.so)
        target_link_libraries (${Trans} ${CMAKE_CURRENT_LIST_DIR}/lib/simone/libtsf.so)
        target_link_libraries (${Trans} ${CMAKE_CURRENT_LIST_DIR}/lib/simone/libSimOneStreamingAPI.so)

        target_link_libraries(${Trans} ${CMAKE_CURRENT_LIST_DIR}/lib/boost/libboost_random.a)
        target_link_libraries(${Trans} ${CMAKE_CURRENT_LIST_DIR}/lib/boost/libboost_system.a)
        target_link_libraries(${Trans} ${CMAKE_CURRENT_LIST_DIR}/lib/boost/libboost_date_time.a)
        target_link_libraries(${Trans} ${CMAKE_CURRENT_LIST_DIR}/lib/boost/libboost_regex.a)

endif()
