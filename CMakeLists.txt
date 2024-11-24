# CMake 최소 요구 버전
cmake_minimum_required(VERSION 3.30.0)

# 프로젝트 이름과 언어 설정
set(CMAKE_C_COMPILER "C:/MinGW/bin/gcc.exe")
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY) ## This configuration will enable CMake to properly handle the test compilation check during cross-compiling.s
project(my_eskf_project LANGUAGES C VERSION 0.1.0) 



# 소스 파일 자동 검색
file(GLOB SOURCES "${CMAKE_SOURCE_DIR}/src/*.c")

# 실행 파일 생성
add_executable(MyExecutable ${SOURCES})


# 헤더 파일 경로 추가
include_directories(${CMAKE_SOURCE_DIR}/include)
#include_directories("C:/MinGW/include")

# 라이브러리 추가
target_link_libraries(MyExecutable m)
