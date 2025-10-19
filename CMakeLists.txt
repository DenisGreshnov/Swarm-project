cmake_minimum_required(VERSION 3.10)
project(FlockingSimulation)

set(CMAKE_CXX_STANDARD 17)

# Простая настройка для Windows с GLFW
set(GLFW_PATH ${CMAKE_SOURCE_DIR}/libs/glfw)

# Добавляем исполняемый файл
add_executable(flocking_simulation
    src/main.cpp
    src/simulation.cpp
    src/renderer.cpp
)

# Подключаем заголовки GLFW
target_include_directories(flocking_simulation PRIVATE ${GLFW_PATH}/include)

# Подключаем библиотеки вручную
if(WIN32)
    target_link_libraries(flocking_simulation
        ${GLFW_PATH}/lib-mingw-w64/libglfw3.a
        opengl32
        gdi32
    )
else()
    # Для Linux
    target_link_libraries(flocking_simulation
        glfw
        GL
    )
endif()