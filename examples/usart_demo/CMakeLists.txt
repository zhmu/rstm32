project(usart_demo)

add_executable(usart_demo.elf usart_demo.cpp)
target_link_libraries(usart_demo.elf rstm32)
