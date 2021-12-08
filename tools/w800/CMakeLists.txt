project(wm_tools)
add_executable(wm_tool wm_tool.c)
target_link_libraries(wm_tool PRIVATE pthread)