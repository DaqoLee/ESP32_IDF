file(GLOB_RECURSE SOURCES ./*.c)
idf_component_register(SRCS ${SOURCES}
        #.h文件所在路径
        INCLUDE_DIRS
        .
        #components,可以是IDF里的也可以是自己的
        REQUIRES
        lvgl
        lvgl_esp32_drivers
        device
        )