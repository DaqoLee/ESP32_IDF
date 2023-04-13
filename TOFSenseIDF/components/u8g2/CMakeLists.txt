file(GLOB_RECURSE SOURCES u8g2/src/*.c)

idf_component_register(
  SRCS ${SOURCES}
  INCLUDE_DIRS "include"
  
  REQUIRES
  device
  )
