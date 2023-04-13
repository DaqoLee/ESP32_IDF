file(GLOB_RECURSE SOURCES ./src/*.c)

# set(component_srcs "src/rotary_encoder_pcnt_ec11.c")

idf_component_register(

SRCS ${SOURCES} 
INCLUDE_DIRS "include".

REQUIRES
u8g2

)
