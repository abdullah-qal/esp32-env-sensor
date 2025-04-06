file(GLOB_RECURSE srcs "app_main.c" "src/*.c")

idf_component_register(
    SRCS "${srcs}"
    INCLUDE_DIRS
    "./include" 
    REQUIRES driver
    PRIV_REQUIRES bt nvs_flash
)
