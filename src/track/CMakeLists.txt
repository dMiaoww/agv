aux_source_directory(. src)
aux_source_directory(controller CONTROL_SRC)

add_library(track ${src} ${CONTROL_SRC})