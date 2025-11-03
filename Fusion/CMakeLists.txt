# Fusion/CMakeLists.txt

add_library(Fusion STATIC
    ${CMAKE_CURRENT_LIST_DIR}/FusionAhrs.c
    ${CMAKE_CURRENT_LIST_DIR}/FusionCompass.c
    # ${CMAKE_CURRENT_LIST_DIR}/FusionMath.c
    ${CMAKE_CURRENT_LIST_DIR}/FusionOffset.c
    # Se existir outro .c na sua pasta, adicione aqui
)

target_include_directories(Fusion PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}   # exporta o include para achar "Fusion.h"
)
