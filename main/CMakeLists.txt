if (NOT FREERTOS_KERNEL_PATH AND NOT DEFINED ENV{FREERTOS_KERNEL_PATH})
    message("Skipping FreeRTOS examples as FREERTOS_KERNEL_PATH not defined")
    return()
endif()

include(FreeRTOS_Kernel_import.cmake)

# === (1) Adiciona a pasta Fusion que está no RAIZ ===
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../Fusion ${CMAKE_BINARY_DIR}/fusion_build)

# === (2) Executável principal ===
add_executable(pico_emb
    main.c
)

# Includes do projeto (main/)
target_include_directories(pico_emb PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

# === (3) Linkagem ===
target_link_libraries(pico_emb PRIVATE
    pico_async_context_freertos
    FreeRTOS-Kernel-Heap4
    hardware_i2c
    Fusion                 # <- agora existe, veio do add_subdirectory acima
    pico_stdlib
)

target_compile_definitions(pico_emb PRIVATE
    configNUMBER_OF_CORES=1
)

if(PICO_CYW43_SUPPORTED)
    target_link_libraries(pico_emb PRIVATE pico_cyw43_arch_none)
endif()

pico_add_extra_outputs(pico_emb)
set_target_properties(pico_emb PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
