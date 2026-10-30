# ------------------------------------------------------------------------------
# |> FILTER ENGINE
# ------------------------------------------------------------------------------
# -> Applies global exclusion rules (folders/files) to any list.
macro(apply_global_filters TARGET_LIST)
    if(EXCLUDED_FOLDERS)
        string(REPLACE ";" "|" FOLDER_REGEX "${EXCLUDED_FOLDERS}")
        list(FILTER ${TARGET_LIST} EXCLUDE REGEX "/(${FOLDER_REGEX})/")
    endif()
    if(EXCLUDED_FILES)
        string(REPLACE ";" "|" FILE_REGEX "${EXCLUDED_FILES}")
        list(FILTER ${TARGET_LIST} EXCLUDE REGEX "/(${FILE_REGEX})$")
    endif()
endmacro()

# ------------------------------------------------------------------------------
# |> BUILD STATE LOGIC
# ------------------------------------------------------------------------------
# -> TRUE when a usable precompiled archive already exists on disk.
function(is_stm_precompiled OUTPUT_VAR)
    if (EXISTS "${PATH_PRECOMPILED_LIBRARY}/libstm32_precompiled.a")
        set(${OUTPUT_VAR} TRUE PARENT_SCOPE)
    else()
        set(${OUTPUT_VAR} FALSE PARENT_SCOPE)
    endif()
endfunction()

# ------------------------------------------------------------------------------
# |> STM32 HAL / MIDDLEWARE WIRING
# ------------------------------------------------------------------------------
# -> Reuse branch: import the archive, then re-supply the CubeMX include dirs,
#    defines and application/startup sources (which normally come from the
#    generated subdirectory we are intentionally skipping).
# -> First-build branch: let the generated subdirectory compile + link the HAL
#    object libs into the exe as usual, and archive those same objects into
#    PATH_PRECOMPILED_LIBRARY for the next build.
function(create_stm32_lib)
    is_stm_precompiled(ARCHIVE_PRESENT)

    if(ARCHIVE_PRESENT)
        message(STATUS "Using existing precompiled HAL: ${PATH_PRECOMPILED_LIBRARY}/libstm32_precompiled.a")

        add_library(stm32_precompiled STATIC IMPORTED GLOBAL)
        set_target_properties(stm32_precompiled PROPERTIES
            IMPORTED_LOCATION "${PATH_PRECOMPILED_LIBRARY}/libstm32_precompiled.a"
        )

        # The generated subdir is skipped, so supply its pieces ourselves.
        target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${MX_Application_Src})
        target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${MX_Include_Dirs})
        target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE ${MX_Defines_Syms})
        target_link_libraries(${CMAKE_PROJECT_NAME} stm32_precompiled)

    else()
        message(STATUS "No precompiled HAL found. Building and archiving it now...")

        add_subdirectory(${CMAKE_SOURCE_DIR}/cmake/stm32cubemx)

        # Bundle the CubeMX object libraries into one real static archive.
        add_library(stm32_precompiled STATIC
            $<TARGET_OBJECTS:STM32_Drivers>
            $<TARGET_OBJECTS:Interfaces>
            $<TARGET_OBJECTS:LevelX>
            $<TARGET_OBJECTS:FileX>
            $<TARGET_OBJECTS:ThreadX>
            $<TARGET_OBJECTS:NetXDuo>
        )
        set_target_properties(stm32_precompiled PROPERTIES
            LINKER_LANGUAGE C
            ARCHIVE_OUTPUT_DIRECTORY "${PATH_PRECOMPILED_LIBRARY}"
            OUTPUT_NAME "stm32_precompiled"
        )
        target_link_libraries(stm32_precompiled PUBLIC stm32cubemx)

        add_custom_command(TARGET stm32_precompiled POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E make_directory "${PATH_PRECOMPILED_LIBRARY}"
            COMMAND ${CMAKE_COMMAND} -E copy
                    "$<TARGET_FILE:stm32_precompiled>"
                    "${PATH_PRECOMPILED_LIBRARY}/libstm32_precompiled.a"
            COMMENT "Saving precompiled HAL -> ${PATH_PRECOMPILED_LIBRARY}/libstm32_precompiled.a"
        )

        add_dependencies(${CMAKE_PROJECT_NAME} stm32_precompiled)
    endif()
endfunction()
