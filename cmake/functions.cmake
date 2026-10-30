# --- [ generic filtered glob helper ] ----------------------------------------
function(get_filtered_sources OUTPUT_VAR GLOB_EXPRESSION EXCLUDE_REGEX)
    file(GLOB_RECURSE FOUND_FILES "${GLOB_EXPRESSION}")
    list(LENGTH FOUND_FILES FILE_COUNT)
    if(FILE_COUNT EQUAL 0)
        message(WARNING "No files found matching pattern: ${GLOB_EXPRESSION}")
    endif()
    list(FILTER FOUND_FILES EXCLUDE REGEX "${EXCLUDE_REGEX}")
    set(${OUTPUT_VAR} ${FOUND_FILES} PARENT_SCOPE)
endfunction()

# --- [ cubemx includer] ----------------------------------------
function(add_cube_and_export)
    add_subdirectory(${CMAKE_SOURCE_DIR}/cmake/stm32cubemx cube_build)

    # Re-read the variables the subdirectory set, from *its* directory scope.
    get_directory_property(_inc  DIRECTORY ${CMAKE_SOURCE_DIR}/cmake/stm32cubemx DEFINITION MX_Include_Dirs)
    get_directory_property(_defs DIRECTORY ${CMAKE_SOURCE_DIR}/cmake/stm32cubemx DEFINITION MX_Defines_Syms)
    get_directory_property(_app  DIRECTORY ${CMAKE_SOURCE_DIR}/cmake/stm32cubemx DEFINITION MX_Application_Src)

    set(MX_Include_Dirs    "${_inc}"  PARENT_SCOPE)
    set(MX_Defines_Syms    "${_defs}" PARENT_SCOPE)
    set(MX_Application_Src "${_app}"  PARENT_SCOPE)
endfunction()