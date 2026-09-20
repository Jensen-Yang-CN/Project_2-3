if(NOT DEFINED APP_EXECUTABLE OR NOT EXISTS "${APP_EXECUTABLE}")
    message(FATAL_ERROR "APP_EXECUTABLE is missing: ${APP_EXECUTABLE}")
endif()
if(NOT DEFINED DESTINATION)
    message(FATAL_ERROR "DESTINATION is not set")
endif()

if(POLICY CMP0207)
    cmake_policy(SET CMP0207 NEW)
endif()

file(GET_RUNTIME_DEPENDENCIES
    EXECUTABLES "${APP_EXECUTABLE}"
    DIRECTORIES ${SEARCH_DIRECTORIES}
    RESOLVED_DEPENDENCIES_VAR _resolved_dependencies
    UNRESOLVED_DEPENDENCIES_VAR _unresolved_dependencies
    CONFLICTING_DEPENDENCIES_PREFIX _dependency_conflicts
    PRE_EXCLUDE_REGEXES
        "^api-ms-"
        "^ext-ms-"
        "^libpcl_.*"
        "^libboost_.*"
        "^av(codec|device|filter|format|util)-.*"
        "^sw(resample|scale)-.*"
        "^libGetLidarData_LS\\.dll$"
        "^[Ww][Pp][Cc][Aa][Pp]\\.dll$"
        "^[Pp][Aa][Cc][Kk][Ee][Tt]\\.dll$"
        "^AzureAttestManager\\.dll$"
        "^AzureAttestNormal\\.dll$"
        "^HvsiFileTrust\\.dll$"
        "^PdmUtilities\\.dll$"
        "^wpaxholder\\.dll$"
    POST_EXCLUDE_REGEXES
        "^[A-Za-z]:[/\\\\][Ww][Ii][Nn][Dd][Oo][Ww][Ss][/\\\\][Ss][Yy][Ss][Tt][Ee][Mm]32[/\\\\].*"
        "^[A-Za-z]:[/\\\\][Ww][Ii][Nn][Dd][Oo][Ww][Ss][/\\\\][Ss][Yy][Ss][Ww][Oo][Ww]64[/\\\\].*"
)

foreach(_dependency IN LISTS _resolved_dependencies)
    execute_process(
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different
                "${_dependency}" "${DESTINATION}"
        COMMAND_ERROR_IS_FATAL ANY
    )
endforeach()

if(_unresolved_dependencies)
    list(SORT _unresolved_dependencies)
    message(WARNING
        "Unresolved non-system runtime dependencies: "
        "${_unresolved_dependencies}")
endif()

list(LENGTH _resolved_dependencies _dependency_count)
message(STATUS
    "Copied ${_dependency_count} recursive runtime dependencies to "
    "${DESTINATION}")
