add_executable(
    ParserTest 
    parser_test.cpp
    parser.cpp
    ${PROJECT_SOURCE_DIR}/CTSCommon/arghandler.cpp
    ${PROJECT_SOURCE_DIR}/CTSCommon/objects.cpp
    ${PROJECT_SOURCE_DIR}/CTSDB/ctsdb.cpp
)
INCLUDE_DIRECTORIES (
	${PROJECT_SOURCE_DIR}/Parser
	${PROJECT_SOURCE_DIR}/CTSCommon
	${PROJECT_SOURCE_DIR}/CTSDB
)