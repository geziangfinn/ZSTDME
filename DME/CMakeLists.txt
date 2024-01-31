add_executable(
    ZSTDME
    DME_test.cpp
    DME.cpp
    ${PROJECT_SOURCE_DIR}/CTSCommon/arghandler.cpp
    ${PROJECT_SOURCE_DIR}/CTSCommon/objects.cpp
    ${PROJECT_SOURCE_DIR}/CTSDB/ctsdb.cpp
    ${PROJECT_SOURCE_DIR}/Parser/parser.cpp
    ${PROJECT_SOURCE_DIR}/Topology/topology.cpp
)
INCLUDE_DIRECTORIES (
	${PROJECT_SOURCE_DIR}/Parser
	${PROJECT_SOURCE_DIR}/CTSCommon
	${PROJECT_SOURCE_DIR}/CTSDB
    ${PROJECT_SOURCE_DIR}/Topology
    ${PROJECT_SOURCE_DIR}/Plotter
)