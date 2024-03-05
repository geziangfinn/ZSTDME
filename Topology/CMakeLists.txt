add_executable(
    TopoTest 
    topo_test.cpp
    topology.cpp
    ${PROJECT_SOURCE_DIR}/CTSCommon/arghandler.cpp
    ${PROJECT_SOURCE_DIR}/CTSCommon/objects.cpp
    ${PROJECT_SOURCE_DIR}/CTSDB/ctsdb.cpp
    ${PROJECT_SOURCE_DIR}/Parser/parser.cpp
)
INCLUDE_DIRECTORIES (
	${PROJECT_SOURCE_DIR}/Parser
	${PROJECT_SOURCE_DIR}/CTSCommon
	${PROJECT_SOURCE_DIR}/CTSDB
    ${PROJECT_SOURCE_DIR}/Topology
    ${PROJECT_SOURCE_DIR}/Plotter
)