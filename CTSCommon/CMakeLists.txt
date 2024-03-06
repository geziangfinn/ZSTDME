add_executable(
    SEGCUT
    segmentCutTest.cpp
    objects.cpp
)
INCLUDE_DIRECTORIES (
	${PROJECT_SOURCE_DIR}/CTSCommon
    ${PROJECT_SOURCE_DIR}/Plotter
)