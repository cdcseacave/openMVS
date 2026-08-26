# benchmark harness for the mesh-reconstruction graph-cut solvers (developer tool)

# List sources files
FILE(GLOB LIBRARY_FILES_C "*.cpp")
FILE(GLOB LIBRARY_FILES_H "*.h")

cxx_executable_with_flags(GraphCutBench "Apps" "${cxx_default}" "Math" ${LIBRARY_FILES_C} ${LIBRARY_FILES_H})
