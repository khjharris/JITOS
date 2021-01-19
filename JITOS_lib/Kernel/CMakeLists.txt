cmake_minimum_required(VERSION 3.10)

# Set the project name and version
project(Kernel VERSION 1.0)

set( kernel_source_files 
  main.c
)

add_library( Kernel ${ kernel_source_files } ) 
