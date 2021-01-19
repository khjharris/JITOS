project( common )

set( common_source_files
  hw_Iface.c
  hw_Iface.h
  JITOS.c
  JITOS.h
  syscalls.c
)

add_library(common ${ common_source_files })
