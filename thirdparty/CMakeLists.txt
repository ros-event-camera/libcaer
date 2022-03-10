
if(CC_MSVC)
ADD_LIBRARY(stdatomic SHARED
        simple-stdatomic/stdatomic.c
        simple-stdatomic/stdatomic.h
        simple-stdatomic/stdalign.h
        simple-stdatomic/threads.h
        )

INCLUDE_DIRECTORIES(simple-stdatomic)
ENDIF()
