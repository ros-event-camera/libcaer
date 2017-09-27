# script that generates a python bindings of libcaer, it uses swig and gcc
# check that your python version and adjust accordingly 
swig -python -cpperraswarn -I/usr/local/include/ pyflags.i
gcc -fPIC -c pyflags_wrap.c -I/usr/include/python2.7
ld -shared -lcaer pyflags_wrap.o -o _libcaer_wrap.so
