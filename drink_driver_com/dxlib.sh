unzip DXLIB_V4.3.zip
gcc -c -D__MAKE_LIB__ DXLIB_v4.3/DXLIB/dxlib_intuitive.cpp DXLIB_v4.3/DXLIB/dxlib.cpp
ar -rcsv libxlib.a dxlib_intuitive.o dxlib.o
mv dxlib.o DXLIB_v4.3/DXLIB/dxlib.h ../drink_server_hw/include/drink_server_hw/
