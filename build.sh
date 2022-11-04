cd ./thirdparty/osmap
protoc --cpp_out=. osmap.proto

cd ../..

cd ./build 
cmake .. 
make -j4
