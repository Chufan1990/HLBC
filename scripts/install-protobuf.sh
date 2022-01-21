echo "========================="
echo "= Protobuf Installation ="
echo "========================="

cd ../third_party/protobuf

git submodule update --init --recursive

./autogen.sh

cd -

rm -rfi ../third_party/protobuf/build -y

mkdir -p ../third_party/protobuf/build && cd ../third_party/protobuf/build

../configure --prefix=/home/${USERNAME}/autoware.ai/src/autoware/common/HLBC/third_party/protobuf/

sleep 1

make -j 8

sleep 1

make check -j 8

sleep 1

make install

cd -

echo "==================================="
echo "= Protobuf Installation completed ="
echo "==================================="
