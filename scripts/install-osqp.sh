echo "====================="
echo "= OSQP Installation ="
echo "====================="

cd ../third_party/osqp

git submodule update --init --recursive

cd -

rm -rfi ../third_party/osqp/build -y

mkdir -p ../third_party/osqp/build && cd ../third_party/osqp/build

cmake -G "Unix Makefiles" -DCMAKE_INSTALL_PREFIX:PATH=/home/${USERNAME}/autoware.ai/src/autoware/common/HLBC/third_party/osqp/ ..

sleep 1

cmake --build .

sleep 1

cmake --build . --target install

cd -

echo "==============================="
echo "= OSQP Installation completed ="
echo "==============================="
