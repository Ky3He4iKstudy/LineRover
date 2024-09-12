ROOT_DIR="."
docker build -t itmo-ros-aaa -f $ROOT_DIR/Dockerfile --network=host $ROOT_DIR
