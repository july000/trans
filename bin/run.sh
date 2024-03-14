current_path=$(pwd)
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/:$current_path/../lib:$current_path/../lib/simone
./Trans
