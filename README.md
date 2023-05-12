# DSP-SLAM编程记录

原文链接：
>https://github.com/JingwenWang95/DSP-SLAM#building-script

注：本文档只用于学习使用，只是用来记录DSP-SLAM开源代码的编译过程，不做其他使用。

## Enviroment
  显卡驱动：470.161.03（可以不一样）
  
  anaconda需要提前安装 我安装了anaconda3，安装过程可参考：
  >https://blog.csdn.net/weixin_70026476/article/details/128184135?spm=1001.2014.3001.5501

cuda和cudnn建议先别自己装！！！提前下好的，建议先删除（挺方便的）因为源代码中已经提供了相应的文件，在后续安装中会逐步安装上
***
## 安装流程
1、下载源码：
```bash
sudo git clone --recursive https://github.com/JingwenWang95/DSP-SLAM.git
```
或者直接下载我已经修改过的代码：
```bash
sudo git clone --recursive https://github.com/Ruan-xianzheng/DSP-SLAM-explain.git #留意一下pybind11有没有下下来
```
2、进入文件
```bash
cd DSP-SLAM/
```
3、编译该文件中包含的cuda环境： 
```bash
sudo ./build_cuda113.sh --install-cuda --build-dependencies --create-conda-env
```
4、安装cuda:
```bash
sudo sh cuda_11.3.0_465.19.01_linux.run
```
5、删除cuda原文件：
```bash
sudo rm cuda_11.3.0_465.19.01_linux.run
```
6、修改bashrc文件
```bash
sudo gedit ~/.bashrc  
```
在里面添加cuda的信息：
```cpp
export PATH=/usr/local/cuda-11.3/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-11.3/lib64:$LD_LIBRARY_PATH
```
7、进入Thirdparty，安装其他库
```bash
cd Thirdparty
```
8、安装opencv 3.4.1 （如果下载不成功，可以多尝试）
```bash
sudo git clone --branch 3.4.1 --depth=1 https://github.com/opencv/opencv.git
```
```bash
cd opencv/
```
```bash
sudo mkdir build
```
```bash
cd build/
```
```bash
sudo cmake       -DCMAKE_BUILD_TYPE=Release       -DWITH_CUDA=OFF        -DBUILD_DOCS=OFF        -DBUILD_PACKAGE=OFF       -DBUILD_TESTS=OFF        -DBUILD_PERF_TESTS=OFF        -DBUILD_opencv_apps=OFF       -DBUILD_opencv_calib3d=ON        -DBUILD_opencv_cudaoptflow=OFF        -DBUILD_opencv_dnn=OFF        -DBUILD_opencv_dnn_BUILD_TORCH_IMPORTER=OFF        -DBUILD_opencv_features2d=ON       -DBUILD_opencv_flann=ON       -DBUILD_opencv_java=ON        -DBUILD_opencv_objdetect=ON        -DBUILD_opencv_python2=OFF        -DBUILD_opencv_python3=OFF        -DBUILD_opencv_photo=ON       -DBUILD_opencv_stitching=ON        -DBUILD_opencv_superres=ON        -DBUILD_opencv_shape=ON        -DBUILD_opencv_videostab=OFF       -DBUILD_PROTOBUF=OFF       -DWITH_1394=OFF        -DWITH_GSTREAMER=OFF        -DWITH_GPHOTO2=OFF        -DWITH_MATLAB=OFF        -DWITH_NVCUVID=OFF       -DWITH_OPENCL=OFF       -DWITH_OPENCLAMDBLAS=OFF       -DWITH_OPENCLAMDFFT=OFF       -DWITH_TIFF=OFF        -DWITH_VTK=OFF        -DWITH_WEBP=OFF  ..
```
```bash
 sudo make -j8
```
```bash
  OpenCV_DIR=$(pwd) #设置opencv的路径
```
```bash
cd ../..
```
9、安装eigen库
```bash
sudo git clone --branch=3.4.0 --depth=1 https://gitlab.com/libeigen/eigen.git
```
```bash
cd eigen/
```
```bash
sudo mkdir build
```
```bash
sudo mkdir install
```
```bash
cd build
```
```bash
sudo cmake -DCMAKE_INSTALL_PREFIX="$(pwd)/../install" ..
```
```bash
sudo make -j8
```
```bash
sudo make install
```
```bash
cd ../..
```
10、安装pangolin

**这里和源码介绍的不一样,从以下路径里下载pangolin0.6版本，否则后续会出现问题）**

pangolin的下载地址：
>https://github.com/stevenlovegrove/Pangolin/archive/refs/tags/v0.6.zip
```bash
cd Pangolin/
```
```bash
sudo mkdir build
```
```bash
cd build
```
```bash
sudo cmake ..
```
```bash
sudo make -j8
```
```bash
sudo make install #这一步也是需要的，不然后面会报如下错误，找不到.h文件
```
```bash
Pangolin_DIR=$(pwd) 
```
```bash 
cd ../..
```
11、安装g2o
```bash
cd g2o/

sudo mkdir build

cd build

sudo cmake -DEigen3_DIR="$(pwd)/../../eigen/install/share/eigen3/cmake" ..

sudo make -j8

cd ../..
```
12、安装DBOW2
```bash
cd DBoW2/
sudo mkdir build
cd build
sudo cmake -DOpenCV_DIR=$OpenCV_DIR ..
sudo make -j8
cd ../../..
```
13、创造conda环境 通过这一步可以将安装torch等环境 同时创建一个虚拟环境
```bash
conda env create -f environment_cuda113.yml 
```
14、激活虚拟环境
```bash
conda activate dsp-slam
```
15、
```bash
pip install pycocotools==2.0.1
```
```bash
pip install mmcv-full==1.4.0 -f https://download.openmmlab.com/mmcv/dist/cu113/torch1.10.0/index.html
```
```bash
pip install mmdet==2.14.0
```
```bash
pip install mmsegmentation==0.14.1
```
16、
```bash
 cd Thirdparty/
```
```bash
sudo git clone https://github.com/JingwenWang95/mmdetection3d.git
```
```bash
cd mmdetection3d/
```
```bash
sudo chmod -R 777 /home/rxz/DSP-SLAM/Thirdparty/mmdetection3d/mmdet3d #需要给权限，不然会报错
```
```bash
pip install -v -e .
```
```bash
cd ../..
```
17、编译DSP-SLAM：

```bash
sudo mkdir build

cd build

conda_python_bin=`which python`

conda_env_dir="$(dirname "$(dirname "$conda_python_bin")")"

sudo cmake \  -DOpenCV_DIR="$(pwd)/../Thirdparty/opencv/build" \  -DEigen3_DIR="$(pwd)/../Thirdparty/eigen/install/share/eigen3/cmake" \  -DPangolin_DIR="$(pwd)/../Thirdparty/Pangolin/build" \  -DPYTHON_LIBRARIES="$conda_env_dir/lib/libpython3.7m.so" \  -DPYTHON_INCLUDE_DIRS="$conda_env_dir/include/python3.7m" \  -DPYTHON_EXECUTABLE="$conda_env_dir/bin/python3.7" \  ..

sudo make -j8
```
18、配置好环境后，下载数据集：

下载链接：

>https://liveuclac-my.sharepoint.com/:f:/g/personal/ucabjw4_ucl_ac_uk/Eh3nHv6D-LZHkuny4iNOexQBGdDVxloM_nwbEZdxeRfStw?e=sYO1Ot

下载的各文件的存放路径（参照相应的config配置文件）：
  
  （1）对于redwood09374数据集，新建一个weights文件夹，将下载的maskrcnn文件夹放入其中；将下载好的deepsdf文件夹放入到weights文件夹中；
  （2）对于kitti07数据集，新建一个weights文件夹，将下载的pointpillars文件放入其中，将下载的maskrcnn文件放入其中，将下载的deepsdf文件放入其中
***
## 编译流程

1、进入dsp-slam虚拟环境

```bash
conda activate dsp-slam
```

2、
```bash
export MESA_GL_VERSION_OVERRIDE=3.3 #这句代码是为了在后续仿真时出现语义渲染的效果，没有这句代码，则只有稠密建图
```
3、开始运行（以redwood09374数据集为例）
```bash
./dsp_slam_mono Vocabulary/ORBvoc.bin configs/redwood_09374.yaml data/09374/ map/redwood_09374/ #各参数介绍：dsp_slam_mono为可执行文件；Vocabulary：词典文件；configs：配置文件； data：数据集地址； map：存放最后地图的路径
```
