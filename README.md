## 1、环境安装
```
1、创建环境
conda create -n yolo python=3.7
conda activate yolo
2、安装pytorch
conda install pytorch torchvision cudatoolkit=11.3 -c pytorch
#或者用pip
pip3 install torch==1.9.1+cu111 torchvision==0.10.1+cu111 torchaudio==0.9.1 -f https://download.pytorch.org/whl/torch_stable.html -i https://pypi.tuna.tsinghua.edu.cn/simple
11.3为cuda版本号
3、克隆yolov5
git clone https://github.com/ultralytics/yolov5  # clone
cd yolov5
pip install -i https://pypi.tuna.tsinghua.edu.cn/simple -r requirements.txt  
//为了避免安装失误采用镜像安装https://pypi.tuna.tsinghua.edu.cn/simple
```
## 2、快速测试
```
python detect.py --source ./data/images/ --weights weights/yolov5s.pt --conf 0.4
```
## 3、txt数据集训练
```
python train.py --data data/drone.yaml --cfg models/yolov5s.yaml --weights weights/yolov5s.pt --batch-size 16 --epochs 50
可视化
tensorboard --logdir=./runs
=后面是存放exp训练过程文件的路径
```
## 4、启动ROS节点
```
python ros_detect.py
```
## 5、断点续训
```
python train.py --resume  #找到最近的权重文件继续训练
python train.py --resume runs/exp0/weights/last.pt  #指定权重文件进行训练
```
