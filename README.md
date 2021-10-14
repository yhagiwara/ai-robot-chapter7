# AI-ROBOT-text

## ROS2のインストール手順
以下のウェブページの手順に従ってROS2(foxy)をインストールしてください．  
※OSは、Ubuntu 20.04(LTS)を使用してください．  
https://docs.rs.org/en/foxy/Installation/Ubuntu-Install-Debians.html  

## ROS2の読み込み
ROS2のコマンドなどを読み込むためには以下のコマンドを実行します． 

```bash
source /opt/ros/foxy/setup.bash  
```

しかし，毎回端末を開いて上記のコマンドを実行するのは手間なので，~/.bashrcに上記のコマンドを追記し，自動的に読み込まれるようにすることをお勧めします．

## ROS2の動作を確認する
正しくROS2が読み込まれているかを確認するため，turtle simと呼ばれるシミュレータを起動してみましょう．
https://docs.ros.org/en/foxy/Tutorials/Turtlesim/Introducing-Turtlesim.html

```bash
端末１：ros2 run turtlesim turtlesim_node
```
```bash
端末２：ros2 run tutrlesim turtle_teleop_key
```
ROS2のインストールと動作確認は終了です。
