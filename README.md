# hakoniwa-ros-samples

箱庭ROS機能の利用方法を習得するためのサンプルプロジェクト．

HackEVとTurtleBot3を対象として，箱庭上でROS制御するためのサンプルプログラムを公開しています．

# 対応しているROSバージョン
ROS1/ROS2 両方で動作します．

* ROS1/melodic
* ROS2/foxy

# 準備
## 本リポジトリをクローンする

```
$ git clone --recursive https://github.com/toppers/hakoniwa-ros-samples.git
```

## 作業フォルダへ移動する

### ROS1 の場合

```
$ cd hakoniwa-ros-samples/ros1/workspace
```

### ROS2 の場合

```
$ cd hakoniwa-ros-samples/ros2/workspace
```

## ROSサンプルプログラムをビルドする

作業フォルダ上で以下のコマンドを実行する

```
$ bash clean.bash
$ bash build.bash
```

## Unity バイナリをダウンロードする

以下のフォルダに移動し，Unity バイナリをダウンロードする

### ROS1 の場合

```
$ cd hakoniwa-ros-samples/ros1
$ bash unity/download.bash
```

### ROS2 の場合

```
$ cd hakoniwa-ros-samples/ros2
$ bash unity/download.bash
```

# シミュレーション実行

## Unity バイナリを起動する

### ROS1 の場合

Windows エクスプローラで，「hakoniwa-ros-samples\ros1\unity\Ros1WindowsBinary」内の「tb3_ros1.exe」をダブルクリックする．

### ROS2 の場合

Windows エクスプローラで，「hakoniwa-ros-samples\ros2\unity\Ros2WindowsBinary」内の「tb3_ros2.exe」をダブルクリックする．


## ROS-TCP-Endpointを起動する

作業フォルダ上で以下のコマンドを実行する

```
$ bash launch.bash
```

## ROSプログラムを実行する
作業フォルダ上で，以下のコマンドを実行する

```
$ bash run.bash ev3
```

```
$ bash run.bash tb3
```

## デモ

デモ内容としては，HackEVがライントレースでゴールを目指して走行中に，TurtleBot3が障害物として走行路に停止しているような状況です．互いにうまく回避行動をとりながら動くような感じです．

![ros2_demo3](https://user-images.githubusercontent.com/164193/126057869-60e4d492-6c32-44bf-a34a-92595ac08ccb.gif)

