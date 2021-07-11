# 前提とする環境

以下がインストールされていること．

- ROS/melodic

# 準備

## 本リポジトリをクローンする

```
$ git clone https://github.com/toppers/hakoniwa-ros-samples.git
$ cd hakoniwa-ros-samples/workspace/turtlebot3
```

## ROS-TCP-Endpointを起動する

```
bash launch.bash
```

## Unity バイナリをダウンロードする

```
$ bash unity/download.bash
```

## ROSサンプルプログラムをビルドする
```
$ cd hakoniwa-ros-samples/workspace/turtlebot3
$ bash clean_build.bash
```

# シミュレーション実行

## Unity バイナリを起動する
Windows エクスプローラで，「hakoniwa-ros-samples\workspace\turtlebot3\unity\WindowsBinary」内の「ros-sample.exe」をダブルクリックする． 

## ROSプログラムを実行する
別端末で，以下のコマンドを実行する

```
$ bash run.bash
```
