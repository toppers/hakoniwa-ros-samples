# 前提とする環境

以下がインストールされていること．

- ROS/melodic

# 準備

## 本リポジトリをクローンする

```
$ git clone https://github.com/toppers/hakoniwa-ros-samples.git
$ cd hakoniwa-ros-samples/workspace/robots
```

## ROS-TCP-Endpointを起動する

```
bash launch.bash ev3_tb3
```

## Unity バイナリをダウンロードする

```
$ bash unity/download.bash
```

## ROSサンプルプログラムをビルドする
```
$ cd hakoniwa-ros-samples/workspace/robots
$ bash clean_build.bash
```

# シミュレーション実行

## Unity バイナリを起動する
Windows エクスプローラで，「hakoniwa-ros-samples\workspace\turtlebot3\unity\WindowsBinary」内の「ros-sample.exe」をダブルクリックする． 

## ROSプログラムを実行する
別端末で，以下のコマンドを実行する

```
$ bash run.bash ev3
```

```
$ bash run.bash tb3
```

実行すると，このような感じで動きます．


![debug](https://user-images.githubusercontent.com/164193/125229001-cdffb000-e310-11eb-8a01-6221ed7b2620.gif)
