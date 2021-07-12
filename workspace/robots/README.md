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
$ bash run.bash tb3
```

実行すると，このような感じで動きます．


![ros1_demo7](https://user-images.githubusercontent.com/164193/124454332-563afe00-ddc3-11eb-92c3-52e7d0dd601f.gif)
