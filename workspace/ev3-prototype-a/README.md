# 前提とする環境
以下がインストールされていること．
* ROS/melodic

# 準備
## 本リポジトリをクローンする

```
$ git clone https://github.com/toppers/hakoniwa-ros-samples.git
$ cd hakoniwa-ros-samples/workspace/ev3-prototype-a
```
## ROS-TCP-Endpointを起動する

```
$ roscore &
$ run-endpoint.bash
```

## Unity バイナリをダウンロードする

```
$ bash unity/download.bash
```

## ROSサンプルプログラムをビルドする

```
$ cd hakoniwa-ros-samples/workspace/ev3-prototype-a
$ catkin_make
```

# シミュレーション実行

## Unity バイナリを起動する

Windows エクスプローラで，「hakoniwa-ros-samples\workspace\ev3-prototype-a\unity\\WindowsBinary」内の「ros-sample.exe」をダブルクリックする．
成功すると，以下のようにUnityバイナリが起動する．

![image](https://user-images.githubusercontent.com/164193/123540784-b60a2700-d77b-11eb-8d44-4f04b4c90f39.png)

## ROSサンプルプログラムを実行する

```
$ cd hakoniwa-ros-samples/workspace/ev3-prototype-a
$ run.bash
```

## デモ

![ros1_demo3](https://user-images.githubusercontent.com/164193/123541210-46496b80-d77e-11eb-98cc-e7fa685deaa2.gif)
