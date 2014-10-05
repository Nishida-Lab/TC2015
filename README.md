# Third Robot Driver

## 準備
`wstool`を使ったことがなかったら`ros`のワークスペースで以下のコマンドを実行する．
```
$ cd ~/catkin_ws/src
$ wstool init .
```

## ビルド方法
```
$ cd ~/catkin_ws/src
$ wstool set --git third_robot_driver https://github.com/AriYu/third_robot_driver.git
$ wstool update
```
`imcs01_driver`を配置する．`imcs01_driver`をダウンロードして解凍し，`third_robot_driver/include/ThirdRobotInterface/`に配置する．
解凍した`imcs01_driver`が`Downloads`ディレクトリにあると仮定すると，
```
$ cp -r ~/Downloads/imcs01_driver ~/catkin_ws/src/third_robot_driver/include/ThirdRobotInterface/
```
とする．その後
```
$ cd ~/catkin_ws
$ catkin_make
```
