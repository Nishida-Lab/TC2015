# steer_ctrl

3号機のステア制御にはステッピングモータを利用していて、そのステッピングモータを制御するコントローラとしてArduinoを利用しています。

## 使い方について

`./steer_ctrl/steer_ctrl.ino` は `ros_serial` を利用しているため、コンパイルするためにはライブラリが必要です。

必要なパッケージをインストールします。
```bash
sudo apt-get install ros-indigo-rosserial-arduino
sudo apt-get install ros-indigo-rosserial
```

`ros_lib`をコンパイルします。
```
cd <sketchbook>/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
```
`<sketchbook>`ディレクトリは通常、ホームディレクトリ直下にあります。
