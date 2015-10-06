# Third Robot gmapping

3号機でSLAM（gmapping）をする手順を示します。

## bagファイルを作成する

オンラインでやれないこともないですが、パラメータを変えてやり直したいとかあるので、一度保存してオフラインでセンサーデータをすべて再生できるようにしておきます。

まず、3号機固有の設定です。
imcs01のドライバを `insmod` した後、それぞれのデバイスに権限を与えます。

```bash
$ sudo chmod 777 /dev/ttyACM*
$ sudo chmod 777 /dev/urbtc0
```

この時、`ttyACM0`がArduino、`ttyACM1`が前方下のLRFになっているかを確認しましょう。その後、以下の`launch`で必要なノードを立ち上げます。

```bash
$ roslaunch third_robot_gmapping third_robot_gmapping.launch
```

すべてのノードが立ち上がったのを確認したら`rosbag`を始めます。

```bash
$ rosbag record -a -O filename.bag
```

全てのセンサデータを保存するのでbagファイルは非常に大きなサイズになります。10分程度で150MB位になります。

`rosbag`が動き始めたのを確認したら手動でロボットを動かします。

全て取り終えたら`Ctrl-C`で止めるとカレントディレクトリに`bag`ファイルが作成されています。

----

シミュレーション時間を合わせる。

```bash
$ rosparam set use_sim_time true
```

`gmapping`を起動する。

```bash
$ rosrun gmapping slam_gmapping scan:=base_scan
```

上記は最も単純で、ロボットそれぞれに対してパラメータを適切に設定することでより綺麗な地図が作成できる。細かい設定パラメータはwikiを参照。
3号機では以下のようにすると大体うまく行きます。

```bash
$ rosrun gmapping slam_gmapping scan:=base_scan _particles:=30 _delta:=0.1 _maxUrange:=30.0 _lstep:=0.01 _astep:=0.01 _stt:=0.25 _minimumScore:=50
```

バグファイルを再生させる。

```bash
$ rosbag play file_name.bag --clock
```

`rviz`で確認して問題なければ地図を保存する。カレントディレクトリに地図が保存される。

```bash
$ rosrun map_server map_saver
```

保存した地図を使いたいときは地図を保存したディレクトリに移動して

```bash
$ rosrun map_server map_server map.yaml
```

