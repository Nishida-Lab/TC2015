# third_robot_lower_step_detector
## Summary
3号機の斜め下向きLRFで下方段差を検出する機能

## Directory
```
- third_robot_lower_step_detector
    + config      // yaml config files
    + launch      // launch files
    + scripts     // python codes
```
## Preparation
- `/config/lower_step_detector.yaml` でパラメータを設定して下さい。
  - laser_intensity_max: 3.0
    - 何[m]以上を段差とみなすか．デフォルトは3[m]．
  - virtual_laser_intensity: 1.0
    - 段差とみなした場合、何[m]の位置に障害物があると仮定するか．デフォルトは1[m]．
  - laser_scan_range_deg: 180
    - LRF の認識範囲[°]．デフォルトは180[°]．
  - detect_step_angle_min_deg: 40.0
    - LRF 認識範囲の両端で段差検出を除外する範囲[°]．指定方法は片側の角度設定．デフォルトは40[°]．左右それぞれ40°カットする．
    - 例えば、高さ0.75[m], 30[°]傾けたセンサが地面を検出した際に出力する値を計算すると、中心から左右に50[°]傾く辺りから急激に大きな値を取り始める。
    - こうなると、何も検出していないのか、段差を検出しているのかを判別できない。そのため、両端は最初から除外するのが妥当と判断。

## How to launch
- デモプログラム起動方法
```
roslaunch third_robot_lower_step_detector lower_step_detector_demo.launch
```
- トピックを変更したい時
```
roslaunch third_robot_lower_step_detector lower_step_detector_demo.launch sub_topic:=hogehoge
``` 
- 補足
  - これで、検出プログラムとrviz が起動します。この後bagファイル再生等をして下さい。
  - デフォルトでは、`/base_scan1`をpublish すれば、修正後の情報である`/base_scan1_fix`がpublish されます。
  - トピックの変更をした場合、このノードがsubscribeするtopic名を`/base_scan1`から`/hogehoge`に変えられます。publish されるトピックは`/hogehoge_fix` となります。
