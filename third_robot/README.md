# Third Robotについて

## 課題

* [ ] 速度制御が不安定。突然、減速したりする。
  * [ ] waypointsごとに減速してるように見える。
* [x] 止まるのが遅い。最悪、ぶつかる。
  * PIDパラメータを調整。
* [ ] 正面に突然障害物が入ると、リカバリー出来ない。バックしても切り返さず、進めない。
  * [ ] ちょっと改善できたけど正面に来た時はやはりリカバリーできない。また、障害物に近づきすぎた場合には障害物を無視することがある。 

## `third_robot`の構成について

* third_robot_2dnav  
  自律走行に必要な設定ファイルとか、ノードの起動とか。コストマップのパラメータとかもこれで設定。
  
* third_robot_description  
  `urdf`とか。
  
* third_robot_driver  
  3号機とROSをつなぐノード。
  
* third_robot_gmapping  
  `gmapping`をする`launch`があるだけ。
  
* third_robot_lower_step_detector  
  下に下がる段差は標準だと検知できないので、下がる場合にも反転して障害物があるように見せるノード。
  
* third_robot_nav_goals  
  `actionlib`を使って、ゴールの情報を`third_robot_2dnav`に送る。wayporintのセーブとかもできる。
  
* third_robot_offset_urg  
  今は使っていない。昔はLRFの位置のオフセットのtfを発行してた。今は`third_robot_description`が行っている。
  
* third_robot_sound  
  話す言葉を生成する。
  
* third_robot_talker  
  ゴールした時とかに音声を流す。`actionlib`のコールバック関数を利用。
  
* third_robot_urg_filter  
  LRFが普通に`inf`とかを返してくる。`inf`がくるとコストマップが反映されず、障害物情報がクリアされないので、`inf`の値は近くのレーザの値に置き換える。
  
## `third_robot`を使うにあたって

### udevの設定

3号機では様々なデバイスをPCに接続して使います。
Linuxの特性上、デバイスは接続された順番に番号が振られて行きますが、接続した順番でパスが変わってしまうのはミスにつながるので、`udev`を使って制御するようにしました。
詳しくは次のドキュメントを参照のこと。

[udevの設定について](./.documents/udev/AboutUdev.md)
