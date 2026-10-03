# omni_chassis

N輪オムニの足回りドライバ (ROS 2 パッケージ)。
機体速度 `cmd_vel` (vx, vy, omega) を逆運動学で各車輪の角速度にし、
速度・加速度の制限をかけて、車輪ごとにモータ軸の目標角速度 [rad/s] (`std_msgs/Float64`) として出す。

[holonomic_tracker](https://github.com/Stew-000-1-0-011/holonomic_tracker) の下位として使う想定
(holonomic_tracker は機体速度までしか扱わず、逆運動学と車輪ごとの制限はこちらの仕事)。
モータドライバとの通信はしない。出力を
[mini_shirasu_ros](https://github.com/Stew-000-1-0-011/mini_shirasu_ros) などの
`target_velocity` につなぐ。

対応環境: ROS 2 Lyrical Luth / Ubuntu 26.04、C++26 (`CMAKE_CXX_STANDARD` で上書き可。C++20 以降が必須)。

## 構成

| ファイル | 役割 |
| --- | --- |
| `include/omni_chassis/kinematics.hpp`, `src/kinematics.cpp` | 逆運動学・順運動学 (最小二乗)・配置の検査・車輪速度の制限。ROS非依存 |
| `src/chassis_node.cpp` | ノード本体。ROSの入出力とパラメータだけを見る |
| `config/chassis_node.yaml` | パラメータ (車輪配置は例なので実機に合わせる) |
| `launch/chassis_node.launch.py` | 起動 |
| `test/kinematics_test.cpp` | 運動学と制限のテスト (ROS不要) |

## 使い方

```bash
ros2 launch omni_chassis chassis_node.launch.py
```

### トピック

| 方向 | トピック | 型 |
| --- | --- | --- |
| sub | `cmd_vel_topic` (既定 `cmd_vel`) | `geometry_msgs/msg/Twist` (機体座標系)。`cmd_vel_stamped: true` で `TwistStamped` |
| pub | `wheels.topic` の各トピック | `std_msgs/msg/Float64` (モータ軸の目標角速度 [rad/s]) |
| pub | `~/wheel_speeds` | `std_msgs/msg/Float64MultiArray` (制限後の車輪軸の角速度 [rad/s]。調整用) |

`control_rate` の周期で全車輪の目標を出し続ける。`cmd_timeout` 以上 `cmd_vel` が来なければ
目標をゼロにする (加速度制限が有効ならそれに沿って減速する)。
holonomic_tracker は止まるとき 1 度ゼロを出して黙るので、それと組み合わせても止まる。

## 車輪の配置

機体座標系 (x 前、y 左) で、車輪ごとに次を並べる。並びは全項目で揃える。

- `wheels.x`, `wheels.y`: 接地点の位置 [m]
- `wheels.angle_deg`: 車輪が**正転**したときに接地点が機体を押す向き [deg]
  (フリーローラはこれに直交するとみなす)
- `wheels.radius`: 車輪半径 [m]
- `wheels.topic`: モータ軸の目標角速度を出すトピック
- `wheels.gear_ratio`: モータ回転 / 車輪回転。**負にすると回転方向を反転**する
  (モータの付け向きで正転の向きが変わる場合に使う)

出力は `車輪角速度 [rad/s] * gear_ratio`。

車輪 i の角速度は、接地点の速度 `(vx - omega*y, vy + omega*x)` の駆動方向成分を半径で割ったもの。
起動時に、配置で (vx, vy, omega) を独立に出せるか (逆運動学の行列 J について J^T J が正則か) を検査し、
出せなければ起動を止める。

例 (`config/chassis_node.yaml`) は 4 輪 X 配置で、各車輪を回転中心まわりの接線方向
(位置の角度 + 90 deg) に向けている。この向きだと、その場回転で全輪が同じ向きに同じ速さで回る。

## 制限

`limits.max_wheel_speed` (車輪軸の角速度) と `limits.max_wheel_accel` (角加速度) は、
**全車輪に同じ倍率をかけて**縮める。逆運動学は線形なので、機体の進む向きと回転の比は変わらない。
加速度は前回出した値からの変化で見る。

## テスト

```bash
colcon test --packages-select omni_chassis && colcon test-result --verbose
```

配置によらない性質 (周速 = 接地点速度の駆動方向成分、逆運動学 -> 順運動学で元に戻る、
制限で進む向きが変わらない) と、4 輪 X 配置・3 輪配置の具体値を確認する。
