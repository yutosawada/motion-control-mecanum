# モーションコントロール Mecanum パッケージ

このパッケージは、ROS 2 向けのメカナムホイール式モーションコントローラのサンプル実装です。CANopen DS402 プロトコルに対応したモータドライバを想定しており、速度指令や各種パラメータ設定を行うための API を提供します。

## 主な機能

- DS402 の *Modes of Operation* オブジェクト (`0x6060`) の設定
- Profile Velocity Mode (`0x60FF`) を用いた目標速度指令
- 速度しきい値 (`0x606F`)、速度ウィンドウ (`0x606D`) の設定
- クイックストップオプション (`0x605A`) およびクイックストップ減速度 (`0x6085`) の設定
- 最大トルク制限 (`0x6072`)、プロファイル加速度 (`0x6083`)、プロファイル減速度 (`0x6084`)
- 終端速度 (`0x6082`)、プロファイル速度 (`0x6081`) の設定
- 実トルク値 (`0x6077`) および実速度値 (`0x606C`) の取得
- サーボ ON/OFF を行うサービスインタフェース

モーションコントローラノードは `geometry_msgs/msg/Twist` を受け取り、各モータへの指令値を計算します。`sensor_msgs/msg/JointState` と `nav_msgs/msg/Odometry` を発行し、TF もブロードキャストします。

## ビルド方法

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# 本リポジトリをクローン
cd ..
colcon build --packages-select motion-control-mecanum-pkg
```

ビルド後は以下のように起動できます。

```bash
source install/setup.bash
ros2 launch motion-control-mecanum-pkg motion_control_mecanum_launch.py
```

## 再利用可能な CI ワークフロー

このリポジトリには `.github/workflows/ci.yml` に定義された再利用可能な GitHub Actions ワークフローが含まれています。他の ROS 2 パッケージから `workflow_call` を用いて次のように呼び出せます。

```yaml
name: CI
on:
  pull_request:
jobs:
  build-and-test:
    uses: <owner>/motion-control-mecanum/.github/workflows/ci.yml@main
    with:
      repo-path: <your-package-name>
```

