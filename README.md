# oni_nav_contoler

鬼ごっこ（鬼役ロボット）向けの **ROS 2 Humble ナビゲーション・制御ノード**（Node 2）です。  
自己位置推定（EKF）、ターゲット座標のセンサ融合、Acados による NMPC を統合し、3 輪オムニ向けの `cmd_vel` を 20 Hz で出力します。

詳細な仕様は [doc/specification.md](doc/specification.md) を参照してください。

## 機能概要

| モジュール | 説明 |
|-----------|------|
| 状態推定器 (EKF) | IMU + オドメトリ融合、スリップ補償 |
| ターゲット座標特定 | フェーズ A（BBox）/ フェーズ B（LiDAR 視錐台 + DBSCAN） |
| NMPC (Acados) | ターゲット追従・障害物回避・滑らかさ最適化 |
| モーション制御 | Node 4 状態連携、減速停止、再探索旋回 |
| フェイルセーフ | ターゲットロスト検出、`/nav/target_lost` 通知 |

## 前提環境

- Ubuntu 22.04
- ROS 2 Humble
- CMake 3.8+
- Eigen3
- Python 3（Acados コード生成・モック用）
- **Acados v0.3.6**（`deps/acados_install` にローカルインストール）

## ワークスペース構成

`oni_msgs` は別リポジトリです。本リポジトリをワークスペースルートとして、依存パッケージを取得してからビルドします。

```bash
cd /path/to/oni_nav_contoler

# oni_msgs をワークスペースに取得（初回のみ）
vcs import < oni_nav.repos
# または: git clone git@github.com:kei487/oni_msgs.git oni_msgs
```

## ビルド

```bash
cd /path/to/oni_nav_contoler
source /opt/ros/humble/setup.bash

# 初回のみ: Acados のビルド（未実施の場合）
# 手順は doc/specification.md の「Acados セットアップ」を参照

colcon build --symlink-install
source install/setup.bash
```

### テスト

```bash
export LD_LIBRARY_PATH=$PWD/deps/acados_install/lib:$LD_LIBRARY_PATH
colcon test --packages-select oni_nav_controller
colcon test-result --verbose
```

## 起動

`exec.bash` が ROS / ワークスペース / Acados ライブラリパスを設定して起動します。

```bash
# 本番（実センサ接続時）
./exec.bash

# モックセンサ付き動作確認
./exec.bash mock chase

# モード一覧: chase | idle | lost | stop
./exec.bash mock lost
./exec.bash --help
```

手動起動する場合:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH=$PWD/deps/acados_install/lib:$LD_LIBRARY_PATH

ros2 launch oni_nav_controller nav_controller.launch.py
ros2 launch oni_nav_controller mock_nav_test.launch.py mode:=chase
```

## トピック（デフォルト）

| 方向 | トピック | 型 |
|------|----------|-----|
| 入力 | `/perception/bbox` | `oni_msgs/BBox` |
| 入力 | `/livox/imu` | `sensor_msgs/Imu` |
| 入力 | `/livox/lidar` | `sensor_msgs/PointCloud2` |
| 入力 | `/odom` | `nav_msgs/Odometry` |
| 入力 | `/system/state` | `oni_msgs/SystemState` |
| 出力 | `/cmd_vel` | `geometry_msgs/Twist` |
| 出力 | `/nav/target_lost` | `std_msgs/Bool` |
| 出力 | `/nav/debug` | `oni_msgs/NavDebug` |

## パッケージ構成

```
oni_nav_contoler/             # ワークスペースルート
├── exec.bash                 # 起動スクリプト
├── oni_nav.repos             # vcs import 用（oni_msgs）
├── doc/
│   ├── specification.md      # 仕様書（本ドキュメントの詳細版）
│   ├── requirement.txt     # 要件定義（原文）
│   └── plan.md               # 開発計画
├── oni_msgs/                 # 別リポジトリ（vcs import / git clone）
├── oni_nav_controller/       # メインノード
│   ├── config/nav_controller.yaml
│   ├── launch/
│   ├── scripts/              # NMPC 生成・モック用
│   └── c_generated_code/     # Acados 生成 C コード
└── deps/                     # Acados（.gitignore）
```

関連リポジトリ: [oni_msgs](https://github.com/kei487/oni_msgs) — カスタムメッセージ定義

## パラメータ

主要パラメータは `oni_nav_controller/config/nav_controller.yaml` で設定します。  
トピック名、EKF ノイズ、ターゲット追従閾値、NMPC 制約、フェイルセーフ時間などを含みます。

**注意**: `double` 型パラメータは YAML 上で `300.0` のように小数表記にしてください（例: `target_lost_timeout_ms`）。

## ドキュメント

| ファイル | 内容 |
|----------|------|
| [doc/specification.md](doc/specification.md) | システム仕様書（インターフェース・アルゴリズム・実装） |
| [doc/requirement.txt](doc/requirement.txt) | 要件定義書 |
| [doc/plan.md](doc/plan.md) | 開発フェーズ計画 |

## ライセンス

MIT License（[LICENSE](LICENSE)）
