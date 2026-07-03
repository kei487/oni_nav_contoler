# oni_nav_controller 仕様書

| 項目 | 内容 |
|------|------|
| ドキュメント版 | 1.0 |
| 対象ノード | Node 2: Navigation & Control Node |
| ROS ディストリビューション | Humble |
| 制御周期 | 20 Hz（50 ms） |
| ロボット | 3 輪オムニ |

本書は [requirement.txt](requirement.txt) の要件をベースに、現行実装（`oni_nav_controller`）の仕様を記述する。

---

## 1. システム概要

### 1.1 目的

鬼ごっこ鬼役ロボットに対し、以下を一つのノードで提供する。

1. 自己位置・速度の推定（オムニホイールのスリップ補償）
2. 追従対象（人間）のロボット座標系における相対目的地座標の算出
3. NMPC によるリアルタイム速度指令（`cmd_vel`）の生成

### 1.2 システムコンテキスト

```
 Node 1 (Perception)  ── BBox ──────────────┐
 Mid360               ── IMU, PointCloud2 ──┤
 Node 3 (Odometry)    ── Odometry ──────────┼──► Node 2 (oni_nav_controller) ──► cmd_vel
 Node 4 (State FSM)   ── SystemState ───────┘         │
                                                       ├──► /nav/target_lost
                                                       └──► /nav/debug
```

### 1.3 内部モジュール構成

| モジュール | クラス | 役割 |
|-----------|--------|------|
| 統合ノード | `NavControllerNode` | 入出力・20 Hz 制御ループ |
| 状態推定器 | `StateEstimator` | EKF（IMU + オドメトリ） |
| ターゲット座標特定 | `TargetLocalizer` | フェーズ A/B、視錐台、DBSCAN |
| フェーズ遷移 | `PhaseFsm` | A↔B ヒステリシス |
| フェイルセーフ | `Failsafe` | ロスト検出、デッドレコニング |
| 障害物モデル | `ObstacleModel` | LiDAR 点群から最近傍距離 |
| NMPC ソルバー | `NmpcSolver` | Acados C API ラッパー |
| モーション制御 | `MotionController` | 減速・再探索・Node 4 連携 |

---

## 2. 外部インターフェース

### 2.1 入力トピック

| トピック（デフォルト） | メッセージ型 | 送信元 | 内容 |
|------------------------|--------------|--------|------|
| `/perception/bbox` | `oni_msgs/BBox` | Node 1 | ターゲット BBox |
| `/livox/imu` | `sensor_msgs/Imu` | Mid360 | 加速度・角速度 |
| `/livox/lidar` | `sensor_msgs/PointCloud2` | Mid360 | LiDAR 点群 |
| `/odom` | `nav_msgs/Odometry` | Node 3 | 位置・速度 |
| `/system/state` | `oni_msgs/SystemState` | Node 4 | FSM 状態 |

#### `oni_msgs/BBox`

| フィールド | 型 | 説明 |
|-----------|-----|------|
| `stamp` | `builtin_interfaces/Time` | タイムスタンプ |
| `u` | `float64` | BBox 水平中心 [px] |
| `h` | `float64` | BBox 高さ [px] |
| `area_ratio` | `float64` | 画面占有率 (0.0–1.0) |
| `lost` | `bool` | ターゲットロストフラグ |

#### `oni_msgs/SystemState`

| 定数 | 値 | 説明 |
|------|-----|------|
| `IDLE` | 0 | 待機 |
| `CHASE` | 1 | 追従 |
| `LOST` | 2 | ロスト（再探索） |
| `STOP` | 3 | 停止 |

### 2.2 出力トピック

| トピック（デフォルト） | メッセージ型 | 説明 |
|------------------------|--------------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | `linear.x`, `linear.y`, `angular.z` |
| `/nav/target_lost` | `std_msgs/Bool` | `true`: ターゲット完全ロスト通知 |
| `/nav/debug` | `oni_msgs/NavDebug` | デバッグ・可視化用 |

#### `oni_msgs/NavDebug`

| フィールド | 説明 |
|-----------|------|
| `phase` | `PHASE_A` (0) / `PHASE_B` (1) |
| `nav_status` | `NAV_IDLE` / `NAV_CHASE` / `NAV_DECEL` / `NAV_REACQUIRE` / `NAV_STOPPED` |
| `p_target[2]` | 目的地座標 [x, y]（base_link） |
| `x_est[6]` | EKF 状態 [x, y, θ, vx, vy, ω] |
| `cost` | NMPC 最終コスト |

### 2.3 TF

| フレーム（デフォルト） | 用途 |
|------------------------|------|
| `base_link` | ロボット基準。点群・制御の基準座標系 |
| `camera_optical_frame` | BBox 視錐台投影 |
| LiDAR フレーム（点群 `header.frame_id`） | `base_link` へ変換して使用 |

パラメータ `base_frame`, `camera_frame` で変更可能。

---

## 3. 制御ループ

### 3.1 周期

- タイマー周期: `control_rate_hz`（デフォルト 20 Hz）
- 各周期で以下を順に実行:
  1. EKF `predict`（IMU）/ `update`（オドメトリ）
  2. ターゲット座標更新（BBox + LiDAR + TF）
  3. フェイルセーフ判定・`/nav/target_lost` 更新
  4. NMPC 求解（`CHASE` かつターゲット有効時）
  5. `MotionController` による最終速度整形
  6. `/cmd_vel`, `/nav/debug` パブリッシュ

### 3.2 Node 4 状態と出力の対応

| `SystemState` | 挙動 |
|---------------|------|
| `IDLE` | 速度ゼロ |
| `CHASE` | NMPC 出力を使用（ターゲット有効時） |
| `LOST` | 減速後、原地旋回（`reacquire_spin_rate`） |
| `STOP` | 段階的減速して停止 |

ターゲット完全ロスト（フェイルセーフ発動時）は `DECEL` 状態となり、NMPC を停止して減速する。

---

## 4. 状態推定器（EKF）

### 4.1 状態ベクトル

\[
\mathbf{x}_t = [x,\ y,\ \theta,\ v_x,\ v_y,\ \omega]^T
\]

- \((x, y, \theta)\): ワールド（オドメトリ）座標系の位置・姿勢
- \((v_x, v_y, \omega)\): ロボット座標系の並進・角速度

### 4.2 入出力

| 処理 | 入力 | 内容 |
|------|------|------|
| `predict(dt, imu)` | IMU | 加速度 \((a_x, a_y)\)、角速度 \(\omega\) |
| `update_odom(vel, x, y, yaw)` | オドメトリ | 速度観測、位置・姿勢の補正 |

### 4.3 パラメータ

`ekf_process_noise_*`, `ekf_meas_noise_*`（`nav_controller.yaml` 参照）

---

## 5. ターゲット座標特定

### 5.1 フェーズ A（遠中距離）

**条件**: BBox 高さ \(h < h_{thresh}\)（ヒステリシス付き）

**算出**:

1. 方位角: \(\phi = \mathrm{atan2}(u - c_x,\ f_x)\)
2. 擬似距離: \(d_{pseudo} = K_{calib} / h\)
3. 目的地: \(\mathbf{p}_{target} = [d_{pseudo}\cos\phi,\ d_{pseudo}\sin\phi]^T\)

### 5.2 フェーズ B（近距離）

**条件**: \(h \ge h_{thresh} + h_{hysteresis}\) かつ視錐台内 LiDAR が `lidar_stable_frames` 以上連続検出

**算出**:

1. BBox から視錐台を構成し、LiDAR 点群をフィルタ
2. DBSCAN でクラスタリング
3. 脚部サイズ条件を満たす最近クラスタの重心を \(\mathbf{p}_{target}\) とする

### 5.3 ヒステリシス

| 遷移 | 条件 |
|------|------|
| A → B | \(h \ge h_{thresh} + h_{hysteresis}\) かつ LiDAR 安定 |
| B → A | \(h < h_{thresh} - h_{hysteresis}\) または LiDAR 無効 |

### 5.4 LiDAR オクルージョン時

近距離フェーズでクラスタ消失時:

- `use_dead_reckon_on_occlusion: true` → 直前速度でデッドレコニング
- `false` → フェーズ A（BBox のみ）へフォールバック

---

## 6. NMPC（Acados）

### 6.1 モデル

**状態**（6 次元）: \([x, y, \theta, v_x, v_y, \omega]^T\)

**制御**（3 次元）: \([a_x, a_y, \alpha]^T\)

\[
\dot{\mathbf{x}} = \begin{bmatrix}
v_x\cos\theta - v_y\sin\theta \\
v_x\sin\theta + v_y\cos\theta \\
\omega \\
a_x \\
a_y \\
\alpha
\end{bmatrix}
\]

### 6.2 コスト関数（ステージ）

\[
J = \sum_{k} \left( Q\|\mathbf{p}_k - \mathbf{p}_{target}\|^2 + R\|\mathbf{u}_k\|^2 + S\|\Delta\mathbf{u}_k\|^2 + I_{obs}(d_{min,k}) \right)
\]

- \(I_{obs}\): \(d_{min} < d_{safe}\) のときバリアペナルティ（実装: 逆数型）
- 外部パラメータ（ステージ毎）: \([p_{target,x}, p_{target,y}, u_{prev}, d_{min}]\)

### 6.3 制約

| 変数 | 制約 |
|------|------|
| \(v_x, v_y\) | \(\pm v_{max}\) |
| \(\omega\) | \(\pm \omega_{max}\) |
| \(a_x, a_y\) | \(\pm a_{max}\) |
| \(\alpha\) | \(\pm \alpha_{max}\) |

### 6.4 ソルバー設定

| 項目 | 値 |
|------|-----|
| ソルバー | Acados SQP-RTI |
| ホライゾン N | 20 |
| \(\Delta t\) | 0.05 s |
| 生成コード | `oni_nav_controller/c_generated_code/` |

### 6.5 出力

予測軌道のステージ 1 の速度 \([v_x, v_y, \omega]\) を `cmd_vel` 候補として使用。

---

## 7. フェイルセーフ

### 7.1 ターゲット完全ロスト

**条件**（両方を満たし 300 ms 以上継続）:

1. BBox `lost == true`
2. 直前 BBox に基づく視錐台内に有効 LiDAR 点群なし

**動作**:

- NMPC 停止、段階的減速（`decel_rate`）
- `/nav/target_lost` に `true` を publish
- 再捕捉時に `false` を publish

### 7.2 起動時

初回 BBox 受信前はターゲット処理を行わず、誤ロスト通知を出さない。

---

## 8. パラメータ一覧

設定ファイル: `oni_nav_controller/config/nav_controller.yaml`

| カテゴリ | パラメータ | デフォルト | 説明 |
|----------|-----------|-----------|------|
| 制御 | `control_rate_hz` | 20.0 | 制御ループ [Hz] |
| ターゲット | `h_thresh` | 120.0 | 近距離閾値 [px] |
| ターゲット | `h_hysteresis` | 15.0 | ヒステリシス [px] |
| ターゲット | `K_calib` | 200.0 | 距離推定 \(K/h\) |
| NMPC | `horizon_N` | 20 | 予測ホライゾン |
| NMPC | `dt` | 0.05 | 離散化時間 [s] |
| NMPC | `v_max` | 1.0 | 最大速度 [m/s] |
| NMPC | `d_safe` | 0.4 | 障害物安全距離 [m] |
| フェイルセーフ | `target_lost_timeout_ms` | 300.0 | ロスト判定 [ms] |
| フェイルセーフ | `reacquire_spin_rate` | 0.3 | 再探索旋回 [rad/s] |
| フェイルセーフ | `decel_rate` | 2.0 | 減速度 [m/s²] |

トピック名はすべて YAML で変更可能。

---

## 9. ビルド・依存関係

### 9.1 ROS パッケージ

| パッケージ | 説明 |
|-----------|------|
| `oni_msgs` | カスタムメッセージ |
| `oni_nav_controller` | メインノード・ライブラリ |

### 9.2 外部ライブラリ

- Eigen3
- Acados（`deps/acados_install`）
- tf2_ros

### 9.3 Acados セットアップ

```bash
cd deps
git clone --depth 1 --branch v0.3.6 https://github.com/acados/acados.git
cd acados && git submodule update --init --recursive
mkdir build && cd build
cmake -DACADOS_INSTALL_DIR=../acados_install -DACADOS_WITH_QPOASES=ON -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc) install

pip3 install -e interfaces/acados_template
pip3 install casadi
```

NMPC コード再生成:

```bash
export ACADOS_SOURCE_DIR=deps/acados
python3 oni_nav_controller/scripts/generate_nmpc_solver.py
colcon build --packages-select oni_nav_controller
```

---

## 10. 起動・テスト

### 10.1 起動

```bash
./exec.bash              # 本番
./exec.bash mock chase   # モック付き
```

### 10.2 単体テスト

| テスト | 内容 |
|--------|------|
| `test_state_estimator` | EKF |
| `test_phase_fsm` | フェーズ遷移 |
| `test_target_localizer` | 座標算出・ロスト |
| `test_obstacle_model` | 障害物距離 |
| `test_motion_controller` | 減速・状態連携 |
| `test_nmpc_solver` | NMPC 求解 |

### 10.3 動作確認コマンド

```bash
ros2 topic echo /nav/debug --once
ros2 topic hz /cmd_vel
ros2 topic echo /nav/target_lost
```

---

## 11. ファイル構成（実装）

```
oni_nav_contoler/
├── exec.bash
├── doc/
│   ├── specification.md    # 本書
│   ├── requirement.txt
│   └── plan.md
├── oni_msgs/
│   └── msg/                  # BBox, SystemState, NavDebug
└── oni_nav_controller/
    ├── include/oni_nav_controller/
    │   ├── nav_controller_node.hpp
    │   ├── state_estimator.hpp
    │   ├── target_localizer.hpp
    │   ├── phase_fsm.hpp
    │   ├── failsafe.hpp
    │   ├── obstacle_model.hpp
    │   ├── nmpc_solver.hpp
    │   ├── motion_controller.hpp
    │   └── types.hpp
    ├── src/
    ├── c_generated_code/       # Acados 生成物
    ├── config/
    ├── launch/
    ├── scripts/
    └── test/
```

---

## 12. 既知の制限・今後の改善（Phase 5）

- 実機での NMPC 重み・EKF ノイズのチューニングが未完了
- 20 Hz 下での求解時間プロファイリングは環境依存
- BBox 中心維持コストは NMPC の位置追従により間接的に実現（画像空間コストは未実装）
- `deps/` はリポジトリ外管理（`.gitignore`）。クローン後は Acados のビルドが必要

---

## 改訂履歴

| 版 | 日付 | 内容 |
|----|------|------|
| 1.0 | 2026-07-03 | 初版（Phase 0–4 実装反映） |
