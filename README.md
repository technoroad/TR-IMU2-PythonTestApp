# IMU Platform2 Python Tools

このリポジトリには、`IMU Platform2` の通信用 Python ツールが入っています。

- `CAN FD` での確認用
- `USB virtual COM` での確認用
- 四元数の 3D 表示ビューア

USB 仮想 COM を `ttyACM0` や `COMx` として開くシリアル実装を含みます。

## 対応スクリプト

- `imu_platform2_canfd_test.py`
  - CAN FD の受信確認用
- `imu_platform2_canfd_cube_viewer.py`
  - CAN FD の 3D ビューア
- `imu_platform2_usbserial_test.py`
  - USB serial の受信確認用
- `imu_platform2_usbserial_cube_viewer.py`
  - USB serial の 3D ビューア

## 前提環境

### 共通

- Python 3.10 以降を推奨

### Windows 11

- `USB serial` 版はそのまま実行可能
- `CAN FD` 版は `SocketCAN` 前提のため、そのままでは実行不可

### Jetson Orin Nano / Linux

- `USB serial` 版と `CAN FD` 版の両方を実行可能
- `CAN FD` 版は Linux の `SocketCAN` が必要

## 必要ライブラリ

### pip で入れるもの

```bash
pip install python-can pyserial
```

- `python-can`
  - CAN FD 版で使用
- `pyserial`
  - USB serial 版で使用

### tkinter

3D ビューアを使う場合は `tkinter` が必要です。

#### Windows 11

公式 Python では通常そのまま使えます。

#### Ubuntu / Jetson Linux

```bash
sudo apt install python3-tk
```

## 通信仕様

### CAN FD

- Linux `SocketCAN` を使用
- デフォルトチャネル: `can0`

### USB serial

- 通信条件: `921600 bps`, `8N1`
- ボーレートはコード内で `921600` 固定
- `USB` は仮想 COM として見える
- `UART` と `USB` は同じプロトコルを使用

ポート指定例:

- Windows 11 USB serial: `COM3`
- Jetson USB virtual COM: `/dev/ttyACM0`

## 実行方法

### 1. CAN FD の受信確認

```bash
python imu_platform2_canfd_test.py
```

CAN FD 版は起動時に自動で `ip link` 設定を行います。

```bash
python imu_platform2_canfd_test.py
```

### 2. CAN FD の 3D ビューア

```bash
python imu_platform2_canfd_cube_viewer.py
```

ビューアは通常ユーザーで起動してください。必要な `ip link` 設定だけが内部で `sudo` 実行されます。

```bash
python imu_platform2_canfd_cube_viewer.py
```

`sudo python ...` で起動すると、`tkinter` が X11 / Display に接続できず失敗することがあります。

### 3. USB serial の受信確認

#### Windows 11

```bash
python imu_platform2_usbserial_test.py --port COM3
```

#### Jetson USB virtual COM

```bash
python imu_platform2_usbserial_test.py --port /dev/ttyACM0
```

### 4. USB serial の 3D ビューア

#### Windows 11

```bash
python imu_platform2_usbserial_cube_viewer.py --port COM3
```

#### Jetson USB virtual COM

```bash
python imu_platform2_usbserial_cube_viewer.py --port /dev/ttyACM0
```

## よく使うオプション

### CAN FD 版

- `--channel can0`
  - 使用する SocketCAN チャネル。省略時は `can0`
- `--no-setup-link`
  - 自動の `ip link` 設定を行わず、既存の CAN 設定をそのまま使う
- `--response-timeout 5.0`
  - 応答待ちタイムアウト
- `--response-retries 2`
  - タイムアウト時の再送回数

### USB serial 版

- `--port COM3`
  - 使用する COM / TTY
- `--response-timeout 5.0`
  - 応答待ちタイムアウト
- `--response-retries 2`
  - タイムアウト時の再送回数

### 3D ビューア共通

- `--width 960`
- `--height 720`
- `--fps 60`

## 動作メモ

- 画面クリックで `CMD_RESET_FILTER` を送信します
- 下部に `imu_counter` と四元数を表示します
- USB serial 版は `pyserial` のポート名をそのまま `--port` に渡してください
- 直接 UART 配線の動作は前提にせず、`ttyACM0` や `COMx` の USB serial を使用してください
- `imu_platform2_uart_*.py` は互換のため残していますが、以後は `imu_platform2_usbserial_*.py` を使用してください
- CAN FD 版は Jetson / Linux 前提です
