# ichosai_2026_trolley_esp32

**一橋祭 2026 電動トロッコ — ESP32 モータドライバ ファームウェア**

> [!NOTE]
> このリポジトリは ESP32 側のファームウェアのみを扱います。
> 通信プロトコルの詳細、Raspberry Pi 側のソフトウェア、システム全体の構成については
> **メインリポジトリ [zuiken-code/ichosai_2026_trolley](https://github.com/zuiken-code/ichosai_2026_trolley)** を参照してください。

---

## 概要

RS-775 モーター 2 台を VNH5019A-E モータドライバ (Power 基板) 経由で駆動する ESP32 ファームウェアです。
Raspberry Pi から Wi-Fi (UDP) で受け取ったデューティ指令に対し、**スルーレート制限**・**方向反転デッドタイム**・**フェイルセーフ**を適用して安全にモーターを制御します。

### 主な機能

- **デューティサイクル制御** — ±1.0 の範囲で Duty 指令を受け付け
- **スルーレート制限** — 急な指令変化を鈍らせて機械的負荷を軽減
- **方向反転デッドタイム** — duty=0 を経由し、ブリッジの短絡を防止
- **コマンドタイムアウト** — 指令が途切れたら自動で出力を 0 に落とす
- **故障検出** — VNH5019A-E の DIAG ピンによるフォルト検出・自動復帰
- **電流センス** — CS ピンで電流を読み取り、過電流警告を出力
- **シリアル統計** — パケット統計・モーター状態・Wi-Fi 情報を 1 秒周期で出力

---

## 必要条件

### ハードウェア

| 部品 | 型番 / 仕様 |
|------|-------------|
| マイコン | **ESP32-WROOM-32E** (秋月 AE-ESP32-WROOM-32E-MINI) |
| モータドライバ IC | **VNH5019A-E** (DocID15701 Rev 11) |
| モーター | **RS-775** × 2 |
| Wi-Fi ネットワーク | 2.4 GHz (ESP32 は 5 GHz 非対応) |

### ソフトウェア

| 項目 | バージョン |
|------|-----------|
| Arduino IDE | 2.x 推奨 |
| ESP32 Arduino Core | **3.x** (`ledcAttach()` API を使用) |
| ボード設定 | `ESP32 Dev Module` |

> [!WARNING]
> ESP32 Arduino Core **2.x** では `ledcSetup()` / `ledcAttachPin()` API のため、コンパイルエラーになります。
> 必ず **Core 3.x** を使用してください。

### Arduino IDE ボードマネージャの追加

1. **ファイル → 基本設定 → 追加のボードマネージャのURL** に以下を追加:
   ```
   https://espressif.github.io/arduino-esp32/package_esp32_index.json
   ```
2. **ツール → ボード → ボードマネージャ** から `esp32 by Espressif Systems` をインストール (バージョン 3.x)
3. **ツール → ボード** で `ESP32 Dev Module` を選択

---

## `secrets.h` の作成

Wi-Fi の資格情報は `secrets.h` に記述します。このファイルは `.gitignore` に含まれているため、**各自で作成する必要があります**。

### 最低限の構成 (DHCP)

```cpp
#pragma once

// Wi-Fi
const char* WIFI_SSID     = "your-ssid";
const char* WIFI_PASSWORD = "your-password";
```

### 固定 IP を使う場合

```cpp
#pragma once

#include <WiFi.h>

// Wi-Fi
const char* WIFI_SSID     = "your-ssid";
const char* WIFI_PASSWORD = "your-password";

// 固定 IP
#define USE_STATIC_IP

IPAddress WIFI_LOCAL_IP(192, 168, 1, 100);
IPAddress WIFI_GATEWAY (192, 168, 1, 1);
IPAddress WIFI_SUBNET  (255, 255, 255, 0);
```

> [!IMPORTANT]
> `USE_STATIC_IP` を `#define` しない場合は DHCP が使用されます。
> 固定 IP を使う場合は、Pi 側の送信先アドレスと一致させてください。

---

## 回路 — ESP32 ↔ VNH5019A-E 接続

### ピンアサイン

| 機能 | Motor 1 (GPIO) | Motor 2 (GPIO) | VNH5019A-E ピン |
|------|:-:|:-:|------|
| INA | **25** | **18** | INA |
| INB | **26** | **19** | INB |
| PWM | **27** | **23** | PWM |
| EN/DIAG | **13** | **14** | ENA/DIAGA + ENB/DIAGB (共通接続) |
| CS (電流センス) | **34** (ADC1) | **35** (ADC1) | CS |

### GPIO 選定の根拠

- **ADC2 は使用不可** — Wi-Fi 動作中に ADC2 (GPIO 0/2/4/12/13/14/15/25/26/27) の ADC 機能は利用できないため、電流センス (CS) は **ADC1 (GPIO 32–36, 39)** から取る
- **ストラッピングピン回避** — GPIO 0/2/5/12/15 は起動時のプルダウンと衝突するため、入力用途では使わない
- **EN/DIAG** — ENA/DIAGA と ENB/DIAGB を共通接続し、GPIO の OUTPUT HIGH で有効化。ドライバが FAULT 時に LOW に引き下げるので、`digitalRead()` でフォルト検出

### 電流センスの計算

```
VNH5019A-E: ISENSE = IOUT / K     (K typ = 7000)
Power基板:  R9 = 1kΩ → VSENSE = IOUT / 7000 × 1000 = 0.1429 V/A
ESP32側:    R8 = 10kΩ + GND側 8.2kΩ 分圧
            分圧比 = 8200 / 18200 = 0.4505
            ADC端感度 = 64.4 mV/A
```

> [!NOTE]
> この係数は校正前の計算値です。PWM の Lo 期間は CS がハイインピーダンスになるため、duty 30% 未満の読み値は電流値として信頼できません。

### 回路ブロック図

```
                            ┌────────────────────────────┐
                            │      VNH5019A-E (×2)       │
  ┌──────────────┐          │                            │
  │   ESP32      │          │  ┌──────┐     ┌────────┐   │
  │              │──INA────▶│  │      │     │        │   │
  │              │──INB────▶│  │ Gate │────▶│ RS-775 │   │
  │              │──PWM────▶│  │Driver│     │ Motor  │   │
  │              │◀─CS──────│  │      │     │        │   │
  │              │──EN─────▶│  └──────┘     └────────┘   │
  │              │◀─DIAG────│  (EN/DIAG は共通接続)       │
  └──────────────┘          └────────────────────────────┘
       │  │
     UART  Wi-Fi (UDP)
       │     │
    Serial  Raspberry Pi
```

---

## モーター制御の詳細

### 制御ループ

制御タスクは **Core 1** に固定され、**1 kHz** (1 ms 周期) で実行されます。
Wi-Fi スタックは Core 0 で動作するため、制御ループとは干渉しません。

```
Core 0: loop()          — UDP パケット受信 + 統計表示
Core 1: controlTask()   — 1 kHz モーター制御ループ
```

### スルーレート制限

急な指令変化による機械的・電気的ショックを防ぐため、duty の変化速度を制限します。

| パラメータ | 値 | 意味 |
|-----------|-----|------|
| `SLEW_UP` | 2.0 duty/s | 加速時: 0→100% に **0.5 秒** |
| `SLEW_DOWN` | 4.0 duty/s | 減速時: 100→0% に **0.25 秒** |
| `SLEW_FAILSAFE` | 4.0 duty/s | タイムアウト/停止時: 100→0% に **0.25 秒** |

### 方向反転シーケンス

H ブリッジの短絡 (shoot-through) を防ぐため、方向を変える際は必ず以下の手順を踏みます:

1. スルーレートに従って duty を 0 まで下げる
2. INA/INB を両方 LOW (GND ブレーキ)
3. **20 ms のデッドタイム**を待つ
4. 新しい方向の INA/INB をセット
5. スルーレートに従って duty を上げていく

### VNH5019A-E 真理値表

| INA | INB | PWM | 動作 |
|:---:|:---:|:---:|------|
| 1 | 0 | duty | 正転 |
| 0 | 1 | duty | 逆転 |
| 0 | 0 | — | GND ブレーキ |
| — | — | Lo | コースト (ハイインピーダンス) |

### フェイルセーフ

| 条件 | 動作 |
|------|------|
| UDP 指令が **150 ms** 途切れる | duty を SLEW_FAILSAFE で 0 に落とす |
| DISABLE 指令を受信 | duty を 0 に落とす |
| EN/DIAG ピンが LOW (ドライバ FAULT) | EN を LOW → **250 ms** 後に復帰 |

### PWM 設定

| パラメータ | 値 |
|-----------|-----|
| 周波数 | 16 kHz |
| 分解能 | 11 bit (0–2047) |

> VNH5019A-E の PWM 入力上限は 20 kHz。マージンを取って 16 kHz としています。

---

## テストスケッチ

[test/test.ino](test/test.ino) は Wi-Fi を使わず、ハードウェアの動作確認を行うためのスケッチです。

- Motor 1 正転 → 停止 → 逆転 → 停止
- Motor 2 正転 → 停止 → 逆転 → 停止
- 両方同時に正転 → 停止 → 逆転 → 停止

デフォルトでは **20% duty** / **1 秒間** 回転します。`secrets.h` は不要です。

---

## ビルド & 書き込み

1. `secrets.h` を作成する（[上記参照](#secretsh-の作成)）
2. Arduino IDE で `ESP32.ino` を開く
3. ボードを `ESP32 Dev Module` に設定
4. シリアルポートを選択
5. **書き込み** ボタンをクリック
6. シリアルモニタ (115200 baud) で `Ready` が表示されれば起動完了

---

## シリアル出力の読み方

起動後、1 秒ごとに以下のような統計がシリアルモニタに出力されます:

```
========== STATUS ==========
rx  total=1234 valid=1230 invalid=4
err size=0 header=1 crc=3 id=0 mode=0
M1  pkt=615 stale=0 dir=+1 duty=0.500 req=+0.500 fault=0 I=5.2A oc=0
M2  pkt=615 stale=0 dir=+1 duty=0.500 req=+0.500 fault=0 I=4.8A oc=0
wifi status=3 rssi=-45
============================
```

| フィールド | 意味 |
|-----------|------|
| `total` / `valid` / `invalid` | 受信パケット数 |
| `size` / `header` / `crc` / `id` / `mode` | エラーの内訳 |
| `dir` | 回転方向 (+1=正転, -1=逆転, 0=停止) |
| `duty` | 現在の実 duty |
| `req` | 受信した指令 duty |
| `fault` | フォルト発生回数 |
| `I=` | 電流値 (`~` 付きは信頼度低) |
| `oc` | 過電流警告回数 |
| `TIMEOUT` | 指令タイムアウト中 |

---

## ライセンス

メインリポジトリ [zuiken-code/ichosai_2026_trolley](https://github.com/zuiken-code/ichosai_2026_trolley) に準じます。
