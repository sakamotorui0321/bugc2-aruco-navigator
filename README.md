# BugC2 メインPC側 ArUcoナビゲータ

上空カメラの映像からArUcoマーカーを検出し、フィールド境界・壁・他機を障害物として地図化し、A*でゴールまでの回避経路を作る本番用PC側プログラムです。

起動直後は必ず`WAIT A`となり、UDP移動指令を送信しません。画面上の認識、機体の向き、赤い進入禁止領域、桃色の経路を確認し、OpenCV映像ウィンドウを選択して`A`を押すと制御を開始します。`Space`で即時停止します。

## [公式ページ](https://sites.google.com/view/cq-ws/)で確認した当日設定（2026-08-21）

- BugC2: `CQ-WS-24Gnew` / PASS `00000000`
- メインPC推奨: `CQ-WS-5Gnew` / PASS `00000000`
- 映像: `http://192.168.100.24/video_feed`
- ID 0: ゴール
- ID 1～9: プレイヤー
- ID 10～19: お邪魔ロボット
- ID 20～23: フィールド四隅（時計回り）
- ID 30以降: 若い偶数と次の奇数を結んだ壁

SSIDや映像サーバは当日変更される可能性があります。`config.json`を正として、現場の案内に合わせて変更します。

## ファイル

- `main.py`: 映像取得、ArUco認識、地図、A*、表示、UDP送信、CSVログ
- `planner.py`: OpenCV非依存のA*経路計画
- `send_test_command.py`: カメラなしで機体側の基礎走行を短時間確認する安全制限付きUDP送信
- `config.json`: 自機ID、余白、速度、IPアドレスなど
- `requirements.txt`: Python依存パッケージ
- `tests/test_planner.py`: 経路計画の単体テスト
- `onboard/13_WiFi_UDP_Gyro_PD_Controller/13_WiFi_UDP_Gyro_PD_Controller.ino`: M5StickC Plus2へ書き込むオンボード側プログラム

## Windows Terminalから本番起動（コピペ用）

Windows TerminalのPowerShellを開き、次をそのまま上から順に貼り付けます。最初の`cd ~`は、現在どのフォルダにいても同じ手順で開始するためのものです。

```powershell
cd ~
cd "C:\Users\81703\OneDrive - 学校法人 日本工業大学 (1)\平栗研究室\研究発表会用資料\2026_08_CQWS伊豆高原\メインPCプログラム"
.\start.bat
```

初回は仮想環境の作成とライブラリのダウンロードを自動実行するため時間がかかります。2回目以降も同じ3行で起動できます。実際のフォルダ名は`2026_08_CQWS伊豆高原`です。Markdown上の装飾回避に使う`\_`をWindows Terminalへ入力しないでください。

## 本番ネットワークと起動順序

メインPCと実機は異なる周波数帯のSSIDを使いますが、競技会場の同一ネットワーク内でIPユニキャストUDP通信します。

- メインPC: 5 GHzの`CQ-WS-5Gnew`へ接続する
- M5StickC Plus2: オンボードプログラムが2.4 GHzの`CQ-WS-24Gnew`へ接続する
- 共通パスワード: `00000000`
- カメラ映像: `http://192.168.100.24/video_feed`
- PC→実機: UDP、初期ポート`5005`

本番の起動手順:

1. 実機を安全な場所に置いて電源を入れる。車輪が接地している場合は周囲に停止余地を確保する。
2. M5StickC Plus2の画面に出た`IP:`の値を確認する。
3. `config.json`の`udp.robot_ip`を、その実機IPへ変更して保存する。`udp.robot_port`はオンボード側と同じ`5005`にする。
4. 実機を静止させたままBtnAを押し、ジャイロ校正後に画面が`UDP ARMED`となることを確認する。実機のBtnAは、ARM後は緊急停止ボタンになる。
5. メインPCを`CQ-WS-5Gnew`へ接続し、上記3行のコマンドでプログラムを起動する。
6. ID 0、自機、フィールド四隅、障害物、桃色の経路が正しく表示されるまで待つ。
7. OpenCV映像ウィンドウをクリックしてから、キーボードの`A`を1回押す。条件がそろっていればPCから実機への移動指令が始まる。
8. 停止は`Space`、終了は`Q`または`Esc`を使う。異常時は実機のBtnAも使える。

`A`を押した時点で自機・ゴール・四隅・安全経路がそろっていない場合は始動を拒否し、理由をWindows Terminalへ表示します。Windowsファイアウォールの確認が表示された場合は、会場ネットワークの扱いを確認した上でPythonのプライベートネットワーク通信を許可します。

## Windowsでの準備

PowerShellでこのフォルダへ移動して実行します。

```powershell
py -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

`cv2.aruco`が必要なので、`opencv-python`ではなく`opencv-contrib-python`を使用します。日本語を含むフォルダ名はPython 3とOpenCVの今回の使い方では通常問題ありません。

## 起動バッチの動作

依存ライブラリの確認・不足時のインストール・起動をまとめて行う場合は、次の1コマンドだけです。

```powershell
.\start.bat
```

初回だけライブラリのダウンロードに時間がかかります。2回目以降はそのまま起動します。起動後は`WAIT A`で待機するため、プログラムを立ち上げただけでモータが動くことはありません。

手動で起動する場合:

```powershell
python main.py
```

別の自機IDを一時指定する場合:

```powershell
python main.py --self-id 3
```

動画ファイルやUSBカメラで試す場合:

```powershell
python main.py --source C:\path\field_test.mp4
python main.py --source 0
```

## 画面操作

- `A`: 認識と経路の安全条件が成立している場合だけ、UDP移動指令を開始する
- `Space`: STOPを3回送信して制御を停止・再ロックする
- `Q`または`Esc`: STOPを送って終了する

`Space`を押した後に再開する場合も、改めて`A`を押します。送信を開始する前に、必ず`config.json`の`udp.robot_ip`を実機のIPアドレスへ変更します。

## オンボードプログラムをArduino IDEで開く・保存する

実機へ書き込むファイルは次です。

```text
C:\Users\81703\OneDrive - 学校法人 日本工業大学 (1)\平栗研究室\研究発表会用資料\2026_08_CQWS伊豆高原\メインPCプログラム\onboard\13_WiFi_UDP_Gyro_PD_Controller\13_WiFi_UDP_Gyro_PD_Controller.ino
```

Arduino IDE 2.3.10での手順:

1. Arduino IDEで`ファイル`→`開く...`を選び、上記`.ino`を開く。
2. `ツール`→`ボード`→`M5Stack`→`M5StickCPlus2`を選ぶ。
3. 使用中のM5Stackボードパッケージ3.3.8、M5Unified 0.2.20、M5Hat-BugCが導入済みであることを確認する。`WiFi`と`WiFiUDP`はESP32ボードパッケージに含まれる。
4. USB接続した実機のポートを選ぶ。安全のため最初は車輪を浮かせて`書き込み`を行う。
5. 必要ならシリアルモニタを115200 baudで開く。

Arduinoスケッチは、フォルダ名と`.ino`のベース名を同じ`13_WiFi_UDP_Gyro_PD_Controller`にする必要があります。別の場所へ保存する場合は、`ファイル`→`名前を付けて保存...`を使い、Arduino IDEが作成する同名フォルダ内へ保存してください。元ファイルを残したまま実験版を作る場合は、フォルダ名と`.ino`名を同じ新しい名前にして保存します。

## カメラなしの機体側走行テスト

オンボード側の`13_WiFi_UDP_Gyro_PD_Controller`を書き込み、画面に表示されたIPを`config.json`の`udp.robot_ip`へ設定します。まず車輪を浮かせ、送信しないプレビューを確認します。

```powershell
.\.venv\Scripts\python.exe send_test_command.py forward
```

実際に1秒だけ低速前進指令を送る場合:

```powershell
.\.venv\Scripts\python.exe send_test_command.py forward --duration 1 --level 0.25 --pwm-limit 22 --execute
```

`forward / backward / left / right / cw / ccw / stop`を選べます。移動時間は最大3秒、指令値は最大0.5、PWM上限は28に制限され、終了時やCtrl+C時にはSTOPを5回送ります。このテストは`velocity_local`モードを使うため、カメラがなくても機体側ジャイロで開始方位を維持します。

## 安全条件

次の場合、移動指令は自動的にSTOPになります。

- 自機マーカーが見つからない
- ゴールマーカーが見つからない
- フィールド四隅20～23のいずれかが見つからない
- 安全な経路が見つからない
- ゴール半径内へ到達した
- 映像が途切れた
- 他プレイヤーまたはお邪魔ロボットが`other_robot_stop_distance_cm`以内へ接近した

機体側にも独立したUDPウォッチドッグが必要です。受信から`ttl_ms`（初期値350 ms）を超えたら、PC側の状態に関係なく全モータを停止させます。

## UDP指令仕様 Version 1

UTF-8 JSONをUDPで送信します。初期ポートは5005です。

```json
{
  "v": 1,
  "type": "motion",
  "seq": 123,
  "sent_ms": 1787300000000,
  "ttl_ms": 350,
  "mode": "velocity",
  "forward": 0.35,
  "lateral": 0.0,
  "turn": -0.18,
  "heading_error_deg": -10.0,
  "target_heading_deg": 42.0,
  "pwm_limit": 28,
  "reason": "follow_path"
}
```

値の意味:

- `forward`: 前進を正とする正規化指令（-1～+1）
- `lateral`: 右移動を正とする正規化指令（初版では0固定）
- `turn`: 反時計回りを正とする正規化旋回指令（実機で符号確認する）
- `heading_error_deg`: PC画像で求めた目標方位との差
- `pwm_limit`: 機体側で許可する最大PWM
- `ttl_ms`: この指令を有効とみなせる最大時間

初版は狭いフィールド向けに、方位誤差が20°を超える間は前進せず、目標方向を向いてから低速前進します。横移動は、前後・左右それぞれの実機安定性を確認してから有効化します。

障害物を避ける幅は`config.json`の`robot_width_cm`と`safety_margin_cm`で変更します。A*は直進可能なら直線を選び、壁が塞いでいれば壁の左右を含む全候補から最短の安全経路を選びます。表示上は灰色が自機からゴールへの直線候補、暗い桃色が採用経路の機体幅、明るい桃色が採用経路の中心です。赤い領域は安全余白込みの進入禁止領域です。

他機が近づいた場合は、意図的な接触や攻撃を行わず、動的障害物として再計画します。さらに`other_robot_stop_distance_cm`以内ではSTOPします。

## 本番フィールドで最初に確認する順番

1. `WAIT A`のままID 0、自己ID、20～23、壁IDが安定検出されるか確認する。
2. 黄色い自機の矢印が実際の機体前方を向くか確認する。違う場合は`heading_offset_deg`を`90`、`-90`、`180`のいずれかへ変更する。
3. 桃色経路が壁や赤い進入禁止領域を横切らないことを確認する。
4. `pixels_per_cm`を実測する。画像上で既知の10 cmが何pxか測り、その値を10で割る。
5. 実機IP、UDP受信、通信断停止をモータを浮かせた状態で確認する。
6. 本番机の上で`pwm_limit=20～25`程度から短時間走行し、必要な最低速度を決める。
7. 問題がなければOpenCV映像ウィンドウで`A`を押し、広い停止余地を確保して試す。

飲酒中は実機を走らせず、認識画面の確認までにしてください。

## 現在の制限

- 自機IDの初期値は、提供コードに合わせて4です。当日の割り当てに変更が必要です。
- 20→23を画面上の左上→右上→右下→左下に対応する順として正規化しています。実際の配置方向は画面で確認します。
- 障害物の移動予測はまだなく、現在位置を毎フレーム障害物化します。
- オンボード側は車輪エンコーダを持たないため、目標座標そのものではなく、PCから受けた速度・旋回指令とジャイロ方位PDを実行します。絶対位置補正と再経路計画はPC側が担当します。
- BugC2にはエンコーダがないため、機体側で厳密な車輪回転数は制御できません。PWMとジャイロ方位PDを使います。

## 経路計画テスト

OpenCVを入れる前でも、標準ライブラリだけでA*を検証できます。

```powershell
python -m unittest discover -s tests -v
```
