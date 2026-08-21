# BugC2 メインPC側 ArUcoナビゲータ

上空カメラの映像からArUcoマーカーを検出し、フィールド境界・壁・他機を障害物として地図化し、A*でゴールまでの回避経路を作る本番用PC側プログラムです。

起動直後は必ず`WAIT A`となり、UDP移動指令を送信しません。画面上の認識、機体の向き、赤い進入禁止領域、桃色の経路、実機の`ROBOT CONNECTED`を確認し、Windows Terminalで`a`を1文字入力すると制御を開始します。Enterは不要です。`Space`で即時停止します。

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

希望する本番の起動順序:

1. メインPCを5 GHzの`CQ-WS-5Gnew`へ接続する。
2. 上記3行のコマンドでPCプログラムを起動し、`WAIT A`で待機させる。SSIDが違う場合はカメラ接続を開始せず、正しいSSIDへ切り替わるまで待つ。
3. 実機を安全な場所で静止させ、電源を入れる。起動時に約1.5秒のジャイロ校正を自動実行し、成功すると`UDP ARMED`で停止待機する。
4. PCが停止状態の探索パケットを送り続け、2.4 GHzの`CQ-WS-24Gnew`へ接続した同じ`robot_id`の実機が状態を返信する。
5. Windows Terminalに`ROBOT CONNECTED: ARMED ...`が表示され、映像画面の`ROBOT:`も緑色になることを確認する。
6. ID 0、自機、フィールド四隅、障害物、桃色の経路が正しく表示されるまで待つ。
7. Windows Terminalを選択し、小文字`a`を1文字入力する。Enterは不要。すべての条件が成立していれば移動指令が始まる。
8. 停止はTerminalまたは映像ウィンドウの`Space`、終了は`Q`を使う。異常時は実機のBtnAも使える。

`config.json`の`self_id`と`udp.robot_id`、オンボード`.ino`先頭の`robotId`は同じ値にします。初期値はすべて`4`です。`udp.robot_ip`の初期値`auto`では、`192.168.100.255`と`255.255.255.255`へ探索し、IDが一致する実機からの返信元IPを自動採用します。

`a`を入力した時点で実機応答・実機ARM・自機・ゴール・四隅・安全経路のいずれかが不足している場合は始動を拒否し、理由をWindows Terminalへ表示します。Windowsファイアウォールの確認が表示された場合は、会場ネットワークの扱いを確認した上でPythonのプライベートネットワーク通信を許可します。

### 接続できない場合

`[tcp ...] Connection to 192.168.100.24:80 failed`は実機UDPではなく、カメラ映像HTTPへの接続失敗です。まずWindows Terminalで次を確認します。

```powershell
netsh wlan show interfaces
Test-NetConnection 192.168.100.24 -Port 80
```

SSIDが`CQ-WS-5Gnew`でなければ、`192.168.100.24`へは通常到達できません。カメラが見えるのに`ROBOT CONNECTED`にならない場合は次を確認します。

1. 実機画面が`WiFi: OK`、`UDP ARMED`で、IPが表示されているか確認する。
2. PC側、オンボード側、ArUcoマーカーのrobot IDが一致しているか確認する。
3. 自動探索が会場APで遮断される場合は、`config.json`の`udp.robot_ip`を実機画面のIPへ変更して再起動する。以後はユニキャストだけを使う。
4. 正しいIPを指定しても応答がない場合は、5 GHzと2.4 GHz間の端末間通信がAP側で禁止されていないか主催者へ確認する。APのクライアント分離はPCや実機のプログラムだけでは解除できない。

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

この指定はPC側の`udp.robot_id`も同時に3へ変更します。オンボード`.ino`の`robotId`も3にして書き込んだ実機だけが応答します。

動画ファイルやUSBカメラで試す場合:

```powershell
python main.py --source C:\path\field_test.mp4
python main.py --source 0
```

## 画面操作

- Windows Terminalの`a`または`A`: 実機応答、ARM、認識、経路の全条件が成立している場合だけUDP移動指令を開始する。Enterは不要
- OpenCV映像ウィンドウの`A`: Terminalと同じ始動操作。予備操作として残している
- `Space`: STOPを3回送信して制御を停止・再ロックする
- `Q`または`Esc`: STOPを送って終了する

`Space`を押した後に再開する場合も、改めて`a`を入力します。自動探索を使う場合、実機IPの手入力は不要です。

## カメラ映像とArUco認識結果

起動すると、カメラ映像と画像認識結果を`BugC2 ArUco Navigator`という1つのサイズ変更可能なウィンドウへ表示します。生映像を別ウィンドウでもう一度処理するのではなく、経路計画に使用した同じフレームへ結果を重ねるため、表示のためにカメラへ二重接続することはありません。

同一画面に表示する内容:

- 検出した全ArUcoマーカーの外枠、中心点、`ID`、向きの矢印
- 画面上部の検出数と`ARUCO IDs`一覧
- 紫色のフィールド外周
- 赤色半透明の進入禁止領域
- 灰色のゴール直線候補
- 桃色の採用経路と機体幅
- 現在の送信状態、FPS、移動指令、安全停止理由

映像が表示されていても起動直後は`WAIT A`なので、モータ指令は始まりません。認識結果と`ROBOT CONNECTED`を確認後、Windows Terminalで`a`を入力します。

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

電源投入直後に自動校正するため、実機を約1.5秒動かさないでください。校正失敗またはBtnA緊急停止後は`UDP SAFE`となり、自動再ARMしません。静止させてBtnAを押すと再校正・ARMします。

## カメラなしの機体側走行テスト

オンボード側の`13_WiFi_UDP_Gyro_PD_Controller`を書き込みます。まず車輪を浮かせ、実機画面のIP、`WiFi: OK`、`UDP ARMED`を確認します。

```powershell
.\.venv\Scripts\python.exe send_test_command.py forward
```

実際に1秒だけ低速前進指令を送る場合:

```powershell
.\.venv\Scripts\python.exe send_test_command.py forward --robot-ip 192.168.100.X --robot-id 4 --duration 1 --level 0.25 --pwm-limit 22 --execute
```

`192.168.100.X`は実機画面のIPへ置き換えます。`forward / backward / left / right / cw / ccw / stop`を選べます。移動時間は最大3秒、指令値は最大0.5、PWM上限は28に制限され、終了時やCtrl+C時にはSTOPを5回送ります。このテストは`velocity_local`モードを使うため、カメラがなくても機体側ジャイロで開始方位を維持します。

## 安全条件

次の場合、移動指令は自動的にSTOPになります。

- 自機マーカーが見つからない
- ゴールマーカーが見つからない
- フィールド四隅20～23のいずれかが見つからない
- 安全な経路が見つからない
- ゴール半径内へ到達した
- 映像が途切れた
- 実機からのUDP状態返信が1.5秒以上途切れた
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
  "robot_id": 4,
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
- `robot_id`: 指令対象のArUco自機ID。異なるID宛ての指令は実機側で無視する
- `lateral`: 右移動を正とする正規化指令（初版では0固定）
- `turn`: 反時計回りを正とする正規化旋回指令（実機で符号確認する）
- `heading_error_deg`: PC画像で求めた目標方位との差
- `pwm_limit`: 機体側で許可する最大PWM
- `ttl_ms`: この指令を有効とみなせる最大時間

PCは制御開始前も`type: "probe"`を2 Hzで送り、実機は`type: "status"`で`robot_id`、IP、ARM、Wi-Fi、UDP、制御状態を返信します。PCは一致する`robot_id`の返信を1.5秒以内に受けている場合だけ`a`始動を許可します。

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
7. 問題がなければWindows Terminalで`a`を入力し、広い停止余地を確保して試す。

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
