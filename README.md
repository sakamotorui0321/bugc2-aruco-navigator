# BugC2 メインPC側 ArUcoナビゲータ

上空カメラの映像からArUcoマーカーを検出し、フィールド境界・壁・他機を障害物として地図化し、A*でゴールまでの回避経路を作るPC側プログラムです。

初期状態ではUDP送信を行わない`DRY-RUN`です。まず画面上の認識、機体の向き、赤い進入禁止領域、桃色の経路を確認してください。

## 当日ルールとして確認した設定（2026-08-21）

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
- `config.json`: 自機ID、余白、速度、IPアドレスなど
- `requirements.txt`: Python依存パッケージ
- `tests/test_planner.py`: 経路計画の単体テスト

## Windowsでの準備

PowerShellでこのフォルダへ移動して実行します。

```powershell
py -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

`cv2.aruco`が必要なので、`opencv-python`ではなく`opencv-contrib-python`を使用します。日本語を含むフォルダ名はPython 3とOpenCVの今回の使い方では通常問題ありません。

## 最初の起動（送信なし）

依存ライブラリの確認・不足時のインストール・起動をまとめて行う場合は、次の1コマンドだけです。

```powershell
.\start.bat
```

初回だけライブラリのダウンロードに時間がかかります。2回目以降はそのまま起動します。

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

- `E`: UDP送信の有効／無効を切り替える
- `Space`: STOPを3回送信して送信を無効化する
- `Q`または`Esc`: STOPを送って終了する

送信を有効にする前に、必ず`config.json`の`udp.robot_ip`を実機のIPアドレスへ変更します。

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

1. `DRY-RUN`のままID 0、自己ID、20～23、壁IDが安定検出されるか確認する。
2. 黄色い自機の矢印が実際の機体前方を向くか確認する。違う場合は`heading_offset_deg`を`90`、`-90`、`180`のいずれかへ変更する。
3. 桃色経路が壁や赤い進入禁止領域を横切らないことを確認する。
4. `pixels_per_cm`を実測する。画像上で既知の10 cmが何pxか測り、その値を10で割る。
5. 実機IP、UDP受信、通信断停止をモータを浮かせた状態で確認する。
6. 本番机の上で`pwm_limit=20～25`程度から短時間走行し、必要な最低速度を決める。
7. 問題がなければ`E`で送信を有効化し、広い停止余地を確保して試す。

飲酒中は実機を走らせず、認識画面の確認までにしてください。

## 現在の制限

- 自機IDの初期値は、提供コードに合わせて4です。当日の割り当てに変更が必要です。
- 20→23を画面上の左上→右上→右下→左下に対応する順として正規化しています。実際の配置方向は画面で確認します。
- 障害物の移動予測はまだなく、現在位置を毎フレーム障害物化します。
- PC側UDP形式に対応するM5StickC Plus2側受信スケッチは次段階で作成します。
- BugC2にはエンコーダがないため、機体側で厳密な車輪回転数は制御できません。PWMとジャイロ方位PDを使います。

## 経路計画テスト

OpenCVを入れる前でも、標準ライブラリだけでA*を検証できます。

```powershell
python -m unittest discover -s tests -v
```
