# pressure_ros2
## プログラムの説明
 - detect_circle : 取得した画像から、メーターの中心と周りの四点をマウスクリックし手動でメーターを抽出
 - image_processing : 抽出した画像から針の部分を抽出
 - needle_angle : 針の角度を導き、実際のメーター値に変換

## 使用方法
 - rosで実行。実験なら
  >python3 func_main.py
 - 画像がでてくるから、中心をクリック→円周を4点適当にクリック
 - 角度調整：中心と12時方向の値をクリック
 - メーターの圧力値の最大値と最小値を選択
 - 検出した線が出力される
 - 適当なコマンド打つと、圧力値が出力される
 - 実行終了

## 実行コード
```bash
ros2 run pressure pressure_subscriber
```
