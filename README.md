# pressure_ros2
## プログラムの説明
detect_circle : 取得した画像から、メーターの中心と周りの四点をマウスクリックし手動でメーターを抽出
image_processing : 抽出した画像から針の部分を抽出
needle_angle : 針の角度を導き、実際のメーター値に変換

## 実行コード
ros2 run pressure pressure image_subscriber
