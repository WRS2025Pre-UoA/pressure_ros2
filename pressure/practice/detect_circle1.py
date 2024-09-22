import cv2
import numpy as np

# 円の画像を読み込む
circle_img = cv2.imread('/home/ros/ros2_ws/src/pressure/pressure/cut_circle.png')
if circle_img is None:
    print("Failed to load circle image")
    exit()

# 背景サイズを設定 (W=640, H=480)
background_width = 640
background_height = 480

# 背景を白に設定
background = np.ones((background_height, background_width, 3), dtype=np.uint8) * 255

# リサイズする円の直径を640ピクセルに設定 (半径320)
target_diameter = 320

# 元の円の画像を直径640にリサイズ
resized_circle_img = cv2.resize(circle_img, (target_diameter, target_diameter), interpolation=cv2.INTER_LINEAR)

# リサイズ後の画像の高さと幅を取得
rh, rw = resized_circle_img.shape[:2]

# 中心に配置するためのオフセットを計算
center_x = background_width // 2
center_y = background_height // 2
start_x = center_x - rw // 2
start_y = center_y - rh // 2

# 円を背景に配置 (画像をクロップして背景に収める)
end_x = start_x + rw
end_y = start_y + rh

# 背景のサイズ範囲に収まるように、描画する部分をクロップ
if start_x < 0:
    resized_circle_img = resized_circle_img[:, -start_x:]
    start_x = 0
if start_y < 0:
    resized_circle_img = resized_circle_img[-start_y:, :]
    start_y = 0
if end_x > background_width:
    resized_circle_img = resized_circle_img[:, :(background_width - start_x)]
if end_y > background_height:
    resized_circle_img = resized_circle_img[:(background_height - start_y), :]

# 背景にリサイズされた円の画像を合成
background[start_y:start_y + resized_circle_img.shape[0], start_x:start_x + resized_circle_img.shape[1]] = resized_circle_img

# 結果を表示
cv2.imshow('Circle on Background', background)
cv2.waitKey(0)
cv2.destroyAllWindows()

cv2.imwrite("test1.png",background)
