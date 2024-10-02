import cv2
import numpy as np

# グローバル変数
points = []  # クリックされた点を格納

def resize(img):
    width = 1280
    h, w = img.shape[:2]
    aspect_ratio = h / w
    nh = int(width * aspect_ratio)
    resized_image = cv2.resize(img, (width, nh), interpolation=cv2.INTER_AREA)
    return resized_image

# 2点間の距離を計算する関数
def dist(p1, p2):
    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

# マウスイベント処理関数
def mouseEvents(event, x, y, flags, points_list):
    #左クリックした場合、その座標を保管
    if event == cv2.EVENT_LBUTTONDOWN:
        print(x, y)
        points_list.append([x, y])

def extract_test_piece(img,points_list):
    # 画像を表示してマウスイベントを設定
    cv2.imshow("Select the 4 points", img)
    cv2.setMouseCallback("Select the 4 points", mouseEvents,points_list)
    # 十分なクリックが行われるまで待機
    while len(points_list) < 4:
        key=cv2.waitKey(1)  # 小さな待機時間で処理を継続する
        if key==-1 and cv2.getWindowProperty("Select the 4 points",cv2.WND_PROP_VISIBLE)<1:
            raise ValueError("Closed Window!")

    cv2.destroyAllWindows()

    # クリックした4つの点を取得
    points = np.array(points_list, dtype="float32")
    # print("Selected points:", points)

    # 最も長い辺の長さを計算
    # lengths = [dist(points[i], points[(i+1) % 4]) for i in range(4)]r = atan2(b.y - a.y, b.x - a.x)

    max_length = int(max(lengths))

    # 正方形のターゲット座標を設定
    square = np.array([[0, 0], [max_length, 0], 
                    [max_length, max_length], [0, max_length]], dtype="float32")

    # 射影変換行列を計算
    M = cv2.getPerspectiveTransform(points, square)

    # 変換後の画像サイズ
    output_size = (max_length, max_length)

    # 射影変換を適用
    warped = cv2.warpPerspective(img, M, output_size)
    
    return warped

def main():
    # image_path = "/home/ros/ros2_ws/src/pressure/data/drone/dji_fly_20240925_141130_0013_1727242599714_photo.jpeg"
    image_path = "/home/ros/ros2_ws/src/pressure/data/drone/dji_fly_20240925_141054_0011_1727242600796_photo.jpeg"
    image = cv2.imread(image_path)

    if image is None:
        print("Error: Could not read image")
        return

    image = resize(image)
    points_list = []

    new_img=extract_test_piece(image,points_list)

    cv2.imshow("Warped1",new_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    

if __name__ == "__main__":
    main()
