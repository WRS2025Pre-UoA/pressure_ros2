import cv2
import numpy as np
import matplotlib.pyplot as plt

# グローバル変数
# clicks = []  # クリックした座標を格納するリスト


# 2点間の距離を計算する関数
def dist(p1, p2):
    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

# マウスクリックのイベントを定義
def on_mouse_click(event, x, y, flags, clicks):
    if event == cv2.EVENT_LBUTTONDOWN:
        clicks.append([x, y])  # クリックした座標をリストに追加
        print(f"Clicked at: {x}, {y}")
        
def process_circle(img,clicks):
    cv2.imshow('Click center and Four points around the meter', img)
    cv2.setMouseCallback('Click center and Four points around the meter', on_mouse_click,clicks)
    while len(clicks) < 5:
        # cv2.imshow('Click center and Four points around the meter', img)
        # cv2.setMouseCallback('Click center and Four points around the meter', on_mouse_click,clicks)
    # 画像を表示し続ける
        key=cv2.waitKey(1) 
        # print(key)
        # print(cv2.getWindowProperty("Click center and Four points around the meter",cv2.WND_PROP_VISIBLE))
        # if key != -1 :#or cv2.getWindowProperty("Click center and Four points around the meter",cv2.WND_PROP_VISIBLE)==0:
        #     cv2.destroyAllWindows()
        #     # raise ValueError("Closed Window!")
        #     return None
    cv2.destroyAllWindows()

    center = clicks[0]  # 最初の点を中心とする
    corners = clicks[1:]  # 残りの4点を四隅とする

    # clicks=[]
    
    # 各隅と中心の距離を計算
    distances = [np.sqrt((center[0] - corner[0]) ** 2 + (center[1] - corner[1]) ** 2) for corner in corners]
    
    # 最大距離を取得
    max_distance = int(max(distances))
    
    print(f"Center: {center}, Max Radius: {max_distance}")

    # 円を描画するためのマスクを作成
    mask = np.zeros_like(img)
    cv2.circle(mask, center, max_distance, (255, 255, 255), -1)  # マスクに円を描く
    
    # マスクを使って円形に画像を切り抜く
    cut_out = cv2.bitwise_and(img, mask)
    
    

    # cv2.imwrite("test.png",background)       
        # 円の部分だけを切り取る
    x, y = center
    x, y = int(x), int(y)
    cut_out = cut_out[y-max_distance:y+max_distance, x-max_distance:x+max_distance]
    
    points = []
    cv2.imshow('Click on the centre and middle values', cut_out)
    cv2.setMouseCallback('Click on the centre and middle values', on_mouse_click,points)
    while len(points) < 2:
    # 画像を表示し続ける
        key=cv2.waitKey(1)
        if key==-1 and cv2.getWindowProperty("Click on the centre and middle values",cv2.WND_PROP_VISIBLE)<1:
            raise ValueError("Closed Window!")
    cv2.destroyAllWindows()
    # print(points)
    # P1とP2を使って角度を計算
    P2 = points[1]
    P1 = points[0]
    delta_x = P2[0] - P1[0]
    delta_y = P2[1] - P1[1]
    angle = np.arctan2(delta_y, delta_x) * (180 / np.pi)+90  # ラジアンから度に変換

    print(angle)
    # 画像の中心を計算
    (h, w) = cut_out.shape[:2]
    center = (w // 2, h // 2)

    # 回転行列を作成
    M = cv2.getRotationMatrix2D(center, angle, 1.0)  # スケールは1.0

    # 画像を回転
    rotated_image = cv2.warpAffine(cut_out, M, (w, h))
    # cv2.imshow("1",rotated_image)
    # cv2.waitKey(0)
    # 背景画像のサイズを決定
    background_width = rotated_image.shape[1] + 200
    background_height = rotated_image.shape[0] + 200

    # 背景画像を作成
    background = np.ones((background_height, background_width, 3), dtype=np.uint8) * 0

    # 中心に配置するためのオフセットを計算
    start_x = (background_width - rotated_image.shape[1]) // 2
    start_y = (background_height - rotated_image.shape[0]) // 2

    # 背景に円形の画像を合成
    background[start_y:start_y + rotated_image.shape[0], start_x:start_x + rotated_image.shape[1]] = rotated_image

    # 画像の表示
    # cv2.imshow("Cropped Image", cut_out)
    # cv2.imshow("Background with Cropped Circle", background)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    # cv2.imwrite("cut_circle.png",background) 
    # h1,w1 = background.shape[:2]

    

    return background

def affine(img):
    cv2.imshow('Original Image', img)
    cv2.setMouseCallback('Original Image', on_mouse_click)
    while len(clicks) < 4:
    # 画像を表示し続ける
        key = cv2.waitKey(1)
        if key==-1 and cv2.getWindowProperty("Select the 4 points",cv2.WND_PROP_VISIBLE)<1:
            raise ValueError("Closed Window!")
    cv2.destroyAllWindows()

    # クリックした4つの点を取得
    points = np.array(clicks, dtype="float32")
    # print("Selected points:", points)

    # 最も長い辺の長さを計算
    lengths = [dist(points[i], points[(i+1) % 4]) for i in range(4)]
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
    # 画像を読み込む
    # img = cv2.imread('/home/ros/ros2_ws/src/pressure/data/sample/LINE_ALBUM_パシャリーズ_240726_5.jpg')  # 画像パスを指定
    # img = cv2.imread("/home/ros/ros2_ws/src/pressure/data/cropped_images/LINE_ALBUM_パシャリーズ_240726_20.jpg")
    #img = cv2.imread("/home/ros/ros2_ws/src/pressure/data/Origin_data/LINE_ALBUM_20240711_240730_200.jpg")
    img = cv2.imread("/home/ros/ros2_ws/src/pressure/data/Origin_data/LINE_ALBUM_20240711_240730_74.jpg")
    # img = cv2.imread("/home/ros/ros2_ws/src/pressure/data/Origin_data/LINE_ALBUM_野坂_240730_2.jpg")
    # ウィンドウを作成し、マウスクリックイベントを設定
    h,w=img.shape[:2]

    # cv2.imshow('Original Image', img)
    # cv2.setMouseCallback('Original Image', on_mouse_click)
    # while len(clicks) < 5:
    # # 画像を表示し続ける
    #     cv2.waitKey(1)
    # cv2.destroyAllWindows()

    # img1 = process_circle(img)
    img1 = affine(img)

    cv2.imshow("result",img1)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()