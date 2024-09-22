from image_processing import process_image
from needle_angle import calculate_angle
from needle_angle import pressure_value_from_angle
from detect_circle import process_circle
import cv2
import numpy as np
#pressure_value_from_angle

point=[]
# マウスクリックのイベントを定義
def on_mouse_click(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        point.append((x, y))  # クリックした座標をリストに追加

def initialize():
    # 真っ白な背景 (650x400)
    img = np.ones((400, 650, 3), dtype=np.uint8) * 255

    # 長方形の大きさ (150x100)
    rect_width = 150
    rect_height = 100

    # 黒い枠線で描く長方形の左上座標
    positions = [(50, 100), (250, 100), (450, 100)]

    # 各長方形に表示するテキスト
    texts = ["1MPa", "0.25MPa", "1.6MPa"]

    # 描画する枠線の色と太さ
    color = (0, 0, 0)  # 黒
    thickness = 2

    # フォント、スケール、色、太さ
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 1
    font_color = (0, 0, 255)  # 赤
    font_thickness = 2

    # 各長方形を描画し、中央にテキストを配置
    for i, pos in enumerate(positions):
        # 長方形の右下座標を計算
        top_left = pos
        bottom_right = (top_left[0] + rect_width, top_left[1] + rect_height)
        
        # 黒い枠線の長方形を描画
        cv2.rectangle(img, top_left, bottom_right, color, thickness)
        
        # 長方形の中心座標を計算
        center_x = top_left[0] + rect_width // 2
        center_y = top_left[1] + rect_height // 2
        
        # テキストのサイズを取得して、テキストの位置を調整
        text_size = cv2.getTextSize(texts[i], font, font_scale, font_thickness)[0]
        text_x = center_x - text_size[0] // 2
        text_y = center_y + text_size[1] // 2
        
        # テキストを中央に描画
        cv2.putText(img, texts[i], (text_x, text_y), font, font_scale, font_color, font_thickness, cv2.LINE_AA)

    # 画像を表示
    cv2.imshow("Rectangles with Text", img)
    cv2.setMouseCallback("Rectangles with Text", on_mouse_click)
    while len(point) < 1:
    # 画像を表示し続ける
        cv2.waitKey(1)
    cv2.destroyAllWindows()
    X = point[0][0]
    
    if X >= 50 and X < 200:
        return 1
    elif X >= 250 and X < 400:
        return 2
    elif X >= 450 and X < 600:
        return 3


def m1(img):

    # カメラで補助線を表示して撮影
    # img1=display_camera_with_guidelines()
    # cv2.imshow("test0",img1)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    # print(img1.shape[:1])
    # 撮影した画像のパス
    
    new = process_circle(img)
    # メーターの種類 1 MPa, 0.25 MPa, 1.6 MPa
    n = initialize()
    # 画像処理を行い、圧力計の円の中心と針の最長線を取得
    center_x, center_y, longest_line = process_image(new)

    if longest_line is not None:
        # 針の角度を計算
        angle_deg = calculate_angle((center_x, center_y), longest_line)

        # 圧力値を計算（例として最大圧力値を100として計算）
        # pressure_value = pressure_value_from_angle(angle_deg, max_pressure=1)
        pressure_value = pressure_value_from_angle(angle_deg, max_pressure=n)
        print(f"針の角度: {round(angle_deg,4)}度")
        print(f"圧力計の値: {round(pressure_value,4)} Pa")

        # テキストの内容
        text = f"Angle of the needle:{round(angle_deg,3)} degrees"
        text1 = f"Pressure:{round(pressure_value,3)} Pa"

        # フォントの種類 (例: cv2.FONT_HERSHEY_SIMPLEX)
        font = cv2.FONT_HERSHEY_SIMPLEX

        # フォントのスケール（大きさ)
        font_scale = 1

        # テキストの色 (B, G, R)
        color = (0, 0, 255)  # 白色

        # 線の太さ
        thickness = 2

        # テキストを画像に描画
        cv2.putText(img, text, (100,100), font, font_scale, color, thickness, cv2.LINE_AA)
        cv2.putText(img, text1, (100,150), font, font_scale, color, thickness, cv2.LINE_AA)

        # 画像を表示
        cv2.imshow('Image with Text', img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        # 画像を保存
        cv2.imwrite('image_with_text.jpg', img)
    else:
        print("圧力計の針を検出できませんでした。")

def main():

    # n = initialize()
    # print(n)
    #image_path = "/home/ros/ros2_ws/src/pressure/pressure/cut_circle.png"
    # image_path = "/home/ros/ros2_ws/src/pressure/data/cropped_images/LINE_ALBUM_パシャリーズ_240726_2.jpg"#0.25 Not
    # image_path = "/home/ros/ros2_ws/src/pressure/data/sample/LINE_ALBUM_パシャリーズ_240726_24.jpg"#1
    image_path = "/home/ros/ros2_ws/src/pressure/data/sample/LINE_ALBUM_パシャリーズ_240726_25.jpg"#1.6
    # image_path = "/home/ros/ros2_ws/src/pressure/data/sample/LINE_ALBUM_パシャリーズ_240726_3.jpg"#1.6
    # image_path = "/home/ros/ros2_ws/src/pressure/data/Origin_data/LINE_ALBUM_20240711_240730_54.jpg"#0.25
    img = cv2.imread(image_path)

    m1(img)

if __name__ == "__main__":
    main()