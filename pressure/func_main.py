from pressure.image_processing import process_image
from pressure.needle_angle import calculate_angle
from pressure.needle_angle import pressure_value_from_angle
from pressure.detect_circle import process_circle
from pressure.detect_circle import affine

import cv2
import numpy as np
#pressure_value_from_angle

# point=[]
# マウスクリックのイベントを定義
def on_mouse_click(event, x, y, flags, point):
    if event == cv2.EVENT_LBUTTONDOWN:
        point.append((x, y))  # クリックした座標をリストに追加

# def initialize(values):
#     # 真っ白な背景 (650x400)
#     img = np.ones((400, 650, 3), dtype=np.uint8) * 255

#     # 長方形の大きさ (150x100)
#     rect_width = 150
#     rect_height = 100

#     # 黒い枠線で描く長方形の左上座標
#     positions = [(50, 75), (250, 75), (450, 75),(50, 215), (250, 215), (450, 215)]
    

#     # 各長方形に表示するテキスト
#     # texts = ["1MPa", "0.25MPa", "1.6MPa","2.5MPa","",""]
#     texts = ["%.2fMPa"%v for v in values]

#     # 描画する枠線の色と太さ
#     color = (0, 0, 0)  # 黒
#     thickness = 2

#     # フォント、スケール、色、太さ
#     font = cv2.FONT_HERSHEY_SIMPLEX
#     font_scale = 1
#     font_color = (0, 0, 255)  # 赤
#     font_thickness = 2

#     point=[]

#     # 各長方形を描画し、中央にテキストを配置
#     for i, pos in enumerate(positions):
#         # 長方形の右下座標を計算
#         top_left = pos
#         bottom_right = (top_left[0] + rect_width, top_left[1] + rect_height)
        
#         # 黒い枠線の長方形を描画
#         cv2.rectangle(img, top_left, bottom_right, color, thickness)
        
#         # 長方形の中心座標を計算
#         center_x = top_left[0] + rect_width // 2
#         center_y = top_left[1] + rect_height // 2
        
#         # テキストのサイズを取得して、テキストの位置を調整
#         text_size = cv2.getTextSize(texts[i], font, font_scale, font_thickness)[0]
#         text_x = center_x - text_size[0] // 2
#         text_y = center_y + text_size[1] // 2
        
#         # テキストを中央に描画
#         cv2.putText(img, texts[i], (text_x, text_y), font, font_scale, font_color, font_thickness, cv2.LINE_AA)

#     # 画像を表示
#     cv2.imshow("Rectangles with Text", img)
#     cv2.setMouseCallback("Rectangles with Text", on_mouse_click,point)
#     while len(point) < 1:
#     # 画像を表示し続ける
#         key = cv2.waitKey(1)
#         if key==-1 and cv2.getWindowProperty("Rectangles with Text",cv2.WND_PROP_VISIBLE)<1:
#             raise ValueError("Closed Window!")

#     cv2.destroyAllWindows()
#     X = point[0][0]
#     Y = point[0][1]
    
#     if X >= 50 and X < 200: 
#         if Y >= 75 and Y < 175:
#             return 1
#         elif Y >= 215 and Y < 315:
#             return 4
#     elif X >= 250 and X < 400:
#         if Y >= 75 and Y < 175:
#             return 2
#         elif Y >= 215 and Y < 315:
#             return 5
#     elif X >= 450 and X < 600:
#         if Y >= 75 and Y < 175:
#             return 3
#         elif Y >= 215 and Y < 315:
#             return 6
def initialize(values,num):
    # 真っ白な背景 (600x400)
    img = np.ones((400, 600, 3), dtype=np.uint8) * 255

    # 長方形の大きさ (150x100)
    rect_width = 100
    rect_height = 75
# ボタンの配置（3x3）
    positions = []
    texts = []
    if num == 1:
        positions = [(50 + (i % 3) * 200, 75 + (i // 3) * 120) for i in range(9)]

        # 各長方形に表示するテキスト
        texts = ["%.2f" % v for v in values] + [""] * (9 - len(values))  # 9個に満たない場合は空白を追加
    elif num == 2:
        # ボタンの配置（3x2）
        positions = [(50 + (i % 3) * 200, 75 + (i // 3) * 120) for i in range(6)]

        # 各長方形に表示するテキスト
        texts = ["%.2f" % v for v in values] + [""] * (6 - len(values))  # 6個に満たない場合は空白を追加

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
        top_left = pos
        bottom_right = (top_left[0] + rect_width, top_left[1] + rect_height)

        # 黒い枠線の長方形を描画
        cv2.rectangle(img, top_left, bottom_right, color, thickness)

        # 長方形の中心座標を計算
        center_x = top_left[0] + rect_width // 2
        center_y = top_left[1] + rect_height // 2

        # テキストのサイズを取得して、テキストの位置を調整
        text = texts[i]
        text_size = cv2.getTextSize(text, font, font_scale, font_thickness)[0]
        text_x = center_x - text_size[0] // 2
        text_y = center_y + text_size[1] // 2

        # テキストを中央に描画
        cv2.putText(img, text, (text_x, text_y), font, font_scale, font_color, font_thickness, cv2.LINE_AA)
    point = []
    # 画像を表示
    if num == 1:
        cv2.imshow("Choose Max Value", img)
        cv2.setMouseCallback("Choose Max Value", on_mouse_click, point)

        while True:
            # 画像を表示し続ける
            key = cv2.waitKey(1)
            # if key == -1 and cv2.getWindowProperty("Choose Max Value", cv2.WND_PROP_VISIBLE) < 1:
            #     raise ValueError("Closed Window!")
            if len(point)>0:
                X, Y = point[-1]

                # ボタンのクリック位置に応じた値を返す
                for i, pos in enumerate(positions):
                    top_left = pos
                    bottom_right = (top_left[0] + rect_width, top_left[1] + rect_height)
                    if top_left[0] <= X < bottom_right[0] and top_left[1] <= Y < bottom_right[1]:
                        if i < len(values):  # 有効な値の範囲内の場合
                            cv2.destroyAllWindows()
                            return values[i]  # クリックしたボタンに対応する値を返す

        
        #         else:
        #             return None  # 空白ボタンがクリックされた場合

        # return None  # どのボタンにも該当しない場合
    elif num == 2:
        cv2.imshow("Choose Min Value", img)
        cv2.setMouseCallback("Choose Min Value", on_mouse_click, point)

        while True:
            # 画像を表示し続ける
            key = cv2.waitKey(1)
            # if key == -1 and cv2.getWindowProperty("Choose Min Value", cv2.WND_PROP_VISIBLE) < 1:
                # raise ValueError("Closed Window!")

            if len(point)>0:
                X, Y = point[-1]

                # ボタンのクリック位置に応じた値を返す
                for i, pos in enumerate(positions):
                    top_left = pos
                    bottom_right = (top_left[0] + rect_width, top_left[1] + rect_height)
                    if top_left[0] <= X < bottom_right[0] and top_left[1] <= Y < bottom_right[1]:
                        if i < len(values):  # 有効な値の範囲内の場合
                            cv2.destroyAllWindows()
                            return values[i]  # クリックしたボタンに対応する値を返す

        # cv2.destroyAllWindows()

        # X, Y = point[0]

        # # ボタンのクリック位置に応じた値を返す
        # for i, pos in enumerate(positions):
        #     top_left = pos
        #     bottom_right = (top_left[0] + rect_width, top_left[1] + rect_height)
        #     if top_left[0] <= X < bottom_right[0] and top_left[1] <= Y < bottom_right[1]:
        #         if i < len(values):  # 有効な値の範囲内の場合
        #             print(values[i])
        #             return values[i]  # クリックしたボタンに対応する値を返す
        #         else:
        #             return None  # 空白ボタンがクリックされた場合

        return None  # どのボタンにも該当しない場合
    
#追加したよん
def initialize_unit():
    """
    圧力単位を選ぶための画面を表示
    """
    # 真っ白な背景 (400x200)
    img = np.ones((200, 400, 3), dtype=np.uint8) * 255

    # 単位の選択ボタンの配置
    positions = [(50, 75), (250, 75)]
    texts = ["MPa", "Pa"]

    rect_width = 100
    rect_height = 50
    color = (0, 0, 0)
    thickness = 2
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 1
    font_color = (0, 0, 255)
    font_thickness = 2

    for i, pos in enumerate(positions):
        top_left = pos
        bottom_right = (top_left[0] + rect_width, top_left[1] + rect_height)

        cv2.rectangle(img, top_left, bottom_right, color, thickness)

        center_x = top_left[0] + rect_width // 2
        center_y = top_left[1] + rect_height // 2

        text_size = cv2.getTextSize(texts[i], font, font_scale, font_thickness)[0]
        text_x = center_x - text_size[0] // 2
        text_y = center_y + text_size[1] // 2

        cv2.putText(img, texts[i], (text_x, text_y), font, font_scale, font_color, font_thickness, cv2.LINE_AA)

    point = []
    cv2.imshow("Choose Unit", img)
    cv2.setMouseCallback("Choose Unit", on_mouse_click, point)

    while len(point) < 1:
        key = cv2.waitKey(1)
        if key == -1 and cv2.getWindowProperty("Choose Unit", cv2.WND_PROP_VISIBLE) < 1:
            raise ValueError("Closed Window!")

    cv2.destroyAllWindows()

    X, Y = point[0]

    if positions[0][0] <= X < positions[0][0] + rect_width and positions[0][1] <= Y < positions[0][1] + rect_height:
        return "MPa"
    elif positions[1][0] <= X < positions[1][0] + rect_width and positions[1][1] <= Y < positions[1][1] + rect_height:
        return "Pa"
    else:
        return None


def m1(img,clicks):

    values =[1.0,0.25,1.6,2.5]
    values1 = [-0.1,-0.05,0.0]
    if clicks is None:
        clicks = []
    
        
    
    
    new = process_circle(img,clicks)
    # new = affine(img)
    
    # メーターの種類 1 MPa, 0.25 MPa, 1.6 MPa
    maxV = initialize(values,1)
    minV = initialize(values1,2)
    print(maxV,minV)

    unit = initialize_unit()
    # 画像処理を行い、圧力計の円の中心と針の最長線を取得
    center_x, center_y, longest_line = process_image(new)

    if longest_line is not None:
        # 針の角度を計算
        angle_deg = calculate_angle((center_x, center_y), longest_line)

        # 圧力値を計算（例として最大圧力値を100として計算）
        # pressure_value = pressure_value_from_angle(angle_deg, max_pressure=1)
        pressure_value = pressure_value_from_angle(angle_deg, max_pressure=maxV,min_pressure=minV)
        if pressure_value == None:
            return None,None
        
        if unit == "Pa":
            pressure_value *= 1e6

        print(f"針の角度: {round(angle_deg,4)}度")
        print(f"圧力計の値: {round(pressure_value,4)} Pa")

        # テキストの内容
        text = f"Angle of the needle:{round(angle_deg,3)} degrees"
        text1 = f"Pressure:{round(pressure_value,3)} MPa"

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

        return img,pressure_value
        # 画像を保存
        # cv2.imwrite('image_with_text.jpg', img)
    else:
        print("圧力計の針を検出できませんでした。")
        return None, None

def main():

    # n = initialize()
    # print(n)
    #image_path = "/home/ros/ros2_ws/src/pressure/pressure/cut_circle.png"
    # image_path = "../data/cropped_images/LINE_ALBUM_パシャリーズ_240726_2.jpg"#0.25 Not
    # image_path = "../data/sample/LINE_ALBUM_パシャリーズ_240726_24.jpg"#1
    # image_path = "../data/sample/LINE_ALBUM_パシャリーズ_240726_25.jpg"#1.6
    image_path = "../data/sample/LINE_ALBUM_パシャリーズ_240726_3.jpg"#1.6
    # image_path = "../data/Origin_data/LINE_ALBUM_20240711_240730_54.jpg"#0.25
    clicks = []
    
    img = cv2.imread(image_path)
    N,value = m1(img,clicks)

if __name__ == "__main__":
    main()