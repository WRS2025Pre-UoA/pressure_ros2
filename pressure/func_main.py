from camera_helper import display_camera_with_guidelines
from image_processing import process_image
from needle_angle import calculate_angle
#pressure_value_from_angle

def m(img):
    # カメラで補助線を表示して撮影
    # img=display_camera_with_guidelines()

    # 撮影した画像のパス
    # image_path = '../data/Origin_data_renamed/test_6.jpg'

    # 画像処理を行い、圧力計の円の中心と針の最長線を取得
    center_x, center_y, longest_line = process_image(img)

    if longest_line is not None:
        # 針の角度を計算
        angle_deg = calculate_angle((center_x, center_y), longest_line)

        # 圧力値を計算（例として最大圧力値を100として計算）
        #pressure_value = pressure_value_from_angle(angle_deg, max_pressure=100)

        print(f"針の角度: {angle_deg}度")
        #print(f"圧力計の値: {pressure_value} Pa")
    else:
        print("圧力計の針を検出できませんでした。")

def main():
    m()

if __name__ == "__main__":
    main()