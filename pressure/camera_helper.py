'''
撮影の補助線ファイル
撮影するときにこれを使って！main.pyを実行すれば内部カメラが動く

補助線の全体像
垂直線（縦線）: 画像の中央を上下に通過する緑色の線。
水平線（横線）: 画像の中央を左右に通過する緑色の線。
円: 半径が画像の横幅の3分の1、つまり直径が横幅の3分の2の緑色の円が、画像の中心に描画されます。
'''
import cv2

def display_camera_with_guidelines():
    # カメラを起動して、補助線を表示する

    #内部カメラ用
    cap = cv2.VideoCapture(0)
    #外部カメラ用
    #cap = cv2.VideoCapture(1)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # 'q' キーを押すと終了
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            return frame
            break

        # 画像の中央に補助線を描画
        height, width, _ = frame.shape
        center_x, center_y = width // 2, height // 2
        color = (0, 255, 0)  # 緑色
        thickness = 2
        
        # 横線と縦線を描画
        cv2.line(frame, (center_x, 0), (center_x, height), color, thickness)
        cv2.line(frame, (0, center_y), (width, center_y), color, thickness)

        # 半径が横線の4a分の1の円を描画
        radius = width // 4  # 半径は横線（画像幅）の4分の1
        cv2.circle(frame, (center_x, center_y), radius, color, thickness)

        # フレームを表示
        cv2.imshow('Camera with Guidelines', frame)
    
    cap.release()
    cv2.destroyAllWindows()