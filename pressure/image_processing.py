import cv2
import numpy as np

def process_image(img):
    # 画像の読み込みとグレースケール化
    # img = cv2.imread(image_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # バイナリ化 (しきい値の値は画像に応じて調整)
    _, bw = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY)

    # ノイズ除去（小さいゴミを取り除く）
    bw = cv2.medianBlur(bw, 5)

    # 圧力計の円形部分の検出
    # minRadius, maxRadius をカメラ補助線の円を基準に設定
    image_width = bw.shape[1]
    minRadius = int(image_width // 5)  # 画像の幅の1/5
    maxRadius = int(image_width // 3)  # 画像の幅の1/3

    # HoughCirclesによる円の検出（Cannyやdpは使わない）
    circles = cv2.HoughCircles(bw, cv2.HOUGH_GRADIENT, 1,minDist=50,
                               param2=30, minRadius=minRadius, maxRadius=maxRadius)
    
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            # 検出された円を描画して確認
            cv2.circle(img, (i[0], i[1]), i[2], (0, 255, 0), 2)  # 外円
            cv2.circle(img, (i[0], i[1]), 2, (0, 0, 255), 3)  # 中心点

            # 最長線の検出のために、円の外側をマスクしてノイズを除去
            mask = np.zeros_like(bw)
            cv2.circle(mask, (i[0], i[1]), i[2], 255, thickness=-1)
            masked_img = cv2.bitwise_and(bw, bw, mask=mask)

            # 最長線の検出
            lines = cv2.HoughLinesP(masked_img, 1, np.pi / 180, threshold=100, minLineLength=50, maxLineGap=10)
            if lines is not None:
                # 最長の線を選択
                longest_line = max(lines, key=lambda line: np.linalg.norm((line[0][0] - line[0][2], line[0][1] - line[0][3])))

                print(longest_line)
                for l in lines:
                    cv2.line(masked_img,l[0][:2],l[0][2:],100,2)
                cv2.imshow("test",masked_img)
                cv2.waitKey(0)

                cv2.imwrite("test.png",masked_img)

                return i[0], i[1], longest_line  # 円の中心座標と最長線の情報を返す

    return None, None, None