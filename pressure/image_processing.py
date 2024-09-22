import cv2
import numpy as np

def re(img):
    height,width=img.shape[:2]
    # 目標の幅と高さ
    target_width = 640
    target_height = int((target_width / width) * height)

    # アフィン変換行列を計算する
    scale_x = target_width / width
    scale_y = target_height / height

    # アフィン変換行列の作成
    affine_matrix = np.array([[scale_x, 0, 0],
                            [0, scale_y, 0]], dtype=np.float32)

    # アフィン変換を適用する
    resized_image = cv2.warpAffine(img, affine_matrix, (target_width, target_height))

    return resized_image

def is_point_on_line(line, point, tolerance=15):
        """
        指定された点が線分上にあるかを確認する関数。
        toleranceは許容距離のしきい値。
        """
        x1, y1, x2, y2 = line[0]
        px, py = point

        # ベクトルを計算
        line_vec = np.array([x2 - x1, y2 - y1])
        point_vec = np.array([px - x1, py - y1])
        
        # 線の長さ
        line_len = np.linalg.norm(line_vec)
        if line_len == 0:
            return False
        
        # 線分上に点があるか確認するために、線分上の射影と距離を計算
        proj_len = np.dot(point_vec, line_vec) / line_len  # 射影の長さ
        proj_point = (proj_len / line_len) * line_vec + np.array([x1, y1])  # 射影点

        # 射影点と元の点の距離を測定して、tolerance以内であることを確認
        dist_to_line = np.linalg.norm(np.array([px, py]) - proj_point)
        
        # 射影が線分上にあり、距離がtolerance以内であれば線上にあるとみなす
        return 0 <= proj_len <= line_len and dist_to_line < tolerance


def process_image(img):
    # 画像の読み込みとグレースケール化
    # img = cv2.imread(image_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # バイナリ化 (しきい値の値は画像に応じて調整)
    # _, bw = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY)
    bw = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY_INV,11,10)
    # bw = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY_INV,11,10)
    # ノイズ除去（小さいゴミを取り除く）
    bw = cv2.medianBlur(bw, 5)

    # bw=re(bw)

    cv2.imshow("1",bw)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    # 圧力計の円形部分の検出
    # minRadius, maxRadius をカメラ補助線の円を基準に設定
    image_width = bw.shape[1]
    minRadius = int(image_width // 5)  # 画像の幅の1/5
    maxRadius = int(image_width // 3)  # 画像の幅の1/3

    print(minRadius,maxRadius)
    # HoughCirclesによる円の検出（Cannyやdpは使わない）
    circles = cv2.HoughCircles(bw, cv2.HOUGH_GRADIENT, 1,minDist=50,
                               param2=30, minRadius=minRadius, maxRadius=maxRadius)
    # print(circles)
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            # 検出された円を描画して確認
            cv2.circle(img, (i[0], i[1]), i[2], (0, 255, 0), 2)  # 外円
            # cv2.imshow("2-1",img)
            cv2.circle(img, (i[0], i[1]), 2, (0, 0, 255), 3)  # 中心点

            cv2.imshow("2-2",img)
            cv2.waitKey(0)
            
            # 最長線の検出のために、円の外側をマスクしてノイズを除去
            mask = np.zeros_like(bw)
            # cv2.imshow("mask",mask)
            # cv2.waitKey(0)
            cv2.circle(mask, (i[0], i[1]), i[2], 255, thickness=-1)
            masked_img = cv2.bitwise_and(bw, bw, mask=mask)

            # 最長線の検出
            lines = cv2.HoughLinesP(masked_img, 1, np.pi / 180, threshold=100, minLineLength=20, maxLineGap=20)
            longest_line=[]
            k = (i[0],i[1])
            # print(k)
            # print(lines)
            if lines is not None:
                print("Find line passing through the center")
                
                # 点 (k[0], k[1]) を通る線だけをフィルタリング
                valid_lines = [line for line in lines if is_point_on_line(line, k)]    
                # 最長の線を選択
                if valid_lines:
                    print("found")
                    longest_line = max(lines, key=lambda line: np.linalg.norm((line[0][0] - line[0][2], line[0][1] - line[0][3])))

                    # print(longest_line)
                    # for l in lines:
                    cv2.line(masked_img,longest_line[0][:2],longest_line[0][2:],100,2)
                    cv2.imshow("test",masked_img)
                    cv2.waitKey(0)
                else:
                    print("not found")

                # cv2.imwrite("test.png",masked_img)

                return i[0], i[1], longest_line  # 円の中心座標と最長線の情報を返す

    return None, None, None