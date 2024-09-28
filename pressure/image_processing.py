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
    指定された点が線分上またはその延長上にあるかを確認する関数。
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

    # 線分上の射影の長さを計算
    proj_len = np.dot(point_vec, line_vec) / line_len  # 射影の長さ
    proj_point = (proj_len / line_len) * line_vec + np.array([x1, y1])  # 射影点

    # 射影点と元の点の距離を測定
    dist_to_line = np.linalg.norm(np.array([px, py]) - proj_point)

    # 射影が線の延長上にあり、距離がtolerance以内であれば線上にあるとみなす
    return dist_to_line < tolerance




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

    # cv2.imshow("1",bw)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    # 圧力計の円形部分の検出
    # minRadius, maxRadius をカメラ補助線の円を基準に設定
    image_width = bw.shape[1]
    minRadius = int(image_width // 5)  # 画像の幅の1/5
    maxRadius = int(image_width // 3)  # 画像の幅の1/3

    # print(minRadius,maxRadius)
    # HoughCirclesによる円の検出（Cannyやdpは使わない）
    circles = cv2.HoughCircles(bw, cv2.HOUGH_GRADIENT, 1,minDist=50,
                               param2=30, minRadius=minRadius, maxRadius=maxRadius)
    # print(circles)
    if circles is not None:
        circles = np.uint16(np.around(circles))

        largest_circle = max(circles[0, :], key=lambda c: c[2])  # 半径が最大の円を取得

        # 最大の円の座標と半径
        cx, cy, r = largest_circle
        # 検出された円を描画して確認
        cv2.circle(img, (cx, cy), r, (0, 255, 0), 2)  # 外円
        # cv2.imshow("2-1",img)
        cv2.circle(img, (cx, cy), 2, (0, 0, 255), 3)  # 中心点

        # cv2.imshow("2-2",img)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        
        # 最長線の検出のために、円の外側をマスクしてノイズを除去
        mask = np.zeros_like(bw)
        # cv2.imshow("mask",mask)
        # cv2.waitKey(0)
        cv2.circle(mask, (cx, cy), r, 255, thickness=-1)
        masked_img = cv2.bitwise_and(bw, bw, mask=mask)

        # 最長線の検出
        lines = cv2.HoughLinesP(masked_img, 1, np.pi / 180, threshold=100, minLineLength=10, maxLineGap=50)
        longest_line=[]
        k = (cx,cy)
        # print(k)
        print(lines)
        if lines is not None:
            print("Find line passing through the center")
            
            # 点 (k[0], k[1]) を通る線だけをフィルタリング
            valid_lines = [line for line in lines if is_point_on_line(line, k)]    
            # for line in lines:
            #     x1, y1, x2, y2 = line[0]  # 線の始点と終点の座標を取得
            #     # 線の長さを求める
            #     length = np.linalg.norm((x2 - x1, y2 - y1))
            #     # print(length)
            #     # cv2.line(img, (x1, y1), (x2, y2), (255, 0, 0), 2)  # すべての線を青色で描画
            #     # cv2.imshow("test9", img)
            #     # cv2.waitKey(0)
                
            # 最長の線を選択
            if valid_lines:
                print("found")
                # longest_line = max(lines, key=lambda line: np.linalg.norm((line[0][0] - line[0][2], line[0][1] - line[0][3])))
                longest_line = max(
                    valid_lines, 
                    key=lambda line: np.linalg.norm((line[0][0] - line[0][2], line[0][1] - line[0][3]))
                )
                # print(longest_line)
                # for l in lines:
                    # 最長の線を描画
                pt1 = (int(longest_line[0][0]), int(longest_line[0][1]))  # 線の始点
                pt2 = (int(longest_line[0][2]), int(longest_line[0][3]))  # 線の終点

                cv2.line(img, pt1, pt2, (0, 0, 255), 3)
                cv2.imshow("test",img)
                cv2.waitKey(0)
                cv2.destroyAllWindows() 
                return cx, cy, longest_line # 円の中心座標と最長線の情報を返す
            else:
                print("not found")

            # cv2.imwrite("test.png",masked_img)

                

    return None, None, None