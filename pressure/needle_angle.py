import numpy as np

def calculate_angle(center, line):
    """
    針の角度を計算する関数。
    2つの端点のうち、圧力計の中心から遠い方を針の先端とみなして角度を計算します。

    Parameters:
    center (tuple): 圧力計の中心座標 (x, y)
    line (tuple): 針の2つの端点 (x1, y1, x2, y2)

    Returns:
    float: 針の角度（度数法）
    """
    # 圧力計の中心からの距離を計算
    x1, y1, x2, y2 = line[0]
    distance_to_point1 = np.linalg.norm([x1 - center[0], y1 - center[1]])  # 点1の距離
    distance_to_point2 = np.linalg.norm([x2 - center[0], y2 - center[1]])  # 点2の距離

    # 圧力計の中心から遠い方を針の先端とみなす
    if distance_to_point1 > distance_to_point2:
        needle_endpoint = (x1, y1)
    else:
        needle_endpoint = (x2, y2)

    # 針の先端と中心を結ぶベクトルを計算
    needle_vector = np.array([needle_endpoint[0] - center[0], needle_endpoint[1] - center[1]])
    reference_vector = np.array([0, -1])  # 12時方向の基準ベクトル

    # ベクトルを正規化
    needle_vector = needle_vector / np.linalg.norm(needle_vector)
    reference_vector = reference_vector / np.linalg.norm(reference_vector)

    # 内積から角度を計算
    dot_product = np.dot(needle_vector, reference_vector)
    angle_rad = np.arccos(dot_product)  # ラジアンの角度
    angle_deg = np.degrees(angle_rad)   # 度数法に変換

    # 針が左半分にある場合は、360度 - angle_deg にする
    if needle_vector[0] < 0:
        angle_deg = 360 - angle_deg

    return angle_deg