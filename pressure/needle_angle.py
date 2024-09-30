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


def pressure_value_from_angle(theta, max_pressure,min_pressure):
    """
    針の角度から圧力計の値を計算する関数。
    228度の時に0Pa, 360度と0度の時に0.5MPa, 132度の時に1MPa。

    Parameters:
    angle_deg (float): 針の角度（度数法）

    Returns:
    float: 圧力値（MPa）
    """

    MaxValue = max_pressure

    MinValue= min_pressure; 
    
    if 228 <=theta and theta <= 360:
        p = (MaxValue-MinValue)/2*(theta-228)/132+MinValue
    elif 0 <= theta and theta <= 132:
        p = (MaxValue-MinValue)/2*theta/132+(MaxValue-MinValue)/2 + MinValue
    elif 218 <= theta and theta <= 228:
        p = MinValue
    elif 132 <= theta and theta <= 142:
        p = MaxValue
    else:
        p = None
    return p
    # if max_pressure == 1:#1.0MPa
    #     if 228 <= angle_deg < 270:
    #         # 228度から270度までは0MPaから0.16MPaに線形補間
    #         pressure = (angle_deg - 228) * (0.16 / (270 - 228))
    #     elif 270 <= angle_deg <= 360:
    #         # 270度から360度までは0.16MPaから0.5MPaに線形補間
    #         pressure = 0.16 + (angle_deg - 270) * (0.34 / (360 - 270))
    #     elif 0 <= angle_deg <= 90:
    #         # 0度から90度までは0.5MPaから0.84MPaに線形補間
    #         pressure = 0.5 + (angle_deg / 90) * (0.84 - 0.5)
    #     elif 90 < angle_deg <= 132:
    #         # 90度から132度までは0.84MPaから1.0MPaに線形補間
    #         pressure = 0.84 + ((angle_deg - 90) / (132 - 90)) * (1.0 - 0.84)

    # elif max_pressure == 2:#0.25MPa
    #     if 228 <= angle_deg < 270:
    #         # 228度から270度までは0 MPaから0.04 MPaに線形補間
    #         pressure = (angle_deg - 228) * (0.04 / (270 - 228))
    #     elif 270 <= angle_deg <= 360:
    #         # 270度から360度までは0.04 MPaから0.125 MPaに線形補間
    #         pressure = 0.04 + (angle_deg - 270) * (0.125 / (360 - 270))
    #     elif 0 <= angle_deg <= 90:
    #         # 0度から90度までは0.125 MPaから0.21 MPaに線形補間
    #         pressure = 0.125 + (angle_deg / 90) * (0.21 - 0.125)
    #     elif 90 < angle_deg <= 132:
    #         # 90度から132度までは0.21 MPaから0.25 MPaに線形補間
    #         pressure = 0.21 + ((angle_deg - 90) / (132 - 90)) * (0.25 - 0.21)
    #     else:
    #         # 132度から228度までは0 MPa
    #         pressure = 0.0

    # elif max_pressure == 3:#1.6MPa
    #     if 228 <= angle_deg < 270:
    #         # 228度から270度までは0 MPaから0.256 MPaに線形補間
    #         pressure = (angle_deg - 228) * (0.256 / (270 - 228))
    #     elif 270 <= angle_deg <= 360:
    #         # 270度から360度までは0.256 MPaから0.8 MPaに線形補間
    #         pressure = 0.256 + (angle_deg - 270) * (0.8 / (360 - 270))
    #     elif 0 <= angle_deg <= 90:
    #         # 0度から90度までは0.8 MPaから1.344 MPaに線形補間
    #         pressure = 0.8 + (angle_deg / 90) * (1.344 - 0.8)
    #     elif 90 < angle_deg <= 132:
    #         # 90度から132度までは1.344 MPaから1.6 MPaに線形補間
    #         pressure = 1.344 + ((angle_deg - 90) / (132 - 90)) * (1.6 - 1.344)

    # elif max_pressure == 4:#2.5MPa
    #     if 228 <= angle_deg < 270:
    #         # 228度から270度までは0 MPaから0.4 MPaに線形補間
    #         pressure = (angle_deg - 228) * (0.4 / (270 - 228))
    #     elif 270 <= angle_deg <= 360:
    #         # 270度から360度までは0.4 MPaから1.25 MPaに線形補間
    #         pressure = 0.4 + (angle_deg - 270) * (1.25 / (360 - 270))
    #     elif 0 <= angle_deg <= 90:
    #         # 0度から90度までは1.25 MPaから2.1 MPaに線形補間
    #         pressure = 1.25 + (angle_deg / 90) * (2.1 - 1.25)
    #     elif 90 < angle_deg <= 132:
    #         # 90度から132度までは2.1 MPaから2.5 MPaに線形補間
    #         pressure = 2.1 + ((angle_deg - 90) / (132 - 90)) * (2.5 - 2.1)
    #     else:
    #         # 132度から228度までは0 MPa
    #         pressure = 0.0
    # else:
    #     pressure =0.0



    # if max_pressure == 1:
    #     if 228 <= angle_deg <= 360:
    #         # 228度から360度までの範囲では、0Paから0.5MPaに線形補間
    #         pressure = (angle_deg - 228) * (0.5 / (360 - 228))
    #     elif 0 <= angle_deg <= 132:
    #         # 0度から132度までの範囲では、0.5MPaから1MPaに線形補間
    #         pressure = 0.5 + (angle_deg / 132) * (1 - 0.5)
    #     else:
    #         # 132度から228度までは0MPa
    #         pressure = 0.0

    # elif max_pressure == 2:
    #     if 228 <= angle_deg <= 360:
    #         # 228度から360度までの範囲では、0Paから0.125MPaに線形補間
    #         pressure = (angle_deg - 228) * (0.125 / (360 - 228))
    #     elif 0 <= angle_deg <= 132:
    #         # 0度から132度までの範囲では、0.125MPaから0.25MPaに線形補間
    #         pressure = 0.125 + (angle_deg / 132) * (0.25 - 0.125)
    #     else:
    #         # 132度から228度までは0MPa
    #         pressure = 0.0

    # elif max_pressure == 3:
        #  if 228 <= angle_deg <= 360:
    #         # 228度から360度までの範囲では、0Paから0.125MPaに線形補間
    #         pressure = (angle_deg - 228) * (0.8 / (360 - 228))
    #     elif 0 <= angle_deg <= 132:
    #         # 0度から132度までの範囲では、0.125MPaから0.25MPaに線形補間
    #         pressure = 0.8 + (angle_deg / 132) * (1.6 - 0.125)
    #     else:
    #         # 132度から228度までは0MPa
    #         pressure = 0.0

    # elif max_pressure == 4:
    #     if 228 <= angle_deg <= 360:
    #         # 228度から360度までの範囲では、0Paから0.125MPaに線形補間
    #         pressure = (angle_deg - 228) * (1.25 / (360 - 228))
    #     elif 0 <= angle_deg <= 132:
    #         # 0度から132度までの範囲では、0.125MPaから0.25MPaに線形補間
    #         pressure = 1.25 + (angle_deg / 132) * (2.5 - 0.125)
    #     else:
    #         # 132度から228度までは0MPa
    #         pressure = 0.0