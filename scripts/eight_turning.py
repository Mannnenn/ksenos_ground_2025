import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import cumulative_trapezoid

def calculate_eight_trajectory_curvature(distance, max_curvature=0.1, cycle_length=200.0, 
                                        straight_ratio=0.25, transition_ratio=0.125, max_curve_ratio=0.125):
    """
    8の字軌道の曲率を計算する関数
    
    Parameters:
    - distance: 現在の距離
    - max_curvature: 最大曲率
    - cycle_length: 8の字1周期の距離
    - straight_ratio: 直線部の割合（0-1、デフォルト0.25 = 1/4周期分）
    - transition_ratio: 変化部の割合（0-1、デフォルト0.125 = 1/8周期分）
    - max_curve_ratio: 最大曲率部の割合（0-1、デフォルト0.125 = 1/8周期分）
    
    注意: straight_ratio + transition_ratio + max_curve_ratio ≤ 0.5であること
    （半周期分でこれらが繰り返される）
    """
    # パラメータの正規化（合計が0.5を超えないよう調整）
    total_ratio = straight_ratio + transition_ratio + max_curve_ratio
    if total_ratio > 0.5:
        # 0.5になるよう比例縮小
        scale = 0.5 / total_ratio
        straight_ratio *= scale
        transition_ratio *= scale
        max_curve_ratio *= scale
    
    # 8の字軌道の1周期内での位置を正規化（0-1）
    normalized_pos = (distance % cycle_length) / cycle_length
    
    # 半周期ごとに処理（前半と後半）
    half_cycle_pos = (normalized_pos * 2) % 1.0  # 0-1の範囲で半周期
    is_second_half = normalized_pos >= 0.5  # 後半かどうか
    
    # セグメント境界を計算
    transition1_end = transition_ratio * 2  # 変化部1の終了位置
    max_curve_end = transition1_end + max_curve_ratio * 2  # 最大曲率部の終了位置
    transition2_end = max_curve_end + transition_ratio * 2  # 変化部2の終了位置
    straight_end = transition2_end + straight_ratio * 2  # 直線部の終了位置
    
    curvature = 0.0
    
    if half_cycle_pos <= transition1_end:  # 変化部1（0から最大曲率へ）
        if transition_ratio > 0:
            progress = half_cycle_pos / (transition_ratio * 2)
            curvature = max_curvature * progress
    elif half_cycle_pos <= max_curve_end:  # 最大曲率部
        curvature = max_curvature
    elif half_cycle_pos <= transition2_end:  # 変化部2（最大曲率から0へ）
        if transition_ratio > 0:
            progress = (half_cycle_pos - max_curve_end) / (transition_ratio * 2)
            curvature = max_curvature * (1.0 - progress)
    else:  # 直線部
        curvature = 0.0
    
    # 後半は曲率の符号を反転
    if is_second_half:
        curvature = -curvature
    
    return curvature

def generate_eight_trajectory(total_distance=600, num_points=3000, max_curvature=0.1, cycle_length=200.0,
                             straight_ratio=0.25, transition_ratio=0.125, max_curve_ratio=0.125):
    """
    8の字軌道のX-Y座標を生成する関数
    
    Parameters:
    - total_distance: 総距離
    - num_points: 生成する点数
    - max_curvature: 最大曲率
    - cycle_length: 8の字1周期の距離
    - straight_ratio: 直線部の割合
    - transition_ratio: 変化部の割合  
    - max_curve_ratio: 最大曲率部の割合
    """
    # 距離配列を作成
    distances = np.linspace(0, total_distance, num_points)
    
    # 各点での曲率を計算
    curvatures = np.array([calculate_eight_trajectory_curvature(d, max_curvature, cycle_length,
                                                              straight_ratio, transition_ratio, max_curve_ratio) 
                          for d in distances])
    
    # 曲率から角度変化を積分
    ds = distances[1] - distances[0]  # 距離刻み
    angle_changes = curvatures * ds
    angles = np.cumsum(angle_changes)
    
    # 角度からX-Y座標を計算
    x = np.cumsum(np.cos(angles) * ds)
    y = np.cumsum(np.sin(angles) * ds)
    
    # 原点を調整
    x = x - x[0]
    y = y - y[0]
    
    return distances, x, y, curvatures, angles

def plot_eight_trajectory(straight_ratio=0.00, transition_ratio=0.125, max_curve_ratio=0.25):
    """
    8の字軌道をプロットする関数
    
    Parameters:
    - straight_ratio: 直線部の割合（0-1）
    - transition_ratio: 変化部の割合（0-1）
    - max_curve_ratio: 最大曲率部の割合（0-1）
    """
    # パラメータ設定
    max_curvature = 1/4.1  # 最大曲率 [1/m]
    cycle_length = 55.0  # 8の字1周期の距離 [m]
    total_distance = cycle_length * 2  # 総距離（3周期）
    
    # 軌道生成
    distances, x, y, curvatures, angles = generate_eight_trajectory(
        total_distance, 3000, max_curvature, cycle_length,
        straight_ratio, transition_ratio, max_curve_ratio)
    
    # プロット設定
    plt.figure(figsize=(15, 10))
    
    # X-Y軌道プロット
    plt.plot(x, y, 'b-', linewidth=2, label='Eight Trajectory', zorder=1)
    plt.scatter(x[0], y[0], color='green', s=100, label='Start', zorder=5)
    plt.scatter(x[-1], y[-1], color='red', s=100, label='End', zorder=5)
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title(f'Eight (X-Y plane)\nStraight: {straight_ratio:.2f}, Transition: {transition_ratio:.2f}, Max Curve: {max_curve_ratio:.2f}')
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.legend()
    plt.show()


if __name__ == "__main__":
    # デフォルトパラメータでのメイン軌道プロット
    print("=== デフォルトパラメータでの8の字軌道 ===")
    plot_eight_trajectory()