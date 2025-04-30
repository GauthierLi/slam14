import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# 定义SE(3)到李代数的转换函数
def se3_from_SE3(T):
    R = T[:3, :3]
    p = T[:3, 3]
    
    # 计算旋转角度theta和旋转轴omega
    theta = np.arccos((np.trace(R) - 1) / 2)
    if theta == 0:
        omega_hat = np.zeros((3, 3))
    else:
        omega_hat = (R - R.T) / (2 * np.sin(theta))
    omega = np.array([omega_hat[2, 1], omega_hat[0, 2], omega_hat[1, 0]])
    
    # 计算雅可比矩阵J
    if theta == 0:
        J = np.eye(3)
    else:
        J = (np.sin(theta)/theta) * np.eye(3) + ((1-np.cos(theta))/theta**2) * omega_hat + (1 - np.sin(theta)/theta) / theta**2 * np.outer(omega, omega)
    
    # 计算v
    v = np.dot(np.linalg.inv(J), p)
    
    return np.concatenate([omega, v])

# 指数映射函数
def exp_se3(xi, theta=1.0):
    omega = xi[:3]
    v = xi[3:]
    theta = min(theta, 1.0) # 确保theta不超过1
    omega_norm = np.linalg.norm(omega)
    
    if omega_norm == 0:
        R = np.eye(3)
        V = np.eye(3)
    else:
        omega_hat = np.array([[0, -omega[2], omega[1]],
                              [omega[2], 0, -omega[0]],
                              [-omega[1], omega[0], 0]])
        omega_hat_theta = theta * omega_hat
        R = np.eye(3) + np.sin(theta * omega_norm) / omega_norm * omega_hat + (1 - np.cos(theta * omega_norm)) / omega_norm**2 * np.dot(omega_hat, omega_hat)
        
        V = np.eye(3) + (1 - np.cos(theta * omega_norm)) / omega_norm**2 * omega_hat + (theta * omega_norm - np.sin(theta * omega_norm)) / omega_norm**3 * np.dot(omega_hat, omega_hat)
    
    t = np.dot(V, v)
    
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    
    return T

# 绘制物体（这里用一个简单立方体表示）
def draw_cube(ax, T):
    points = np.array([[0, 0, 0, 1],
                       [1, 0, 0, 1],
                       [1, 1, 0, 1],
                       [0, 1, 0, 1],
                       [0, 0, 1, 1],
                       [1, 0, 1, 1],
                       [1, 1, 1, 1],
                       [0, 1, 1, 1]]).T
    transformed_points = np.dot(T, points)[:3, :]
    
    for i in range(4):
        ax.plot(transformed_points[0, [i, i+4]], transformed_points[1, [i, i+4]], transformed_points[2, [i, i+4]], color='r')
        if i < 3:
            ax.plot(transformed_points[0, [i, i+1]], transformed_points[1, [i, i+1]], transformed_points[2, [i, i+1]], color='b')
            ax.plot(transformed_points[0, [i+4, i+5]], transformed_points[1, [i+4, i+5]], transformed_points[2, [i+4, i+5]], color='b')
    ax.plot(transformed_points[0, [0, 3]], transformed_points[1, [0, 3]], transformed_points[2, [0, 3]], color='b')
    ax.plot(transformed_points[0, [4, 7]], transformed_points[1, [4, 7]], transformed_points[2, [4, 7]], color='b')

# 动画更新函数
trajectory_points = []
def update(frame):
    global trajectory_points
    ax.clear()
    ax.set_xlim([-2, 2])
    ax.set_ylim([-2, 2])
    ax.set_zlim([-2, 2])
    
    current_T = exp_se3(xi, frame/100)
    trajectory_points.append(current_T[:3, 3])
    
    if len(trajectory_points) > 1:
        xs, ys, zs = zip(*trajectory_points)
        ax.plot(xs, ys, zs, color='g', label='Trajectory')
    
    draw_cube(ax, current_T)
    ax.legend()

# 创建图形
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# SE(3) 矩阵，这里假设绕Y轴旋转并沿X轴平移
T = np.array([
    [np.cos(np.pi/4), 0, np.sin(np.pi/4), 1],
    [0, 1, 0, 0],
    [-np.sin(np.pi/4), 0, np.cos(np.pi/4), 0],
    [0, 0, 0, 1]
])

# 转换到李代数
xi = se3_from_SE3(T)

# 创建动画
ani = FuncAnimation(fig, update, frames=100, interval=100, repeat=False)
plt.show()
