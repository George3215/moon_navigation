import numpy as np
import matplotlib.pyplot as plt

HERMITE_MAT = np.array([[1, 0, 0, 0], [0, 1, 0, 0],
                        [-3, -2, 3, -1], [2, 1, -2, 1]])

def cubic_hermite_evaluate(knots: np.ndarray, num_points):
    """
    Evaluate cubic hermite curve at parameter t
    :param knots: control points, literally `np.ndarray([p0, v0, p1, v1])`
    :param t: parameter vector range from 0 to 1
    :return: point on hermite curve
    """
    t = np.linspace(0, 1, num_points)
    return cubic_evaluate(knots, t, HERMITE_MAT)

def cubic_evaluate(knots: np.ndarray, t, para_mat: np.ndarray, eval_mode="pos"):
    """
    Evaluate cubic curve at parameter t
    :param knots: control points of cubic curve with shape (4 row, x col)
    :param t: parameter vector range from 0 to 1
    :param para_mat: polynomial parameter matrix
    :param eval_mode: "pos" or "vel"
    """
    if type(t) is not np.ndarray:
        t = np.array([t])
    if eval_mode == "pos":
        t_vec = np.array([np.ones(t.shape), t, t**2, t**3]).T
    elif eval_mode == "vel":
        t_vec = np.array([np.zeros(t.shape), np.ones(t.shape), 2*t, 3*t**2]).T
    else:
        raise ValueError("Invalid eval_mode")
    point = t_vec @ para_mat @ knots
    if point.shape[0] == 1:
        return point.flatten()
    else:
        return point


if __name__ == '__main__':
    start_pt = np.array([0, 0])
    goal_pt = np.array([10, 10])
    start_vec = np.array([10, 0])
    goal_vec = np.array([0, 10])
    num_points = 100
    # print(start_pt)
    x= cubic_hermite_evaluate(np.array([start_pt, start_vec,goal_pt,goal_vec]), num_points)
    plt.plot(x[:, 0], x[:, 1], 'b-', label='Hermite')
    # 绘制起点和终点
    plt.plot(start_pt[0], start_pt[1], 'ro', label='Start point')
    plt.plot(goal_pt[0], goal_pt[1], 'ro', label='Goal point')
    # 绘制起点和终点向量
    plt.quiver(start_pt[0], start_pt[1], start_vec[0], start_vec[1], angles='xy', scale_units='xy', scale=10)
    plt.quiver(goal_pt[0], goal_pt[1], goal_vec[0], goal_vec[1], angles='xy', scale_units='xy', scale=10)
    plt.legend(loc='best')
    plt.show()
