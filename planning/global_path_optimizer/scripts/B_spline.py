import numpy as np
from scipy.interpolate import BSpline
import matplotlib.pyplot as plt

# B样条，用于全局路径的优化
def B_spline(x, y,point_num,plot_flag):
    # 创建B样条插值对象
    t = np.linspace(0, 1, len(x) - 2, endpoint=True)
    t = np.concatenate(([0, 0, 0], t, [1, 1, 1]))
    spl = BSpline(t, np.array([x, y]).T, 3)
    # 生成插值点
    x2 = np.linspace(0, 1, point_num)
    y2 = spl(x2)


    if plot_flag:
        # 绘制原折线和插值曲线
        plt.plot(x, y, 'ro-', label='Original points')
        plt.plot(y2[:, 0], y2[:, 1], 'b-', label='BSpline')
        plt.legend(loc='best')
        plt.show()

    # 返回np.array
    return np.array(y2)


if __name__ == '__main__':
    # 输入点的坐标
    point_list = [[0, 0], [1, 3], [2, 6], [3, 7], [4, 8], [5, 2], [6, 1], [7, 0]]
    # x = np.array([0, 1, 2, 3,4,5,6,7])
    # y = np.array([0, 3, 6, 7,8,2,1,0])
    path_list = B_spline(np.array(point_list)[:,0], np.array(point_list)[:,1],100,True)
    # B_spline(x, y,100,plot_flag=True)
