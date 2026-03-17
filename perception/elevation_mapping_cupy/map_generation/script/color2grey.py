import cv2

def color2grey(input_image_path, output_image_path, size):
    # 读取图像
    img = cv2.imread(input_image_path)

    # 转换为灰度图像
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 保存提取边缘后的图像
    cv2.imwrite(output_image_path, gray)

if __name__ == "__main__":
    image_path = '../maps/original_int8/map_genshin.jpeg'
    output_path = '../maps/resized/HM99.png'
    color2grey(image_path, output_path, (640, 640))