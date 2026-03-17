from PIL import Image

# 定义压缩图片的函数
def half_size_image(image_path, output_path):
    # 打开图片
    image = Image.open(image_path)
    
    # 获取原始图片的尺寸
    original_width, original_height = image.size
    
    # 计算新的尺寸（原始尺寸的一半）
    new_width = original_width // 6
    new_height = original_height // 6
    
    # 缩放图片
    compressed_image = image.resize((new_width, new_height), Image.ANTIALIAS)
    
    # 保存压缩后的图片
    compressed_image.save(output_path)
    print(new_width, new_height)

# 调用函数，压缩图片
# 请替换成您的图片路径和输出路径
half_size_image('/home/r9000p/catkin_ws/src/rqt_mypkg/resource/images/hunter-se.jpg', 'half_size_image.png')
