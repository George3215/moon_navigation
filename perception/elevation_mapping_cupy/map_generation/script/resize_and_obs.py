from PIL import Image
import yaml

# 读取图片并调整到地图大小
def read_image(input_image_path, size):
    original_image = Image.open(input_image_path)
    #width, height = original_image.size
    #print(f"The original image size is {width} wide x {height} tall")
    resized_image = original_image.resize(size)
    return resized_image

# 添加障碍
def add_obs(image,rockfile_path,rock_resolution,map_height):
    with open(rockfile_path, 'r') as f:
        rocks = f.readlines()
    # image.show() 
    # # 第一行为标题行
    rocks = rocks[1:]
    for rock in rocks:
        x, y, D,z = rock.split(',')
        y, x, D,z = image.height - int(float(x)/rock_resolution), int(float(y)/rock_resolution),  (float(D)/rock_resolution), (float(z))
        # 检测该障碍像素值，像素值应该比原来的略大一点
        print_color = image.getpixel((x, y)) + int((0.3+z)*255/map_height)
        print("original: ", image.getpixel((x, y)), "z",z,"new: ", print_color)
        if print_color > 255:
            print_color = 255
        # if image.getpixel((x, y)) < 128:
        #     print_color = 255
        # else:
        #     print_color = 0
        # 根据z轴计算障碍物直径D，如果z小于0时岩石会在地面以下，导致障碍物范围减小
        if z < 0:
            D = (int(D+z/D))
        else:
            D = int(D)
        #根据D的大小，将障碍物的范围扩大，将周围一圈像素值涂白或黑
        for i in range(x-D, x+D+1):
            for j in range(y-D, y+D+1):
                if (i-x)**2 + (j-y)**2 <= D**2:
                    image.putpixel((i, j), print_color)
    image.show()
    return image
        

def resize_image(original_image, output_image_path, size):
    width, height = original_image.size
    print(f"The original image size is {width} wide x {height} tall")
    resized_image = original_image.resize(size)
    width, height = resized_image.size
    print(f"The resized image size is {width} wide x {height} tall")
    resized_image.show()
    resized_image.save(output_image_path)
    return resized_image

if __name__ == "__main__":

    #image_path = '../maps/original_int8/HM11.png'
    output_path = '../maps/resized/HM11.png'
    config_path = '../config/image_to_gridmap.yaml'
    rockfile_path = '../../../sim_env/world_plugins/config/rock_list.csv'


    # 打开配置文件
    with open(config_path, 'r') as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
    image_path = config['image_to_gridmap_demo']['image_path']
    map_size = config['image_to_gridmap_demo']['map_size']
    map_resolution = config['image_to_gridmap_demo']['resolution']
    map_pub_resolution = config['image_to_gridmap_demo']['publish_resolution']
    map_height = config['image_to_gridmap_demo']['max_height']
    # 设定的岩石分辨率
    rock_resolution = 0.1
    # 将图片放大，方便添加障碍
    print("original map size: ", map_size)
    image = read_image(image_path, (int(map_size/rock_resolution), int(map_size/rock_resolution)))
    #image.show()
    # 将放大后的图片添加障碍物
    image_obs = add_obs(image,rockfile_path,rock_resolution,map_height)

    #resized_image = resize_image(image_obs, output_path, (int(map_size), int(map_size)))

    # 生成一个低分辨率，低精度的地图
    resized_image = resize_image(image_obs, output_path, (int(map_size/map_resolution), int(map_size/map_resolution)))
    
    # 生成一个高分辨率，但低精度的地图
    resized_image = resize_image(resized_image, output_path, (int(map_size/map_pub_resolution), int(map_size/map_pub_resolution)))
