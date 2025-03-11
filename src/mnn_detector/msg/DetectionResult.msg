#检测器结果
std_msgs/Header header
CategoryInfo[] category_infos   #类别信息
float32 all_time                #检测一次所需时间s
float32 img_time                #获取图像时间
float32 detect_time             #检测推理时间