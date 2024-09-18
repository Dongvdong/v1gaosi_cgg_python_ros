# sudo apt-get install python3-rosdep python3-rosinstall python3-rospkg
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image as ImageMsg
from geometry_msgs.msg import PoseStamped,Pose
from cv_bridge import CvBridge, CvBridgeError
from collections import deque

from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from gaosi_img_pose_flag.msg import PoseImgFlagMsg  # 更换为你包的名字

import std_msgs.msg

# import sys
# directory = '/home/dongdong/2project/2_3DGaosi/reduced-3dgs/'
# sys.path.append(directory)

from API_render import *


pose_queue = deque()  # Queue to store pose messages with timestamps
bridge = CvBridge()

def pose_callback(msg):
    # Store the pose message with timestamp in the queue
    pose_queue.append((msg.timestamp, msg))
    #print("收到位姿  x", msg.pose.position.x, "y", msg.pose.position.y, "z", msg.pose.position.z)

    # try:
    #     # Convert ROS image message to OpenCV image
    #     cv_image = bridge.imgmsg_to_cv2(msg.image, desired_encoding="bgr8")
    #     cv2.imshow("Received Image", cv_image)
    #     cv2.waitKey(1)
    # except Exception as e:
    #     rospy.logerr("Failed to convert image: %s", str(e))

    # rospy.loginfo("Received flag: %.2f", msg.flag.data)
    # rospy.loginfo("Received pose: Position(%.2f, %.2f, %.2f), Orientation(%.2f, %.2f, %.2f, %.2f)",
    #                 msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
    #                 msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)




# 继承模式  直接使用而非拷贝
def publish_image_with_pose_gaosi(dataset : ModelParams,
                iteration : int,
                pipeline : PipelineParams,
                ):
   

    # ============  3d 初始化 =================
    with torch.no_grad():# 丢不更新 防止高斯模型数据修改

        print("dataset._model_path 训练渲染保存的模型总路径",dataset.model_path)
        print("dataset._source_path 原始输入SFM数据路径",dataset.source_path)
        print("dataset.sh_degree 球谐系数",dataset.sh_degree)
        print("dataset.white_background 是否白色背景",dataset.sh_degree)

        cam_info = Read_caminfo_from_colmap(dataset.source_path)
        
        height, width = cam_info["height"], cam_info["width"]   
        Fovx,Fovy = cam_info["FovX"], cam_info["FovY"]   
        
        img_opencv =  np.ones((height, width, 3), dtype=np.uint8) * 0
        cv2.namedWindow('python_Node_Rendering_Img', cv2.WINDOW_NORMAL)

       # 加载渲染器
        gaussians = GaussianModel(dataset.sh_degree)

        bg_color = [1,1,1] if dataset.white_background else [0, 0, 0]
        background = torch.tensor(bg_color, dtype=torch.float32, device="cuda")
          
        # 加载什么精度模型
        model = args.models
        print("渲染实际加载的训练模型精度类型 (标准baseline 半精度quantised 半半精度half_float)",model)
        name = models_configuration[model]['name']
        quantised = models_configuration[model]['quantised']
        half_float = models_configuration[model]['half_float']
        try:
            # 选择什么训练次数模型
            model_path = dataset.model_path+"/point_cloud/iteration_"+str(iteration)+"/"
            model_path=os.path.join(model_path,name)
            print("渲染实际加载的训练模型",model_path)
            gaussians.load_ply(model_path, quantised=quantised, half_float=half_float)
                                        
        except:
            raise RuntimeError(f"Configuration {model} with name {name} not found!")
        
        # ==============  rosros 节点 ===============
        i=0
        x,y,z=0,0,0  # 手动控制的位置
        t_x,t_y,t_z=0,0,0 # ros 收到的位置 后期会更新给x,y,z 保证手动控制给的位置是从上次的位姿开始的，而不会突变。
        step_=0.1

        theta_x=0 # 旋转角度
        theta_y=0
        theta_z=0
        step_theta=1

        scale_c2w=1
        t_c2w=np.array([0, 0, 0])
        R_c2w = quaternion_to_rotation_matrix((0,0,0,1))

        # 初始化消息
        image = np.zeros((480, 640, 3), dtype=np.uint8)

        flag_ = Float64()
        flag_.data = 1.0
        
        pose_=Pose()
        pose_.position.x =0
        pose_.position.y =0
        pose_.position.z =0
        pose_.orientation.x =0
        pose_.orientation.y =0
        pose_.orientation.z =0
        pose_.orientation.w =1


      
        ImagePoseFlag_Msg = PoseImgFlagMsg()
        timestamp = rospy.Time.now()# 时间戳 同于数据同步
       
        ImagePoseFlag_Msg.timestamp = std_msgs.msg.Time()  # 初始化时间戳
        ImagePoseFlag_Msg.timestamp.data = timestamp  # 设置当前时间
        ImagePoseFlag_Msg.image = bridge.cv2_to_imgmsg(image, encoding="bgr8")
        ImagePoseFlag_Msg.flag.data  = flag_
        ImagePoseFlag_Msg.pose = pose_
        


        

        # 用于构造渲染视角
        view = Camera_view(img_id=i, 
                R=R_c2w, 
                t=t_c2w, 
                scale=scale_c2w,
                FoVx=Fovx, 
                FoVy=Fovy, 
                image_width=width,
                image_height=height)
                #df = pd.DataFrame()
        # 初期渲染一张
        img_opencv = render_img( view, gaussians, pipeline, background)
        # 用于增加文字信息后的可视化
        image = img_opencv# 原始渲染图不能被污染 要发送slam回去，新创建图可视化 cv2.UMat转换后才可以 cv2.putText

        new_img=0 

        rate = rospy.Rate(20)  # 1 Hz
        while not rospy.is_shutdown():

            new_img=0


            image = img_opencv# 原始渲染图不能被污染 要发送slam回去，新创建图可视化 cv2.UMat转换后才可以 cv2.putText

            # 设置文字的参数

            font_scale = 2 # 大小
            thickness = 2 # 粗细

            text1 ="position_xyz: " + str(round(t_x, 2))+" , "+str(round(t_y, 2)) +" , "+ str(round(t_z, 2))
            position1 = (10, 60)  # 文字的位置
            cv2.putText(image, text1, position1, cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 0, 0), thickness)

            text2 = "theta_xyz: " +  str(round(theta_x, 2))+" , "+str(round(theta_y, 2)) +" , "+ str(round(theta_z, 2))
            position2 = (10, 120)  # 文字的位置
            cv2.putText(image, text2, position2, cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 255), thickness)

            cv2.imshow('python_Node_Rendering_Img', image)
            #cv2.imshow('Rendering_Img', img_opencv)# imshow 不需要额外 cv2.UMat转换
            key = cv2.waitKey(1) & 0xFF
            
            if pose_queue: # 收到渲染请求 位姿队列不为空
                i=i+1 # 记录
                new_img=1

                timestamp, rec_pose_msg = pose_queue.popleft()
                t_x = rec_pose_msg.pose.position.x
                t_y = rec_pose_msg.pose.position.y
                t_z = rec_pose_msg.pose.position.z
                qx = rec_pose_msg.pose.orientation.x
                qy = rec_pose_msg.pose.orientation.y
                qz = rec_pose_msg.pose.orientation.z
                qw = rec_pose_msg.pose.orientation.w




                x,y,z=t_x,t_y,t_z# 将收到的位姿更新给按键变量 确保按键从现有位置开始运动
           
                scale_c2w=1
                t_c2w=np.array([t_x, t_y, t_z])
                quaternion = (qx,qy,qz,qw)
                R_c2w = quaternion_to_rotation_matrix(quaternion)
                # # 从旋转矩阵获取欧拉角
                roll, pitch, yaw = rotation_matrix_to_euler_angles(R_c2w)
                theta_x,theta_y,theta_z = roll, pitch, yaw

                
                flag_ = rec_pose_msg.flag
                pose_ = rec_pose_msg.pose


                #print(f"绕 X 轴的角度 滚转会使物体的左侧和右侧倾斜 (roll): {roll:.2f}°")
                #print(f"绕 Y 轴的角度 俯仰会使物体的前端向上或向下移动 (pitch): {pitch:.2f}°")
                #print(f"绕 Z 轴的角度 偏航会使物体的前端向左或向右转动 (yaw): {yaw:.2f}°")
            else:# 如果没有收到渲染请求 是否手动给了渲染位姿

                if key == 27:  # 按下 'q' 键
                    print("退出")
                    break
                elif key == ord('w'):  # 按下 's' 键
                    print("x前进")
                    x=x+step_
                    i=i+1
                    new_img=1                        
                elif key == ord('s'):  # 按下 's' 键
                    print("x后退")
                    x=x-step_
                    i=i+1
                    new_img=1   
                elif key == ord('a'):  # 按下 's' 键
                    print("y前进")
                    y=y+step_
                    i=i+1
                    new_img=1  
                elif key == ord('d'):  # 按下 's' 键
                    print("y后退")
                    y=y-step_
                    i=i+1
                    new_img=1   
                elif key == ord('q'):  # 按下 's' 键
                    print("z前进")
                    z=z+step_
                    i=i+1
                    new_img=1  
                elif key == ord('e'):  # 按下 's' 键
                    print("z后退")
                    z=z-step_
                    i=i+1
                    new_img=1  

                elif key == ord('i'):  # 按下 's' 键
                    print("x旋转+")
                    theta_x=theta_x+step_theta
                    if(theta_x>360 or theta_x<-360): theta_x=0
                    i=i+1
                    new_img=1  
                elif key == ord('k'):  # 按下 's' 键
                    print("x旋转-")
                    theta_x=theta_x-step_theta
                    if(theta_x>360 or theta_x<-360): theta_x=0
                    i=i+1
                    new_img=1  

                elif key == ord('j'):  # 按下 's' 键
                    print("y旋转+")
                    theta_y=theta_y+step_theta
                    if(theta_y>360 or theta_y<-360): theta_y=0
                    i=i+1
                    new_img=1  
                elif key == ord('l'):  # 按下 's' 键
                    print("y旋转-")
                    theta_y=theta_y-step_theta
                    if(theta_y>360 or theta_y<-360): theta_y=0
                    i=i+1
                    new_img=1  

                elif key == ord('u'):  # 按下 's' 键
                    print("z旋转+")
                    theta_z=theta_z+step_theta
                    if(theta_z>360 or theta_z<-360): theta_z=0
                    i=i+1
                    new_img=1  
                elif key == ord('o'):  # 按下 's' 键
                    print("z旋转-")
                    theta_z=theta_z-step_theta
                    if(theta_z>360 or theta_z<-360): theta_z=0
                    i=i+1
                    new_img=1  


                if new_img==1: 

                    t_x,t_y,t_z = x,y,z# 将按键变量更新给收到的位姿变量 确保图像可以显示刷新当前位置

                    # # 示例角度（以弧度为单位）
                    theta_x_pi = np.radians(theta_x)  # 30度
                    theta_y_pi = np.radians(theta_y)  # 45度
                    theta_z_pi = np.radians(theta_z)  # 60度

                    # # 计算旋转矩阵
                    R_c2w = combined_rotation_matrix(theta_x_pi, theta_y_pi, theta_z_pi)
                    # 相机到世界的旋转矩阵
                    # R_c2w = np.array([
                    #     [1.0, 0.0, 0.0],
                    #     [0.0, 1.0, 0.0],
                    #     [0.0, 0.0, 1.0]
                    # ])
                    # print("旋转矩阵 R:")
                    # print(R)

                    # 相机到世界的平移矩阵 也就是相机在世界坐标系下的位置
                    t_c2w=np.array([x, y, z])
                    scale_c2w=1
                    
                    q_c2w=rotation_matrix_to_quaternion(R_c2w)

                    #timestamp = rospy.Time.now()

                    pose_.position.x =t_c2w[0]
                    pose_.position.y =t_c2w[1]
                    pose_.position.z =t_c2w[2]
                    pose_.orientation.x =q_c2w[0]
                    pose_.orientation.y =q_c2w[1]
                    pose_.orientation.z =q_c2w[2]
                    pose_.orientation.w =q_c2w[3]

                    


            if new_img==1:
                view = Camera_view(img_id=i, 
                                R=R_c2w, 
                                t=t_c2w, 
                                scale=scale_c2w,
                                FoVx=Fovx, 
                                FoVy=Fovy, 
                                image_width=width,
                                image_height=height)


                #df = pd.DataFrame()
                img_opencv = render_img( view, gaussians, pipeline, background)
                #random_image = np.random.randint(0, 256, (480, 640, 3), dtype=np.uint8)

                try:


 

                    ImagePoseFlag_Msg.timestamp.data = rospy.Time.now()
                    ImagePoseFlag_Msg.pose = pose_
                    ImagePoseFlag_Msg.flag.data  = flag_.data

                    ImagePoseFlag_Msg.image = bridge.cv2_to_imgmsg(image, "bgr8") 


                    pub_ImgPoseFlag.publish(ImagePoseFlag_Msg)

                    # Publish pose and image
                    #pose_pub.publish(pose_msg)
                    #image_pub.publish(image_msg)
                    print("图像数据发送", " 位姿xyz ", x, y, z)
                except CvBridgeError as e:
                    rospy.logerr(f'CvBridge Error: {e}')

            rate.sleep()



if __name__ == '__main__':
    # ============  3d 初始化 =================
    parser = ArgumentParser(description="渲染测试脚本")
    model = ModelParams(parser, sentinel=True)
    pipeline = PipelineParams(parser)
    parser.add_argument("--iteration", default=30000, type=int)
    parser.add_argument("--models",    default='baseline',type=str)  #'baseline','quantised'   'quantised_half' 
    parser.add_argument("--quiet", action="store_true") #标记以省略写入标准输出管道的任何文本。
    args = get_combined_args(parser) # 从cfg_args加载路径
    safe_state(args.quiet)
    #render_sets_handMode(model.extract(args), args.iteration, pipeline.extract(args))

    # ==============  rosros 节点初始化 ===============
    rospy.init_node('node2', anonymous=True)

    rospy.Subscriber('slam/image_pose_topic', PoseImgFlagMsg, pose_callback)

    global pub_ImgPoseFlag
    pub_ImgPoseFlag = rospy.Publisher('render/image_pose_topic', PoseImgFlagMsg, queue_size=10)


    publish_image_with_pose_gaosi(model.extract(args), args.iteration, pipeline.extract(args))
