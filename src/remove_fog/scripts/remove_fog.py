#!/usr/bin/env python
import os
import roslib
import rospy

from std_msgs.msg import Header
from std_msgs.msg import String
from sensor_msgs.msg import Image

import cv2
import numpy as np
import argparse  


class SubscribeAndPublish:
		def __init__(self):
				print("waiting image")
				self.all_obstacle_str=''
				self.sub1_name="/image_get"
				self.sub1= rospy.Subscriber(self.sub1_name, Image,self.callback)
				self.pub1_name="refog_image"
				self.pub1= rospy.Publisher(self.pub1_name, Image,queue_size=1)
				self.img = []
				
		def callback(self,data):
				print('callback')
				global i
				img = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
				img = img[:,:,::-1]
				img = cv2.resize(img, (720, 405), interpolation=cv2.INTER_LINEAR)
				#img = cv2.medianBlur(img, 5)
				
				#kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]], np.float32) #定义一个核
				#img = cv2.filter2D(img, -1, kernel=kernel) 
				#print(img.shape)
				m = self.deHaze(img/255)*255
				#print(dtype)  # 处理完要float64 还有负值
				m = np.clip(m,a_min=0,a_max=255) #先归一化到0-2500消除负数
				#print(m)
				cv2.imwrite("/home/cxl/ros_yolov5/src/datasets/result/2.png",m)
				m = m.astype(np.int8)  #在转化为8位数
				i+=1
				#print(m.dtype) 
				self.publish_image(self.pub1,m,'base_link')
				#print('finish remove fog')
        
		def publish_image(self,pub, data, frame_id='base_link'):
				assert len(data.shape) == 3, 'len(data.shape) must be equal to 3.'
				print(data.shape)
				header = Header(stamp=rospy.Time.now())
				header.frame_id = frame_id
			
				msg = Image()
				msg.height = data.shape[0]
				msg.width = data.shape[1]
				msg.encoding = 'rgb8'  #传入数据要求是8位
				msg.data = np.array(data).tostring()
				msg.header = header
				msg.step = msg.width *1*3
				#print("**************************")
	
				pub.publish(msg)
				#print("---------------------------")
        
		def zmMinFilterGray(self,src, r=7):
				'''最小值滤波，r是滤波器半径'''
				'''if r <= 0:
					return src
				h, w = src.shape[:2]
				I = src
				res = np.minimum(I , I[[0]+range(h-1) , :])
				res = np.minimum(res, I[range(1,h)+[h-1], :])
				I = res
				res = np.minimum(I , I[:, [0]+range(w-1)])
				res = np.minimum(res, I[:, range(1,w)+[w-1]])
				return zmMinFilterGray(res, r-1)'''
				return cv2.erode(src, np.ones((2*r+1, 2*r+1)))           #使用opencv的erode函数更高效
		def guidedfilter(self,I, p, r, eps):
				'''引导滤波，直接参考网上的matlab代码'''
				height, width = I.shape
				m_I = cv2.boxFilter(I, -1, (r,r))
				m_p = cv2.boxFilter(p, -1, (r,r))
				m_Ip = cv2.boxFilter(I*p, -1, (r,r))
				cov_Ip = m_Ip-m_I*m_p
				
				m_II = cv2.boxFilter(I*I, -1, (r,r))
				var_I = m_II-m_I*m_I
				
				a = cov_Ip/(var_I+eps)
				b = m_p-a*m_I
				
				m_a = cv2.boxFilter(a, -1, (r,r))
				m_b = cv2.boxFilter(b, -1, (r,r))
				return m_a*I+m_b
			
		def getV1(self,m, r, eps, w, maxV1): #输入rgb图像，值范围[0,1]
				'''计算大气遮罩图像V1和光照值A, V1 = 1-t/A'''
				V1 = np.min(m,2)                     #得到暗通道图像
				V1 = self.guidedfilter(V1, self.zmMinFilterGray(V1,7), r, eps)   #使用引导滤波优化
				bins = 2000    #2000
				ht = np.histogram(V1, bins)               #计算大气光照A
				d = np.cumsum(ht[0])/float(V1.size)
				for lmax in range(bins-1, 0, -1):
					if d[lmax]<=0.999:
						break
				A = np.mean(m,2)[V1>=ht[1][lmax]].max()
						
				V1 = np.minimum(V1*w, maxV1)          #对值范围进行限制
					
				return V1,A
			
		def imgBrightness(self,img1, a, b): 
				h, w, ch = img1.shape#获取shape的数值，height和width、通道
				#新建全零图片数组src2,将height和width，类型设置为原图片的通道类型(色素全为零，输出为全黑图片)
				src2 = np.zeros([h, w, ch], img1.dtype)
				dst = cv2.addWeighted(img1, a, src2, 1-a, b)#addWeighted函数说明如下
				return dst


		def deHaze(self,img, r=81, eps=0.001, w=0.95, maxV1=0.80, bGamma=False): #r=81, eps=0.001, w=0.95, maxV1=0.80, bGamma=False
				
				Y = np.zeros(img.shape)
				V1,A = self.getV1(img, r, eps, w, maxV1)        #得到遮罩图像和大气光照
				for k in range(3):
					Y[:,:,k] = (img[:,:,k]-V1)/(1-V1/A)      #颜色校正
				Y = np.clip(Y, 0, 1)
				#if bGamma:
				#Y = Y**(np.log(0.3)/np.log(Y.mean()))    #gamma校正, 越大越亮  0.3
				#Y = self.imgBrightness(Y, 2.8, -0.15)   #第一个值是亮度，第二个是对比度 2.8 -0.15
				return Y


def init():
	rospy.init_node('remove_fog',anonymous=True)
	t = SubscribeAndPublish()
	rospy.spin()





  
if __name__ == '__main__':
		i=1
		init()
  

