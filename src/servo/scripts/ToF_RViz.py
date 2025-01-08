#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import UInt16MultiArray, Header
from sensor_msgs.msg import PointCloud2
import ros_numpy

class Rviz:

    def __init__(self):
        # Quaternions based on CAD model
        self.Q= [np.array([0.707, 0., 0.707, 0.]), 
                 np.array([0., 0., 1., 0.]), 
                 np.array([0.707, 0., -0.707, 0.]), 
                 np.array([1., 0., 0., 0.])]
        # Transform based on CAD model, in mm
        self.T= [np.array([-30.165, -174.965, 3.754])/1000,
            np.array([1.498, 174.965, 28.977])/1000,
            np.array([-31.233, 174.965, 0.217])/1000,
            np.array([-2.542, 174.965, -32.421])/1000]
        rospy.Subscriber('/sensor/filtered_data', UInt16MultiArray, self.rviz_callback)
        self.pcl_pub = rospy.Publisher("/ToF_RViz", PointCloud2, queue_size=10)

    def rviz_callback(self, msg):
        print(msg)
        readings = np.array(msg.data).ravel()
        # print(type(msg.data), type(readings))
        R = [self.quaternion_to_rotation_matrix(q) for q in self.Q]
        sensor_resolution = [8, 8]
        fov_h = 60
        fov_v = 60

        transformed_points = np.empty((8, 8, 3))
        for i in range(4):
            distance = readings[int(0 + i*64) : int(64 + i*64)].reshape((8,8))
            # distance = np.ones((8,8))
            points = self.get_tof_angles(sensor_resolution, fov_h, fov_v, distance)
            points = points.reshape((64, 3))
            #print(points.shape)
            
            for i in range(64):
                points[i, :] = self.transform(points[i, :], R[0], self.T[0])
            points = points.reshape((8, 8, 3))

            transformed_points = np.concatenate((transformed_points, points))
        # print(np.shape(transformed_points)) # why is the shape (40,8,3) shouldn't it be (64,3)
        # print(transformed_points.tolist()) #still pretty far
        self.publishPointCloud2(transformed_points)
    
    #points have to be a 2d array
    def publishPointCloud2(self, points):
        header = Header()
        header.frame_id = "map"
        header.stamp = rospy.Time.now()
        points = points.reshape(-1, points.shape[-1])
        data = np.array([tuple(p) for p in points], dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32)
        ])
        # print(data.tolist())
        pointcloud_msg = ros_numpy.msgify(PointCloud2, data, stamp=header.stamp, frame_id=header.frame_id)
        self.pcl_pub.publish(pointcloud_msg)

    def get_tof_angles(self, sensor_resolution, fov_h, fov_v, distance):
        h = np.arange(0, fov_h, fov_h/sensor_resolution[0]) + fov_h / 16 - fov_h / 2
        v = np.arange(0, fov_v, fov_v/sensor_resolution[1]) + fov_v / 16 - fov_v / 2
        H, V = np.meshgrid(h, v)
        points = np.stack((H, V), axis = -1)

        return self.pixel_to_3d_pose(points, distance)

    def pixel_to_3d_pose(self, pixel_angles, distance):
        x = distance * np.tan(np.radians(pixel_angles[:,:,0]))
        y = distance * np.tan(np.radians(pixel_angles[:,:,1]))
        z = distance
        return np.stack((x, y, z), axis = -1)

    def test(self, points):
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        ax.scatter(points[:,:,0], points[:,:,1], points[:,:,2])
        ax.set_xlim(-100, 100)
        ax.set_ylim(-100, 100)
        ax.set_zlim(-100, 100)
        plt.show()

    def quaternion_to_rotation_matrix(self, q):
        """
        Convert a quaternion into a 3x3 rotation matrix.
        
        :param q: List or array of quaternion [w, x, y, z]
        :return: 3x3 rotation matrix
        """
        w, x, y, z = q
        
        # Compute rotation matrix
        R = np.array([[1 - 2*(y**2 + z**2), 2*(x*y - z*w), 2*(x*z + y*w)],
                    [2*(x*y + z*w), 1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
                    [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x**2 + y**2)]])
        
        return R

    def transform(self, points, R, T):
        # Global point (after transformation)
        return np.dot(R, points) + T

if __name__ == "__main__":
    rospy.init_node('ToF_Viz', anonymous=True)
    rviz = Rviz()
    rospy.loginfo("Launched PointCloud Visualizer")
    rospy.spin()
    #test(transformed_points)