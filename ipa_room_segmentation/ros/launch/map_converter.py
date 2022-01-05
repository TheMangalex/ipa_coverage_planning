#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid
import actionlib
from ipa_building_msgs.msg import MapSegmentationAction, MapSegmentationGoal
#rosrun hector_compressed_map_transport map_to_image_node
#rosrun image_view image_view image:=/map_image/full _image_transport:=compressed
#http://wiki.ros.org/hector_compressed_map_transport

rospy.init_node("segmentation_map_converter")
client = actionlib.SimpleActionClient('/room_segmentation/room_segmentation_server', MapSegmentationAction)
client.wait_for_server()

def callback(msg):
    seg = MapSegmentationGoal()
    seg.input_map.data = msg.data
    seg.input_map.encoding = "mono8"
    seg.input_map.height = msg.info.height
    seg.input_map.width = msg.info.width
    seg.input_map.step = 400
    #seg.input_map = msg.data
    seg.map_origin = msg.info.origin
    seg.map_resolution = msg.info.resolution
    seg.return_format_in_pixel = False
    seg.return_format_in_meter = True
    seg.robot_radius = 1.0
    seg.room_segmentation_algorithm = 1
    client.send_goal(seg)
    client.wait_for_result()




sub = rospy.Subscriber("/map", OccupancyGrid, callback)



rospy.spin()


