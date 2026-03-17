#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from message_filters import ApproximateTimeSynchronizer, Subscriber

class OccupancyGridMerger:
    def __init__(self):
        rospy.init_node("occupancy_grid_merger", anonymous=True)
        
        # Parameters
        self.weight1 = rospy.get_param("~weight1", 0.5)
        self.weight2 = rospy.get_param("~weight2", 0.5)
        self.topic1 = rospy.get_param("~topic1", "/mode3/slope_cost")
        self.topic2 = rospy.get_param("~topic2", "/mode3/roughness_cost")
        self.output_topic = rospy.get_param("~output_topic", "/mode3/occupancy_grid")

        # Subscribers and Publisher
        self.sub1 = Subscriber(self.topic1, OccupancyGrid)
        self.sub2 = Subscriber(self.topic2, OccupancyGrid)
        self.pub = rospy.Publisher(self.output_topic, OccupancyGrid, queue_size=10)

        # Synchronizer
        self.sync = ApproximateTimeSynchronizer([self.sub1, self.sub2], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.callback)

    def callback(self, map1, map2):
        if (map1.info.width != map2.info.width or
            map1.info.height != map2.info.height or
            map1.info.resolution != map2.info.resolution):
            rospy.logerr("Map dimensions or resolution do not match!")
            return

        merged_map = OccupancyGrid()
        merged_map.header.stamp = rospy.Time.now()
        merged_map.header.frame_id = map1.header.frame_id  # Assuming both maps share the same frame
        merged_map.info = map1.info

        merged_data = []
        for v1, v2 in zip(map1.data, map2.data):
            if v1 == -1 and v2 == -1:
                merged_data.append(-1)
            elif v1 == -1:
                merged_data.append(v2)
            elif v2 == -1:
                merged_data.append(v1)
            else:
                merged_value = int(self.weight1 * v1 + self.weight2 * v2)
                merged_data.append(merged_value)

        merged_map.data = merged_data
        self.pub.publish(merged_map)

if __name__ == "__main__":
    try:
        node = OccupancyGridMerger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
