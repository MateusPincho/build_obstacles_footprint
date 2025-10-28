#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import PolygonStamped, Point32, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


class PolygonBuilderNode(Node):
    def __init__(self):
        super().__init__('polygon_builder_node')

        # --- Parameters ---
        # this parameters is for .yaml parsing, which is not working correctly...
        # self.declare_parameter('target_frame', 'World')
        # self.declare_parameter('tag_prefix', 'tag')
        # self.declare_parameter('obstacles', [])

        # define the obstacles parameters
        self.target_frame = "World"
        self.tag_prefix = "tag"
        self.obstacle_definitions = [
            {
                'name': "wall",
                # tag ids, in the order that it should be connected
                'ids': [241, 242, 243, 244]
            }
            # if you want more obstacles, just add below:
            # ,{
            #     'name': "column",
            #     'ids': [10, 11, 12]
            # }
        ]
        
        if not self.obstacle_definitions:
            self.get_logger().error("Nenhum obstáculo definido! Verifique o 'obstacles.yaml'.")
            return
        
        # --- TF2 ---
        # stores the transformations received over the /tf and /tf_static topics.
        self.tf_buffer = tf2_ros.Buffer()
        # subscribes to these topics and populates the Buffer
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- Publishers ---
        # publish the markers for RViZ
        self.marker_pub = self.create_publisher(MarkerArray, '/obstacle_markers', 10)
        # create PolygonStamped publishers for each obstacle in scene
        self.polygon_pubs = {}
        for obst in self.obstacle_definitions:
            name = obst['name']
            topic = f"~/obstacle_{name}"
            self.polygon_pubs[name] = self.create_publisher(PolygonStamped, topic, 10)

        # --- Timer ---
        # execute the callback fuction 10 times per second
        self.timer = self.create_timer(0.1, self.build_polygons_callback)

    def build_polygons_callback(self):
        '''
        Callback function that listens to the tf tree and publish the Polygon and MarkerArray msgs
        '''
        # create an empty list of markers
        marker_array_msg = MarkerArray()
        # get the actual ROS time
        now = self.get_clock().now()

        # iterate for each obstacle defined on the YAML file
        for i, obst_def in enumerate(self.obstacle_definitions):
            name = obst_def['name']
            ids = obst_def['ids']
            
            # create a polygon msg for this obstacle
            polygon_msg = PolygonStamped()
            polygon_msg.header.stamp = now.to_msg()
            polygon_msg.header.frame_id = self.target_frame

            points_stamped = [] # PolygonStamped (Point32)
            points_marker = []  # Marker (Point)
            all_tags_found = True

            # iterate in each tag ID of this obstacle
            for tag_id in ids:
                tag_frame = f"{self.tag_prefix}{tag_id}"
                try:
                    trans = self.tf_buffer.lookup_transform(
                        self.target_frame,
                        tag_frame,
                        rclpy.time.Time(), 
                        timeout=Duration(seconds=0.05)
                    )
                    # get the marker position
                    pos = trans.transform.translation
                    # create a vertex for the polygon msg
                    pt32 = Point32()
                    pt32.x = float(pos.x)
                    pt32.y = float(pos.y)
                    pt32.z = float(pos.z)
                    points_stamped.append(pt32)
                    
                    # create a vertex for the Marker msg
                    pt = Point()
                    pt.x = float(pos.x)
                    pt.y = float(pos.y)
                    pt.z = float(pos.z)
                    points_marker.append(pt)
                
                except (LookupException, ConnectivityException, ExtrapolationException) as e:
                    all_tags_found = False
                    break
            
            # if it is possible to found the tags
            if all_tags_found and points_stamped:
                # publish the PolygonStamped msg
                polygon_msg.polygon.points = points_stamped
                self.polygon_pubs[name].publish(polygon_msg)
                
                # create and publish the marker array msg
                marker = self.create_obstacle_marker(name, i, polygon_msg.header, points_marker)
                marker_array_msg.markers.append(marker)
                self.marker_pub.publish(marker_array_msg)
        
    def create_obstacle_marker(self, name, marker_id, header, points):
        """
        Function for creating the polygon marker used in RViZ visualization.
        """
        marker = Marker()
        marker.header = header
        marker.ns = "obstacle_footprints" # group all the objects on RViZ 
        marker.id = marker_id             # unique obstacle id 
        marker.type = Marker.LINE_STRIP   # the line type between points 
        marker.action = Marker.ADD        # add the marker
        
        # define the marker vertex 
        marker.points = points
        # conect the last to the first point 
        if len(points) > 1:
            marker.points.append(points[0]) 

        # define the visual properties 
        marker.scale.x = 0.05  
        marker.color = ColorRGBA()
        marker.color.r = 0.0
        marker.color.g = 1.0  
        marker.color.b = 0.0
        marker.color.a = 0.8  
        
        # define the lifetime - should be larger than the callback timer
        marker.lifetime = Duration(seconds=0.5).to_msg()
        
        return marker
    
def main(args=None):
    rclpy.init(args=args)
    node = PolygonBuilderNode() 
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()