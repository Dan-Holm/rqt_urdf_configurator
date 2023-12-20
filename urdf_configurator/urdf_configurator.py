
import os
import sys
import argparse
import urdf_parser_py.urdf as urdf
import rclpy
import rclpy.node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from geometry_msgs.msg import TransformStamped
import tf2_ros
# import euler to quaternion conversion
class UrdfConfigurator(rclpy.node.Node):

    def __init__(self, description_file):
        super().__init__('urdf_configurator_node') #,  automatically_declare_parameters_from_overrides=True)
        print(description_file)
        if description_file is not None:
            # If we were given a URDF file on the command-line, use that.
            self.urdf = urdf.URDF.from_xml_file(description_file)
            self.configure_robot(self.urdf)
        else:
            self.get_logger().info(
                'Waiting for robot_description to be published on the robot_description topic...')


        print(self.urdf.parent_map)
        print(self.urdf.child_map)
        exit()  
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.publishURDF(self.urdf.joints)
        self.publish_static_transforms(self.urdf.links)
        self.publish_transforms(self.joints)
        # print(self.urdf.to_xml_string())

    def configure_robot(self, description):
        self.links = description.links
        self.joints = description.joints
        self.robot_name = description.name

    def publishURDF(self, urdf):
        # create transient local publisher, for latched topic
        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.urdf_publisher = self.create_publisher(String, 'robot_description_debug',  qos_profile=latching_qos) 
        new_urdf_string = self.urdf.to_xml_string()
        String_msg = String()
        String_msg.data = new_urdf_string
        # self.get_logger().info('Publishing new URDF:\n%s' % new_urdf_string)
        self.urdf_publisher.publish(String_msg)


        pass
        

    def origin_to_transofrm(self, origin):
        translation = origin.xyz
        rotation = origin.rpy
        
        transform = TransformStamped()
        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]
        transform.transform.rotation.x = rotation[0]
        transform.transform.rotation.y = rotation[1]
        transform.transform.rotation.z = rotation[2]
        transform.transform.rotation.w = 0.0

        return transform

    def publish_transforms(self, joints):
        frame_prefix = ''
        time = self.get_clock().now().to_msg()
        tf_transforms = []

        for joint in joints:
            tf_transform = self.origin_to_transofrm(joint.origin)
            tf_transform.header.stamp = time
            tf_transform.header.frame_id = frame_prefix + joint.name
            tf_transform.child_frame_id = frame_prefix + joint.child
            tf_transforms.append(tf_transform)

        self.tf_broadcaster.sendTransform(tf_transforms)

    def publish_static_transforms(self, links):
        frame_prefix = ''

        tf_transforms = []

        for link in links:
            if len(link.visuals) == 0:
                continue
            tf_transform = self.origin_to_transofrm(link.visuals[0].origin)
            tf_transform.header.frame_id = frame_prefix + link.name
            tf_transforms.append(tf_transform)

        self.static_tf_broadcaster.sendTransform(tf_transforms)

        


def main():
    print('Hi from urdf_configurator.')

    rclpy.init()
    stripped_args = rclpy.utilities.remove_ros_args(args=sys.argv)
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'description_file', help='Robot description file to use', nargs='?', default=None)

    # Parse the remaining arguments, noting that the passed-in args must *not*
    # contain the name of the program.
    parsed_args = parser.parse_args(args=stripped_args[1:])
    uc = UrdfConfigurator('/home/daniel/master_ws/src/Danitech-master/wagon_description/urdf/wagon.urdf')

    try:
        rclpy.spin(uc)
    except KeyboardInterrupt:
        pass

    uc.destroy_node()
    rclpy.try_shutdown()
    # args = rclpy.cli.arguments.parse(['--ros-args', '--remap', 'chatter:=hello'])

    # parser = argparse.ArgumentParser(description='Process some integers.')


if __name__ == '__main__':
    main()
