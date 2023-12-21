
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


        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        # self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.publishURDF()
        self.publish_static_transforms(self.urdf.child_map)


        joint = urdf.Joint(name='test_joint', joint_type='fixed', parent='base_link', child='test_link')
        joint.origin = urdf.Pose(xyz=[1.0, 1.0, 0.0], rpy=[0.0, 0.0, 0.0])
        self.urdf.add_joint(joint)
        link = urdf.Link(name='test_link',
                         visual=urdf.Visual(geometry=urdf.Cylinder(length=1, radius=1),
                                              material=urdf.Material(name='mat')))
        # print(self.urdf.links[-1])
        robot = urdf.Robot(name='test', version='1.0')
        link.inertial = urdf.Inertial(mass=1, inertia=urdf.Inertia(ixx=1, ixy=0, ixz=0, iyy=1, iyz=0, izz=1), origin=urdf.Pose(xyz=[0, 0, 0], rpy=[0, 0, 0]))
        visual = urdf.Visual(geometry=urdf.Cylinder(length=1, radius=1), material=urdf.Material(color=urdf.Color(0.3, 0.3, 0.3, 1.0), name='dark'), origin=urdf.Pose(xyz=[0, 0, 0], rpy=[0, 0, 0]))
        collision = urdf.Collision(geometry=urdf.Cylinder(length=1, radius=1), origin=urdf.Pose(xyz=[0, 0, 0], rpy=[0, 0, 0]))
        print(visual)
        link.visual = visual
        link.collision = collision
        print(link)
        self.urdf.add_link(link)
        # robot.add_aggregate('visual', visual)
        # robot.add_aggregate('collision', collision)
        # print(self.urdf.links[-1])
        # robot.add_aggregates_to_xml()
        # print(self.urdf.links[-1])
        self.publishURDF()
        self.publish_static_transforms(self.urdf.child_map)
        # self.publish_transforms(self.joints)
        # print(self.urdf.to_xml_string())

    def configure_robot(self, description):
        self.links = description.links
        self.joints = description.joints
        self.robot_name = description.name

    def publishURDF(self):
        # create transient local publisher, for latched topic
        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.urdf_publisher = self.create_publisher(String, 'robot_description_debug',  qos_profile=latching_qos) 
        new_urdf_string = self.urdf.to_xml_string()
        String_msg = String()
        String_msg.data = new_urdf_string
        # self.get_logger().info('Publishing new URDF:\n%s' % new_urdf_string)
        self.urdf_publisher.publish(String_msg)
        

    def origin_to_transofrm(self, origin):

        if origin is None:
            return TransformStamped()
        
        translation = origin.xyz
        rotation = origin.rpy
        
        transform = TransformStamped()
        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

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

    def publish_static_transforms(self, child_map):
        frame_prefix = ''

        tf_transforms = []

        for parent in child_map.keys():
            for child in child_map[parent]:

                child_index = [index for (index, item) in enumerate(self.urdf.joints) if item.name == child[0]]
                tf_transform = TransformStamped()
                if child_index:
                    tf_transform = self.origin_to_transofrm(self.joints[child_index[0]].origin)
                tf_transform.header.frame_id = frame_prefix + parent
                tf_transform.child_frame_id = frame_prefix + child[1]
                tf_transform.transform.rotation.w = 1.0
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
