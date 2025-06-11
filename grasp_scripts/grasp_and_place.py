"""


https://docs.ros.org/en/foxy/How-To-Guides/Sync-Vs-Async.html

"""
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from robotiq_sock_driver_msgs.msg import GripperCmd
from std_srvs.srv import Trigger
import time
import requests
import threading


class SingleViewGraspClient(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.urscript_publisher = self.create_publisher(String, '/urscript_interface/script_command', 10)
        self.grippercmd_publisher = self.create_publisher(GripperCmd, '/gripper/cmd', 10)

        # Create services
        self.grasp_and_place_service = self.create_service(
            Trigger, 
            '/grasp_scripts/grasp_and_place', 
            self.grasp_and_place_service_callback)

        self.grasp_and_place_three_view_service = self.create_service(
            Trigger, 
            '/grasp_scripts/grasp_and_place_three_view', 
            self.grasp_and_place_three_view_service_callback)

        self.capture_point_cli = self.create_client(Trigger, '/multiview_saver/capture_point')
        self.restart_count_cli = self.create_client(Trigger, '/multiview_saver/restart_count')


    def grasp_and_place_service_callback(self, req: Trigger.Request, resp: Trigger.Response):
        """ """
        msg = String()
        gripper_msg = GripperCmd()
        empty_req = Trigger.Request()
        
        gripper_msg.emergency_release = False
        gripper_msg.emergency_release_dir = True
        gripper_msg.stop = False
        gripper_msg.position = 0.
        gripper_msg.speed = 100.
        gripper_msg.force = 20.

        # Move to point #1 (middle)
        msg.data = 'movel(p[-0.4917, -0.1324, 0.5, 2.222, 2.224, 0.002], a=1.2, v=0.25, r=0)'
        self.urscript_publisher.publish(msg)
        time.sleep(3)

        # Restart save count
        self.get_logger().info('Restarting save count.')
        start_t = time.time()
        future = self.restart_count_cli.call_async(empty_req)
        #t = threading.Thread(target=self.restart_count_cli.call_async, args=(empty_req,))
        #t.start()
        #while t.is_alive():
        #    time.sleep(0.01)
        #t.join()
        d = time.time() - start_t
        self.get_logger().info('Restarting save count cost: {}'.format(d))
        time.sleep(3)

        # Take a shot
        self.get_logger().info('Getting scene pointcloud.')
        #future = self.capture_point_cli.call(empty_req)
        future = self.capture_point_cli.call_async(empty_req)
        time.sleep(6)

        # Make a precdition
        r = requests.post('http://127.0.0.1:5566/dev/predict/result/', json={'view': '1'})
        grasp_str = r.text
        self.get_logger().info('Generated pose: [pos_x, pos_y, pos_z, rotvec_x, rotvec_y, rotvec_z] = {}'.format(grasp_str))

        grasp = grasp_str.split(',')
        time.sleep(2)

        # Move to grip
        msg.data = 'movel(p[{}, {}, {}, {}, {}, {}], a=1.2, v=0.25, r=0)'.format(grasp[0], grasp[1], 0.2, grasp[3], grasp[4], grasp[5])
        self.urscript_publisher.publish(msg)
        time.sleep(4)
        
        msg.data = 'movel(p[{}, {}, {}, {}, {}, {}], a=1.2, v=0.25, r=0)'.format(grasp[0], grasp[1], grasp[2], grasp[3], grasp[4], grasp[5])
        self.urscript_publisher.publish(msg)
        time.sleep(2)

        # Close gripper
        gripper_msg.position = 0.
        self.grippercmd_publisher.publish(gripper_msg)
        time.sleep(1.5)

        # Move to point #1 (middle)
        msg.data = 'movel(p[-0.4917, -0.1324, 0.2, 2.222, 2.224, 0.002], a=1.2, v=0.25, r=0)'
        self.urscript_publisher.publish(msg)
        time.sleep(3)

        # Move to placement point
        msg.data = 'movel(p[-0.45, -0.5, 0.2, 2.222, 2.224, 0.002], a=1.2, v=0.25, r=0)'
        self.urscript_publisher.publish(msg)
        time.sleep(3)

        msg.data = 'movel(p[-0.45, -0.5, -0.05, 2.222, 2.224, 0.002], a=1.2, v=0.25, r=0)'
        self.urscript_publisher.publish(msg)
        time.sleep(2)

        # Open gripper
        gripper_msg.position = 50.
        self.grippercmd_publisher.publish(gripper_msg)
        time.sleep(1.5)

        # Move to point #1 (middle)
        msg.data = 'movel(p[-0.4917, -0.1324, 0.5, 2.222, 2.224, 0.002], a=1.2, v=0.25, r=0)'
        self.urscript_publisher.publish(msg)

        resp.success = True
        resp.message = 'Success.'
        return resp


    def grasp_and_place_three_view_service_callback(self, req: Trigger.Request, resp: Trigger.Response):
        """ """
        msg = String()
        gripper_msg = GripperCmd()
        empty_req = Trigger.Request()
        
        gripper_msg.emergency_release = False
        gripper_msg.emergency_release_dir = True
        gripper_msg.stop = False
        gripper_msg.position = 0.
        gripper_msg.speed = 100.
        gripper_msg.force = 100.

        # Restart save count
        self.get_logger().info('Restarting save count.')
        future = self.restart_count_cli.call_async(empty_req)
        time.sleep(3)

        # Move to point #1 (middle)
        msg.data = 'movel(p[-0.4917, -0.1324, 0.5, 2.222, 2.224, 0.002], a=1.2, v=0.25, r=0)'
        self.urscript_publisher.publish(msg)
        time.sleep(3)

        # Take a shot
        self.get_logger().info('Getting scene pointcloud #1.')
        future = self.capture_point_cli.call_async(empty_req)
        time.sleep(7)

        # Move to point #2 (left)
        msg.data = 'movel(p[-0.40337, -0.36476, 0.36938, -1.86123942, -1.86626076, -0.61672924], a=1.2, v=0.25, r=0)'
        self.urscript_publisher.publish(msg)
        time.sleep(3)

        # Take a shot
        self.get_logger().info('Getting scene pointcloud #2.')
        future = self.capture_point_cli.call_async(empty_req)
        time.sleep(7)

        # Move to point #3: right
        msg.data = 'movel(p[-0.40063, 0.15345, 0.34628, 1.85695284,  1.85690571, -0.60882747], a=1.2, v=0.25, r=0)'
        self.urscript_publisher.publish(msg)
        time.sleep(4)

        # Take a shot
        self.get_logger().info('Getting scene pointcloud #3.')
        future = self.capture_point_cli.call_async(empty_req)
        time.sleep(7)

        # Make a precdition
        r = requests.post('http://127.0.0.1:5566/dev/predict/result/', json={'view': '3'})
        grasp_str = r.text
        self.get_logger().info('Generated pose: [pos_x, pos_y, pos_z, rotvec_x, rotvec_y, rotvec_z] = {}'.format(grasp_str))

        grasp = grasp_str.split(',')
        time.sleep(2)

        # Move to grip
        msg.data = 'movel(p[{}, {}, {}, {}, {}, {}], a=1.2, v=0.25, r=0)'.format(grasp[0], grasp[1], 0.2, grasp[3], grasp[4], grasp[5])
        self.urscript_publisher.publish(msg)
        time.sleep(4)
        
        msg.data = 'movel(p[{}, {}, {}, {}, {}, {}], a=1.2, v=0.25, r=0)'.format(grasp[0], grasp[1], grasp[2], grasp[3], grasp[4], grasp[5])
        self.urscript_publisher.publish(msg)
        time.sleep(2)

        # Close gripper
        gripper_msg.position = 0.
        self.grippercmd_publisher.publish(gripper_msg)
        time.sleep(1.5)

        # Move to point #1 (middle)
        msg.data = 'movel(p[-0.4917, -0.1324, 0.5, 2.222, 2.224, 0.002], a=1.2, v=0.25, r=0)'
        self.urscript_publisher.publish(msg)
        time.sleep(3)

        # Move to placement point
        msg.data = 'movel(p[-0.45, -0.5, 0.2, 2.222, 2.224, 0.002], a=1.2, v=0.25, r=0)'
        self.urscript_publisher.publish(msg)
        time.sleep(2.5)

        msg.data = 'movel(p[-0.45, -0.5, -0.05, 2.222, 2.224, 0.002], a=1.2, v=0.25, r=0)'
        self.urscript_publisher.publish(msg)
        time.sleep(2)

        # Open gripper
        gripper_msg.position = 50.
        self.grippercmd_publisher.publish(gripper_msg)
        time.sleep(1.5)

        # Move to point #1 (middle)
        msg.data = 'movel(p[-0.4917, -0.1324, 0.5, 2.222, 2.224, 0.002], a=1.2, v=0.25, r=0)'
        self.urscript_publisher.publish(msg)
        time.sleep(3)

        resp.success = True
        resp.message = 'Success.'
        return resp



def main(args=None):

    rclpy.init(args=args)

    client = SingleViewGraspClient()

    #spin_thread = Thread(target=rclpy.spin, args=(client,))
    #spin_thread.start()

    rclpy.spin(client)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    client.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()
