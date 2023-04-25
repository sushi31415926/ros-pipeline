import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import geometry_msgs.msg

rospy.init_node('tmp', anonymous=True)
publisher = rospy.Publisher(
        "pose", geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=1000)

def callback(data):
    # Create and fill pose message for publishing
    pose = geometry_msgs.msg.PoseWithCovarianceStamped()
    pose.header = data.header
    pose.pose.pose.position.x = data.pose.pose.position.x
    pose.pose.pose.position.y = data.pose.pose.position.y
    pose.pose.pose.position.z = data.pose.pose.position.z
    pose.pose.pose.orientation.x = data.pose.pose.orientation.x
    pose.pose.pose.orientation.y = data.pose.pose.orientation.y
    pose.pose.pose.orientation.z = data.pose.pose.orientation.z
    pose.pose.pose.orientation.w = data.pose.pose.orientation.w


    # Since tf transforms do not have a covariance, pose is filled with
    # a zero covariance.
    pose.pose.covariance = [0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0]

    publisher.publish(pose)


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    rospy.Subscriber('/lio_sam/mapping/odometry', Odometry, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
