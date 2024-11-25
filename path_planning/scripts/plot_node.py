import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import Float64MultiArray

ax = ay = vx = vy = at = vt = []
ax_flag = ay_flag = vx_flag = vy_flag = vt_flag= at_flag = False

def at_callback(data):
    # rospy.loginfo("dt received")
    global at, at_flag
    at = data.data
    at_flag = True
    return

def ax_callback(data):
    # rospy.loginfo("ax received")
    global ax, ax_flag
    ax = data.data
    ax_flag = True
    return

def ay_callback(data):
    # rospy.loginfo("ay received")
    global ay, ay_flag
    ay = data.data
    ay_flag = True
    return

def vt_callback(data):
    # rospy.loginfo("vt received")
    global vt, vt_flag
    vt = data.data
    vt_flag = True
    return

def vx_callback(data):
    # rospy.loginfo("vx received")
    global vx, vx_flag
    vx = data.data
    vx_flag = True
    return

def vy_callback(data):
    # rospy.loginfo("vy received")
    global vy, vy_flag
    vy = data.data
    vy_flag = True
    return

if __name__ == '__main__':
    rospy.init_node('plot_node')
    rospy.Subscriber("/path/plot/at", Float64MultiArray, at_callback)
    rospy.Subscriber("/path/plot/ax", Float64MultiArray, ax_callback)
    rospy.Subscriber("/path/plot/ay", Float64MultiArray, ay_callback)
    rospy.Subscriber("/path/plot/vx", Float64MultiArray, vx_callback)
    rospy.Subscriber("/path/plot/vy", Float64MultiArray, vy_callback)
    rospy.Subscriber("/path/plot/vt", Float64MultiArray, vt_callback)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        if ax_flag and ay_flag and vx_flag and vy_flag and vt_flag and at_flag:
            rospy.loginfo("plan received, plotting ...")
            plt.figure(figsize=(6, 8))

            plt.subplot(2, 1, 1)
            plt.plot(at, ax, label='ax')
            plt.plot(at, ay, label='ay')
            plt.plot([0, vt[-1]], [+3, +3], '--')
            plt.plot([0, vt[-1]], [-3, -3], '--')
            plt.xlabel('Time (s)')
            plt.ylabel('Acceleration (m/s^2)')
            plt.xlim([0, vt[-1]])
            plt.xlim([0, vt[-1]])
            plt.legend()
            plt.title('Acceleration vs Time')

            plt.subplot(2, 1, 2)
            plt.plot(vt, vx, label='vx')
            plt.plot(vt, vy, label='vy')
            plt.plot([0, vt[-1]], [+6, +6], '--')
            plt.plot([0, vt[-1]], [-6, -6], '--')
            plt.xlabel('Time (s)')
            plt.ylabel('Velocity (m/s)')
            plt.xlim([0, vt[-1]])
            plt.xlim([0, vt[-1]])
            plt.legend()
            plt.title('Velocity vs Time')

            plt.tight_layout()
            plt.show()
            ax_flag = ay_flag = vx_flag = vy_flag = at_flag = vt_flag = False

        rate.sleep()
