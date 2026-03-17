import rospy
import sys
from std_msgs.msg import String


def main():
    rospy.init_node("mode_handler")
    # topic
    mode_topic = rospy.get_param("~mode_topic", "/navigation_mode")
    # modes
    modes = ["Mode 1", "Mode 2", "Mode 3"]
    # initialization
    mode_publisher = rospy.Publisher(mode_topic, String, queue_size=0)
    print("Mode Handler Initialized")
    print("Enter mode: 1 for efficient, 2 for safe, 3 for conservative: ")
    mode = 1
    # spin
    while not rospy.is_shutdown():
        # wait for keyboard input to change mode
        sys.stdin.flush()
        mode_input = input()
        # if timeout, continue
        if mode_input != "1" and mode_input != "2" and mode_input != "3":
            # mode_publisher.publish(modes[mode-1])
            print("Invalid mode")
            continue
        else:
            # print("input mode: ", mode_input)
            mode = int(mode_input)
            mode_publisher.publish(modes[mode-1])
            print("Mode changed to", modes[mode-1])

if __name__ == "__main__":
    main()

