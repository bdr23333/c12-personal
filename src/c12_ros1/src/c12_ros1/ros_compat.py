import rospy


class RospyLogger:
    def info(self, text):
        rospy.loginfo(text)

    def warning(self, text):
        rospy.logwarn(text)

    def error(self, text):
        rospy.logerr(text)
