__author__ = 'caja'
import rospy
import time
import shlex
import subprocess
from threading import Thread
from BAL.Interfaces.Runnable import Runnable

TIME_OUT = 5000


class KeepAliveHandler(Runnable):
    is_init = False

    def __init__(self, topic_name, msg_type):
        if not KeepAliveHandler.is_init:
            KeepAliveHandler.is_init = True
            self._watch_dog_time = int(round(time.time() * 1000))
            rospy.Subscriber(topic_name, msg_type, self.callback_to_watch)
            Thread(target=self.run, args=()).start()
        else: pass

    def run(self):
        rate = rospy.Rate(50)
        send_err = False
        while not rospy.is_shutdown() and not send_err:
            if (int(round(time.time() * 1000)) - self._watch_dog_time) > TIME_OUT:
                rospy.logerr("RiC Board is not responding")
                subprocess.Popen(shlex.split("pkill -f ros"))
                send_err = True
            rate.sleep()

    def callback_to_watch(self, msg):
        self._watch_dog_time = int(round(time.time() * 1000))
