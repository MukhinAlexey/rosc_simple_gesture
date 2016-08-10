#!/usr/bin/python

import rospy
import sys
import cv2
from std_msgs.msg import String
import pygame
import pyttsx
import os
import glob

class Audio:
    def __init__(self):
        self.node_name = "stratom_audio_feedback"
        rospy.init_node(self.node_name)
        pygame.init()

        # Params are used to get path to gestures folders in simple_gesture package
        gesture_path = rospy.get_param('~/gesture_path')

        file_path = rospy.get_param('~/file_path')
        self.audio_path = glob.glob(str(file_path) + '/*')[0]

        # A list of the gestures is generated
        self.list = os.listdir(gesture_path)

        rospy.on_shutdown(self.cleanup)
        rospy.Subscriber("gesture_detected", String, self.gest_callback)
        self.previous_gesture = ""
        self.count = 0
        self.counted = 0
        rospy.loginfo("Loading Audio Feedback Node...")

    def gest_callback(self, gesture):
        gesture = gesture.data
        if (gesture != '') and (self.counted < 75) and (self.previous_gesture != gesture):
            if pygame.mixer.music.get_busy() == False:
                pygame.mixer.music.load(self.audio_path)
                pygame.mixer.music.play()
            self.counted = self.timer()
        else:
            pygame.mixer.music.stop()
            self.counted = 0
            self.count = 0


        if (gesture != '') and (self.counted >= 75):
            for i in range(len(self.list)):
                if (gesture == self.list[i]) and (self.previous_gesture != self.list[i]):
                    pygame.mixer.music.stop()
                    engine = pyttsx.init('espeak')
                    engine.setProperty('rate', 100)
                    engine.say(unicode(str(self.list[i]), "utf-8"))
                    engine.runAndWait()
                    self.counted = 0
                    self.count = 0
                    self.previous_gesture = gesture
                    break


    def timer(self):
        while self.count <= 75:
            self.count = self.count + 1
            return self.count

    def cleanup(self):
        print "Shutting down recognition node."
        cv2.destroyAllWindows()


def main(args):
    try:
        Audio()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down recognition node."
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
