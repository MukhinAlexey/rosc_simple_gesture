#!/usr/bin/python

import rospy
import sys
from std_msgs.msg import String
import pygame
import pyttsx
import glob
import time


class Audio:
    def __init__(self):

        # Initialize ROS
        self.node_name = "audio_feedback"
        rospy.init_node(self.node_name)

        #Used for beeping noise
        pygame.init()

        # Params are used to get path to audio file
        file_path = rospy.get_param('~/file_path')
        self.audio_path = glob.glob(str(file_path) + '/*')[0]

        # Subcribed topic and timer constant are read from parameters
        gesture_detected = rospy.get_param('~/gesture_detected_topic')
        timer = rospy.get_param('~/timer')

        # Subscribe to detected gesture
        rospy.Subscriber(gesture_detected, String, self.gest_callback)

        # Initialize variables and constants
        self.match_time = timer
        self.previous_gesture = ""
        self.current_gesture = ""
        self.first_message = True
        self.last_message_time = 0
        self.current_time = 0

        rospy.loginfo("Loading Audio Feedback Node...")

    def gest_callback(self, gesture):

        # if different gesture is being detected, begin timer
        if gesture.data != self.current_gesture:
            self.last_message_time = self.millis()

        # Calculate time elapsed
        delta_time = self.millis() - self.last_message_time

        # if no gesture is detected or an old gesture is detected, cut music and update current gesture
        if gesture.data == '' or (self.previous_gesture == gesture.data):
            self.current_gesture = ""
            pygame.mixer.music.stop()

        # Else if time has not elapsed and a new gesture is detected, play music. Also, update current gesture
        elif (delta_time < self.match_time) and (self.previous_gesture != gesture.data):
            if not pygame.mixer.music.get_busy():
                pygame.mixer.music.load(self.audio_path)
                pygame.mixer.music.play()
            self.current_gesture = gesture.data

        # Else if time has elapsed and we have a new gesture, stop beeping, play voice, and update gestures and time.
        elif (gesture.data != self.previous_gesture) and (delta_time >= self.match_time):
            engine = pyttsx.init('espeak')
            engine.setProperty('rate', 100)
            while engine.isBusy():
                pygame.mixer.music.stop()
                engine.say(unicode(str(gesture.data)), "utf-8")
                engine.runAndWait()
                self.previous_gesture = gesture.data
                self.last_message_time = self.last_message_time

    @staticmethod
    def millis():
        return int(round(time.time() * 1000))


def main(args):
    try:
        Audio()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down recognition node."

if __name__ == '__main__':
    main(sys.argv)
