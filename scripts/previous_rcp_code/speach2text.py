#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import speech_recognition as sr
import os


def googlesr():
    rospy.init_node('speach2text', anonymous=True)
    pub = rospy.Publisher('/speach2text', String, queue_size=10)

    while not rospy.is_shutdown():
        # obtain audio from the microphone
        r = sr.Recognizer()
        
        with sr.Microphone() as source:
            audio = r.listen(source)
            # audio = r.record(source, duration=5)
            
        # recognize speech using Google Speech Recognition
        try:
            result = r.recognize_google(audio, language="en-US")
            print("SR result: " + result)
            pub.publish(result)
        except sr.UnknownValueError:
            print("SR could not understand audio")
        except sr.RequestError as e:
            print("Could not request results from Google Speech Recognition service; {0}".format(e))

if __name__ == '__main__':
    try:
        googlesr()
    except rospy.ROSInterruptException:
        pass