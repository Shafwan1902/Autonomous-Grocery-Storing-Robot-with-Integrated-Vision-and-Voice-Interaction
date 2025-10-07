#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from gtts import gTTS
import os

def callback(data):
    rospy.loginfo("Input: %s", data.data)
    
    text = data.data.strip()
    try:
        tts = gTTS(text=text, lang='en', tld='com', slow=False)
        tts.save("/tmp/speech.mp3")
        os.system("mpg321 /tmp/speech.mp3")
        os.remove("/tmp/speech.mp3")
    except Exception as e:
        rospy.logwarn(f"Failed to generate speech: {e}")

def googletts():
    rospy.init_node('text2speach', anonymous=True)
    rospy.Subscriber("/text2speach", String, callback)
    rospy.loginfo("Text-to-speech node started. Listening on /text2speach...")
    rospy.spin()

if __name__ == '__main__':
    googletts()
