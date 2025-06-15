#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from gtts import gTTS
import os

tts_status_pub = None

def callback(data):
    global tts_status_pub

    rospy.loginfo("Input: %s", data.data)

    text = data.data

    # Notify google_sr the tts is speaking
    tts_status_pub.publish("speaking")
    rospy.loginfo("TTS is speaking")


    tts = gTTS(text, lang="en-US")
    
    tts.save("speech.mp3")
    os.system("mpg321 speech.mp3")
    os.remove("speech.mp3")

    # Notify that tts done
    tts_status_pub.publish("idle")
    rospy.loginfo("Done speaking")

    
def googletts():
    global tts_status_pub

    rospy.init_node('google_tts', anonymous=True)

    tts_status_pub = rospy.Publisher("tts_status", String, queue_size=1, latch=True)

    rospy.Subscriber("response_topic", String, callback)

    rospy.spin()

if __name__ == "__main__":
    googletts()