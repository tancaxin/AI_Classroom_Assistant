#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import speech_recognition as sr
import sounddevice

is_speaking = True

def tts_status_callback(msg):
    global is_speaking
    is_speaking = (msg.data == "speaking") #update is_speaking upon receiving message from tts publisher
    if not is_speaking:
        rospy.loginfo("Robot done speaking...")
    elif is_speaking:
        rospy.loginfo("Robot speaking...")

def googlesr():
    global is_speaking

    rospy.init_node('google_sr', anonymous=True)
    pub = rospy.Publisher('result', String, queue_size=10)

    rospy.Subscriber("tts_status", String, tts_status_callback)


    while not rospy.is_shutdown():
        
        #only do speech recognition when robot not speaking 
        #to avoid picking up its own speech
        if not is_speaking:
            rospy.loginfo("Listening...")

            # obtain audio from the microphone
            r = sr.Recognizer()
            
            with sr.Microphone(device_index=0) as source:
                r.adjust_for_ambient_noise(source)
                audio = r.listen(source)
                
            # recognize speech using Google Speech Recognition
            try:
                result = r.recognize_google(audio, language="en-US")
                print("SR result: " + result)
                pub.publish(result)
            except sr.UnknownValueError:
                print("SR could not understand audio")
            except sr.RequestError as e:
                print("Could not request results from Google Speech Recognition service; {0}".format(e))

if __name__ == "__main__":
    try:
        print(f"Available mics: {sr.Microphone.list_microphone_names()}")
        googlesr()
    except rospy.ROSInterruptException:
        pass