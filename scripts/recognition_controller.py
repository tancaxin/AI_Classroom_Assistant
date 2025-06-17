#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from shape_recognition import recognize_shape
from colour_recognition import recognize_colour

from openai import OpenAI
import os

client = OpenAI(
    api_key="API-KEY"
)

SYSTEM_PROMPT = """You are a friendly and knowledgeable classroom assistant for young children (ages 4–8). You will receive the question regarding object(showed to the camera of the device) from the child, the detected colour, and the detected shape of the object. You need respond with a short, cheerful explanation.

If you are clear that the child is asking for either shape or colour, your answer should consist of the belows, answer each point as different sentences:
- Clearly tell the color or shape, ONLY one of them, according to what the child asks. 
- Include 1–2 fun facts or characteristics about it.
- Give an example of where this color or shape appears in real life.

Else if you cannot catch what the child is asking, you can tell both colour and shape, but don't make it like you are giving two responses, it should be a continuous response, including both colour and shape.

Be phrased in simple, kid-friendly language.

If you did not receive any information about the detected shape or colour, respond gently and ask them to show the object to the camera again.

You do not need to describe anything else except the color or shape mentioned.
"""

class RecognitionController:

    def __init__(self):
        rospy.init_node("recognition_controller_node")
        self.pub = rospy.Publisher('response_topic', String, queue_size=10, latch=True)

        self.bridge = CvBridge()

        self.latest_image = None

        self.speech_subscriber = None
        rospy.Subscriber("/usb_cam/image_raw", Image, self.process_image)
        rospy.Subscriber("tts_status", String, self.subscribe_controller)
        #greet user once
        greet = "Hello! I'm your AI classroom assistant. Let's have fun learning shapes and colours together!"
        self.pub.publish(greet)
        rospy.sleep(1)


    def subscribe_controller(self, msg):
        is_speaking = (msg.data == "speaking")
        if self.speech_subscriber is not None:
            if is_speaking:
                self.speech_subscriber.unregister()
                self.speech_subscriber = None
                rospy.loginfo("Unsubscribed from /result topic")
        elif self.speech_subscriber is None:
            if not is_speaking:              
                rospy.sleep(5)  # Wait a bit before resubscribing to avoid picking up its own speech
                self.speech_subscriber = rospy.Subscriber("result", String, self.process_speech)
                rospy.loginfo("Subscribed to /result topic")

    #Receive from image_raw node, process to .jpg to be compatible for the recognition models
    #Need check if want to unsubscribe to "camera/image_raw" or "result" topic first or not(when going through the following process)
    def process_image(self, img_msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')


    def process_speech(self, msg):

        user_text = msg.data
        rospy.loginfo("SR result: " + user_text)


        if self.latest_image is None:
            rospy.logwarn("No image received yet.")
            return
        
         # Save .jpg as temporary file 
        temp_path = "/tmp/frame.jpg"
        cv2.imwrite(temp_path, self.latest_image)

        # Run shape and color recognition
        shape = recognize_shape(temp_path)
        
        colour = recognize_colour(temp_path)

        rospy.loginfo(f"Detected shape: {shape}, color: {colour}")

        # Use OpenAI to generate response
        response = self.query_openai(user_text, shape, colour)
        rospy.loginfo(f"Generated response: {response}")

        self.pub.publish(response)


      
    def query_openai(self, speech, shape = None, color = None):

        prompt = f'Child said: "{speech}"\n'
        if shape:
            prompt += f"Recognized shape: {shape}\n"
        if color:
            prompt += f"Recognized color: {color}\n"

        response = client.responses.create(
        model="gpt-4.1",
        instructions=SYSTEM_PROMPT,
        input=prompt,
        temperature = 1.00,
        top_p = 1.00,
        )

        return response.output_text

   

if __name__ == "__main__":
    try:
        RecognitionController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
