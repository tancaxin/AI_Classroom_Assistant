#!/usr/bin/env/python3
import rospy
from std_msgs.msg import String
from inference_sdk import InferenceHTTPClient
from sensor_msgs.msg import Image
import cv2


class ColourRecognition:

    def __init__(self):

        rospy.init_node('colour_recognition_node', anonymous=True)


        # initialize the client
        self.client = InferenceHTTPClient(
            api_url="https://serverless.roboflow.com",
            api_key="e68JMBCA09kOCyfJY0Sj" # naomi's roboflow api, can change to urs
        )


    def recognition(self, image):        
        
        result = self.client.infer(image, model_id="warna-6iofs-szvec/3")
        return result


    def run(self, image):

        # Run inference
        result = self.recognition(image)

        # Extract colour names (NEED CHANGE, THIS IS FOR SHAPE
        predictions = result.get("predictions", [])

        if predictions:
            # Get the prediction with the highest confidence
            top_prediction = max(predictions, key=lambda x: x["confidence"])
            msg = top_prediction["class"]
        else:
            msg = "No shape detected"

        # Publish result
        rospy.loginfo(f"Detected: {msg}")
        
        return msg


if __name__ == "__main__":
    try:
        node = ColourRecognition()
        node.run()
    except rospy.ROSInterruptException:
        pass