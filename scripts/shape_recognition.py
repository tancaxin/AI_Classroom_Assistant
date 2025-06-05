#!/usr/bin/env/python3
import rospy
from inference_sdk import InferenceHTTPClient

class ShapeRecognition:

    def __init__(self):

        rospy.init_node('shape_recognition_node', anonymous=True)

        # initialize the client
        self.client = InferenceHTTPClient(
            api_url="https://serverless.roboflow.com",
            api_key="e68JMBCA09kOCyfJY0Sj" # naomi's roboflow api
        )


    def recognition(self, image):        
        
        result = self.client.infer(image, model_id="2d-geometry-figures-wdc1b/2")

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
        node = ShapeRecognition()
        node.run()
    except rospy.ROSInterruptException:
        pass