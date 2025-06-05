#!/usr/bin/env/python3

from inference_sdk import InferenceHTTPClient

client = InferenceHTTPClient(
        api_url="https://serverless.roboflow.com",
        api_key="e68JMBCA09kOCyfJY0Sj" # naomi's roboflow api
)


def recognition(self, image):        
        
    result = client.infer(image, model_id="2d-geometry-figures-wdc1b/2")

    return result


def recognize_shape(self, image):

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
        
    return msg

