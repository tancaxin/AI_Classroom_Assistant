#!/usr/bin/env/python3

from inference_sdk import InferenceHTTPClient
import requests


def recognize_colour(image):

    # call Roboflow api "https://serverless.roboflow.com/[model_id]?api_key=[your api key]"
    url = "https://serverless.roboflow.com/colours-rtjqy-wjzzm/1?api_key=e68JMBCA09kOCyfJY0Sj"

    with open(image, "rb") as image_file:
        result = requests.post(
            url,
            files = {"file": image_file}
        )

    if result.status_code == 200:
        data = result.json()
        predictions = data.get("predictions", [])

    if predictions:
        # Get the prediction with the highest confidence
        top_prediction = max(predictions, key=lambda x: x["confidence"])
        msg = top_prediction["class"]
    else:
        msg = "No colour detected"

    return msg


