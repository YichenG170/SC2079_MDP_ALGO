#!/usr/bin/env python3

import requests
import os

filename = "test_img_right.jpg"
image_path = os.path.join(os.path.abspath(os.path.dirname(__file__)), filename)
url = 'http://127.0.0.1:5000/notify-upload-arrow/test'

response = requests.post(url, files={"file": (filename, open(image_path,'rb'))})

print(response.json())