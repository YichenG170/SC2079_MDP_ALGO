#!/usr/bin/env python3

import requests
import os
from test_obs_inputs import test_inputs
import json

url = 'http://127.0.0.1:6000/path'

for test_course in test_inputs:
    test_course = json.loads(test_course)
    response = requests.post(url, json=test_course)

    if response.status_code != 200:
        print("Error")

    print(response.json(), response.status_code)
