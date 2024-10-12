#!/usr/bin/env python3

import requests
import os

test_course = {
    "obstacles":
    [
        {
            "x": 9,
            "y": 9,
            "id": 1,
            "d": 2
        }
    ]
}

url = 'http://127.0.0.1:5000/path'

response = requests.post(url, json=test_course)

print(response.json(), response.status_code)