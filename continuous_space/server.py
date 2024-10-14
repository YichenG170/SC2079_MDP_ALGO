#!/usr/bin/env python3

from flask import Flask, jsonify, request
import requests
import os
import torch
import numpy as np
import cv2
import endpoint
from PIL import Image
from yolo_to_checklist_id import yolo_to_check

import pathlib
temp = pathlib.PosixPath
pathlib.PosixPath = pathlib.WindowsPath

# config
rpi_url = "http://192.168.27.1:4000"
app = Flask(__name__)
model_path = os.path.join(os.path.abspath(os.path.dirname(__file__)), "best.pt")

# Load your trained model
model = torch.hub.load('ultralytics/yolov5', 'custom', model_path)
model.eval()


@app.route('/status', methods=['GET'])
def status():
    return jsonify({"status": "Live"}), 200


# To get the algo path
@app.route('/path', methods=['POST'])
def path():
    json_input = request.json
    print("Input received: ", json_input)

    return endpoint.map_to_inst(json_input)
    # >>>> End of function


def do_inference(image_path, obstacle_num):

    # Perform inference
    results = model(image_path)

    # if nothing detected
    if len(results.xyxy[0]) == 0:
        return jsonify({
            "id": "NA",
            "name": "NA",
            "obstacle_num": "NA",
        }
        ), 200
    
    # find highest confidence
    highest_conf = 0
    highest_idx = 0
    for i, res in enumerate(results.xyxy[0]):
        try:
            # x1, y1, x2, y2 = map(int, res[:4])  # Coordinates of the bounding box
            conf = res[4].numpy()               # confidence
            cls = int(res[5])                   # Class label (integer index)
            label = f'{model.names[int(cls)]} {conf:.2f}'
            print(f"{i}: conf={conf}, cls={cls}, label={label}")
            if conf > highest_conf:
                highest_conf = conf
                highest_idx = cls
        except:
            pass
        
    ret = jsonify({
            "id": yolo_to_check[int(highest_idx)],
            "name": model.names[int(highest_idx)],
            "obstacle_id": obstacle_num,
        }
        ), 200
    
    print(ret.content)
    return ret

    # >>>>> Function End


@app.route('/notify-upload/<num>', methods=['POST'])
def upload_image(num):
    '''
    ## >> How to use

    image_path = 'path_to_image.jpg'
    with open(image_path, 'rb') as img_file:
        # Prepare the file as part of the form data
        files = {'file': img_file}
    
    # Send the POST request
    response = requests.post(url, files=files)
    '''

    if 'file' not in request.files:
        return jsonify({"error": "No image part in the request"}), 400

    img_file = request.files['file']

    if img_file.filename == '':
        return jsonify({"error": "No selected file"}), 400

    # Save the image
    image_path = os.path.join(os.path.abspath(os.path.dirname(__file__)), "images", img_file.filename)
    img_file.save(image_path)

    return do_inference(image_path, num)


# rerun inference + mark images in the PC folder
@app.route('/mark_rerun', methods=['GET'])
def mark_rerun():
    folder_path = os.path.join(os.path.abspath(os.path.dirname(__file__)), "images")

    # Loop through all files in the folder
    for filename in os.listdir(folder_path):
        if filename.endswith('.jpg'):

            image_path = os.path.join(folder_path, filename)
            save_path = os.path.join(os.path.abspath(os.path.dirname(__file__)), "marked", filename)

            # Perform inference
            results = model(image_path)

            # if nothing detected
            if len(results.xyxy[0]) == 0:
                continue

            # find highest confidence
            highest_conf = 0
            highest_res = 0

            # get the highest confidence read
            for res in results.xyxy[0]:
                try:
                    conf = res[4].numpy()   # confidence
                    if conf > highest_conf:
                        highest_conf = conf
                        highest_res = res
                except:
                    pass

            # open image
            img = cv2.imread(image_path)

            # get obstacle id
            obstacle_id = filename.split("_")[1]

            # mark image
            try:
                mark_img(img, highest_res, obstacle_id)
            except:
                pass
            
            # save image
            cv2.imwrite(save_path, img)

    return jsonify({"Done": "Images marked"}), 200
    # >>>> End of Function


@app.route('/stitch', methods=['GET'])
def stitch():
    direction = "V" # manually edit this to change

    mark_response = requests.get("http://127.0.0.1:5000/mark_rerun")

    if mark_response.status_code != 200:
        return mark_response

    folder_path = os.path.join(os.path.abspath(os.path.dirname(__file__)), "marked")
    save_path = os.path.join(os.path.abspath(os.path.dirname(__file__)), "stitched", "stitched.jpg")

    # Get all .jpg images in the folder
    images = [Image.open(os.path.join(folder_path, file)) for file in os.listdir(folder_path) if file.endswith('.jpg')]

    if not images:
        print("No .jpg images found in the folder.")
        return

    # Get total width and height for the final stitched image
    widths, heights = zip(*(img.size for img in images))

    if direction == 'H':
        # Stitch horizontally: sum of widths, max height
        total_width = sum(widths)
        max_height = max(heights)
        stitched_image = Image.new('RGB', (total_width, max_height))
        
        # Paste images one by one into the new stitched image
        x_offset = 0
        for img in images:
            stitched_image.paste(img, (x_offset, 0))
            x_offset += img.width
    else:
        # Stitch vertically: sum of heights, max width
        total_height = sum(heights)
        max_width = max(widths)
        stitched_image = Image.new('RGB', (max_width, total_height))
        
        # Paste images one by one into the new stitched image
        y_offset = 0
        for img in images:
            stitched_image.paste(img, (0, y_offset))
            y_offset += img.height

    # Save the stitched image
    stitched_image.save(save_path)
    return jsonify({"Done": "Images stitched"}), 200


def mark_img(img, res, obstacle_num):
    height, width, _ = img.shape  # Get image dimensions

    # Adjust font scale and thickness based on image size
    font_scale = min(width, height) / 1000  # Adjust this divisor to fine-tune size
    thickness = max(1, int(min(width, height) / 500))  # Adjust this divisor as needed

    x1, y1, x2, y2 = map(int, res[:4])  # Coordinates of the bounding box
    conf = res[4].numpy()               # confidence
    cls = int(res[5])                   # Class label (integer index)
    label = f'name:{model.names[int(cls)]} | id:{yolo_to_check[int(cls)]} | obs_num:{obstacle_num}'
    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), thickness)

    # Calculate the center of the bounding box
    center_x = (x1 + x2) // 2
    center_y = (y1 + y2) // 2
    
    # Get the size of the text
    (text_width, text_height), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, 2)

    # Calculate text starting position for center alignment
    text_x = center_x - (text_width // 2)
    text_y = center_y + (text_height // 2)

    # Define the background rectangle coordinates
    background_topleft = (text_x - 5, text_y - text_height - 5)  # Add padding around the text
    background_bottomright = (text_x + text_width + 5, text_y + baseline + 5)

    # Draw the background rectangle (filled)
    cv2.rectangle(img, background_topleft, background_bottomright, (0, 0, 0), -1)  # Black background

    # Place the label in the middle of the bounding box
    cv2.putText(img, label, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 255, 0), 2)


if __name__ == "__main__":
    app.run(debug=True, host='0.0.0.0', port=5000) 


# >>>>>>>>>>>> Unused code


# # for getting the image
# @app.route('/notify/<num>', methods=['GET'])
# def do_img_reg(num):

#     # download image
#     imgName = "snap" + num + ".jpg"
#     image_path = os.path.join(os.path.abspath(os.path.dirname(__file__)), imgName)
#     response = requests.get(rpi_url + "/getimage/" + num)

#     # save the image
#     if response.status_code == 200:
#         print(200)
#         with open(image_path, "wb") as file:
#             file.write(response.content)
#     else:
#         return jsonify({"error": "Failed to retrieve image"}), response.status_code
    
#     return do_inference(image_path, num)

#     # >>>>> Function end


