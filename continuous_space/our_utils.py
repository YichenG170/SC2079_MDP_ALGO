from flask import Flask, jsonify, request
from constants import BOXSIZE_EP, IMAGE_BORDER_SIZE, MIN_BOX_SIZE
from yolo_to_checklist_id import yolo_to_check, yolo_to_check_v8_task1, yolo_to_check_v8_task2
import cv2

def do_inference_v8(model, image_path, obstacle_num, arrows_only = False):

    print("Starting image reg v8")

    # inference
    results = model(image_path)

    # init vars
    highest_conf = 0
    highest_boxsize = 0
    highest_idx = 0
    highest_classname = ""
    detected = False
    detected_cls = set()

    for result in results:
    # Loop over each detection result (for multi-image results)
        for i, box in enumerate(result.boxes):
            # Extract information from the box
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()  # Get box coordinates [x1, y1, x2, y2]
            conf = float(box.conf.cpu().numpy())        # Get confidence score
            cls = int(box.cls.cpu().numpy())            # Get label index (class id)
            detected_cls.add(cls)

            boxsize = abs((x2 - x1) * (y2 - y1)) # size of bounding box

            # Optionally: map label index to class names (if using a pre-trained model)
            class_name = model.names[cls]     # Get class name from label

            # debug print
            print(f"{i}: conf={conf}, boxsize={boxsize}, label={cls}, name={class_name}")

            # ensure arrow if only arrow
            # EDIT IF CHANGE FILE
            if arrows_only and (class_name not in ["Left_arrow", "Right_arrow"]):
                print("Not Arrow")
                continue
            
            # ensure large enough
            if boxsize < MIN_BOX_SIZE:
                print("Skipped ^, due to too small")
                continue

            # about same size
            if abs(boxsize - highest_boxsize) <= BOXSIZE_EP:
                # but check confidence
                if conf > highest_conf:
                    highest_conf = conf
                    highest_idx = cls
                    highest_boxsize = (boxsize + highest_boxsize) / 2
                    highest_classname = class_name
                    detected = True
                    
            # bigger: take
            elif boxsize > highest_boxsize:
                highest_conf = conf
                highest_idx = cls
                highest_boxsize = boxsize
                highest_classname = class_name
                detected = True

    if detected:
        id = int(yolo_to_check_v8_task2[int(highest_idx)])

        ret = {
                "id": id,
                "name": highest_classname,
                "obstacle_id": obstacle_num,
                "detected": len(detected_cls)
            }
        
        print(ret)
        return jsonify(ret), 200
            
    # cannot detect for needed purpose
    print("Nothing detected")
    return jsonify({
            "id": 0,
            "name": 0,
            "obstacle_num": obstacle_num,
            "detected": 0
        }
        ), 200



def do_inference_v5(model, image_path, obstacle_num, arrows_only = False):

    print("Starting image reg v5")

    # Perform inference
    results = model(image_path)

    # if nothing detected
    if len(results.xyxy[0]) == 0:
        return jsonify({
            "id": 0,
            "name": 0,
            "obstacle_num": obstacle_num,
            "detected": 0
        }
        ), 200
    
    
    # find highest confidence
    highest_conf = 0
    highest_boxsize = 0
    highest_idx = 0
    detected = False

    if arrows_only:
        print("Finding arrows (+bullseye) only")
        for i, res in enumerate(results.xyxy[0]):
            try:
                x1, y1, x2, y2 = map(int, res[:4])  # Coordinates of the bounding box
                boxsize = abs((x2 - x1) * (y2 - y1)) # size of bounding box
                conf = res[4].numpy()               # confidence
                cls = int(res[5])                   # Class label (integer index) : before convert
                
                if cls not in [13, 16]:
                    continue

                if boxsize < MIN_BOX_SIZE:
                    continue

                label = f'{model.names[int(cls)]} {conf:.2f}'
                print(f"{i}: conf={conf}, boxsize={boxsize}, label={label}")

                # about same size
                if abs(boxsize - highest_boxsize) <= BOXSIZE_EP:
                    # but check confidence
                    if conf > highest_conf:
                        highest_conf = conf
                        highest_idx = cls
                        highest_boxsize = (boxsize, highest_boxsize) / 2
                        detected = True
                # bigger: take
                elif boxsize > highest_boxsize:
                    highest_conf = conf
                    highest_idx = cls
                    highest_boxsize = boxsize
                    detected = True
            except:
                pass
    else:
        print("Finding all symbols")
        for i, res in enumerate(results.xyxy[0]):
            try:
                x1, y1, x2, y2 = map(int, res[:4])   # Coordinates of the bounding box
                boxsize = abs((x2 - x1) * (y2 - y1)) # size of bounding box
                conf = res[4].numpy()                # confidence
                cls = int(res[5])                    # Class label (integer index)
                label = f'{model.names[int(cls)]} {conf:.2f}'
                print(f"{i}: conf={conf}, boxsize={boxsize}, label={label}")

                if boxsize < MIN_BOX_SIZE:
                    continue

                # about same size
                if abs(boxsize - highest_boxsize) <= BOXSIZE_EP:
                    # but check confidence
                    if conf > highest_conf:
                        highest_conf = conf
                        highest_idx = cls
                        highest_boxsize = max(boxsize, highest_boxsize)
                        detected = True
                # bigger: take
                elif boxsize > highest_boxsize:
                    highest_conf = conf
                    highest_idx = cls
                    highest_boxsize = boxsize
                    detected = True
            except:
                pass

    if detected:
        return jsonify({
                "id": int(yolo_to_check[int(highest_idx)]),
                "name": model.names[int(highest_idx)],
                "obstacle_id": obstacle_num,
                "detected": len(results.xyxy[0])
            }
            ), 200
            
    # cannot detect for needed purpose
    return jsonify({
            "id": 0,
            "name": 0,
            "obstacle_num": obstacle_num,
            "detected": 0
        }
        ), 200

    # >>>>> Function End


def mark_img(img, x1, y1, x2, y2, name, id, obstacle_num, border_size):
    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
    
    height, width, _ = img.shape  # Get image dimensions
    height += border_size
    width += border_size

    # Adjust font scale and thickness based on image size
    font_scale = min(width, height) / 1000  # Adjust this divisor to fine-tune size
    thickness = max(1, int(min(width, height) / 500))  # Adjust this divisor as needed

    x1 += border_size
    x2 += border_size
    y1 += border_size
    y2 += border_size

    label = f'name:{name} | id:{id} | obs_num:{obstacle_num}'
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