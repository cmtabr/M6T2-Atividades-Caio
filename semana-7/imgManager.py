import cv2 as cv
from ultralytics import YOLO
from dotenv import load_dotenv
import os
from supabase import create_client, Client
from datetime import datetime

# Load environment variables
load_dotenv()

# Instantiate the camera object
cam = cv.VideoCapture(0)

# Check if the camera is connected or opened
if not cam.isOpened():
    print("Camera not found")
    exit()

# Instantiate the YOLO model
model = YOLO('./model.pt')

# Instantiate the Supabase client
url = os.environ.get("SUPABASE_URL")
key = os.environ.get("SUPABASE_KEY")
supabase: Client = create_client(url, key)

# Define the video writer object
fourcc = cv.VideoWriter_fourcc(*'mp4v')

output_folder = datetime.now().strftime("SCAN_%Y_%d_%m_%H_%M")

os.makedirs(output_folder, exist_ok=True)

video_path = os.path.join(output_folder, 'output.mp4')

out = cv.VideoWriter(video_path, fourcc, 30.0, (640, 480))

image_folder = os.path.join(output_folder, 'images')
os.makedirs(image_folder, exist_ok=True)

frame_counter = 0

# Loop to detect frames with cracks and record the video
while True:
    # Read frames where the crack is identified
    ret, frame = cam.read()

    # Use the YOLO model to detect frames with cracks
    result = model.predict(frame, conf=0.6)

    if ret:
        # Generate a "red box" overlay showing the detection results
        frame_result = result[0].plot()

        image_filename = f"image_{frame_counter}.jpg"

        frame_counter += 1

        # Write the frame result to the video
        out.write(frame_result)

        cv.imshow("Results", frame_result)

        if cv.waitKey(1) == ord('q'):
            break

        # Save the image locally
        image_path = os.path.join(image_folder, image_filename)
        cv.imwrite(image_path, frame_result)

        # Upload the image to Supabase storage
        with open(image_path, 'rb') as f:
            try:
                res = supabase.storage.from_('images').upload(
                    f"{output_folder}/{image_filename}",
                    f.read(),
                    {"contentType": "image/jpeg"}
                )
                if res.status_code != 200:
                    print('Error uploading image:', res.text)
            except Exception as e:
                print('Error uploading image:', str(e))

        # Delete the local image file
        os.remove(image_path)

cam.release()
out.release()

# Upload video to Supabase storage
with open(video_path, 'rb') as f:
    try:
        res = supabase.storage.from_('videos').upload(
            f"{output_folder}/output.mp4",
            f.read(),
            {"contentType": "video/mp4"}
        )
        if res.status_code != 200:
            print('Error uploading video:', res.text)
    except Exception as e:
        print('Error uploading video:', str(e))

# Delete the local video file
os.remove(video_path)

# Delete the image folder
os.rmdir(image_folder)

# Delete the output folder
os.rmdir(output_folder)

cv.destroyAllWindows()
