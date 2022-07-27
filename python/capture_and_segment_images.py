import argparse
import os
import shutil

import cv2
import torchvision

from python.image_segmentation import instance_segmentation

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset_name", help="Provide a name for the dataset")
    parser.add_argument("--video_id", help="Provide the id of your video stream")
    args = parser.parse_args()

    img_path = f"out/datasets/{args.dataset_name}/raw/"
    mask_path = f"out/datasets/{args.dataset_name}/masks/"
    if os.path.isdir(img_path):
        shutil.rmtree(img_path)
    os.mkdir(img_path)
    if os.path.isdir(mask_path):
        shutil.rmtree(mask_path)
    os.mkdir(mask_path)

    # Read video stream
    video = cv2.VideoCapture(args.video_id)

    if video.isOpened() == False:
        print("Error reading video file")

    print("Loading models..")
    weights = torchvision.models.detection.MaskRCNN_ResNet50_FPN_V2_Weights.COCO_V1
    model = torchvision.models.detection.maskrcnn_resnet50_fpn_v2(weights=weights)
    model.eval()
    model.cuda()

    print("Capturing and segmenting images:")
    idx = 0
    while True:
        ret, frame = video.read()
        if ret == True:
            cv2.imshow("Video", frame)
            if cv2.waitKey(0) & 0xFF == ord("c"):
                segmentation = instance_segmentation(frame, model)
                if segmentation is not None:
                    cv2.imshow("segmentation", segmentation)
                    cv2.imwrite(f"out/datasets/{args.dataset_name}/raw/{idx}.jpg", frame)
                    cv2.imwrite(f"out/datasets/{args.dataset_name}/masks/{idx}.jpg", segmentation)
                    print("Captured image #", idx)
                    idx += 1
        # Break the loop
        else:
            break
