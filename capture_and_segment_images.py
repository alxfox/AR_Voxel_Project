import cv2
import os
import shutil
import torchvision
import glob
from image_segmentation import instance_segmentation

if __name__ == "__main__":
    name = "max"
    # img_path = f"out/datasets/{name}/raw/"
    # mask_path = f"out/datasets/{name}/masks/"
    # if os.path.isdir(img_path):
    #     shutil.rmtree(img_path)
    # os.mkdir(img_path)
    # if os.path.isdir(mask_path):
    #     shutil.rmtree(mask_path) 
    # os.mkdir(mask_path)

    # Read video stream
    video = cv2.VideoCapture(4)

    if (video.isOpened() == False):
        print("Error reading video file")

    print("Loading models..")
    weights = torchvision.models.detection.MaskRCNN_ResNet50_FPN_V2_Weights.COCO_V1
    model = torchvision.models.detection.maskrcnn_resnet50_fpn_v2(weights=weights)
    model.eval()
    model.cuda()

    print("Capturing and segmenting images:")
    idx = 0
#    while(True):
#        ret, frame = video.read()
    for filename in glob.glob('/home/bazinga/AR_Voxel_Project/out/datasets/max_pixel2/*.jpg'):
        ret = True
        frame = cv2.imread(filename)
        frame = cv2.resize(frame, [640, 480])
        if ret == True:
            cv2.imshow("Video", frame)
            if cv2.waitKey(0) & 0xFF == ord('c'):
                segmentation = instance_segmentation(frame, model)
                if segmentation is not None:
                    cv2.imshow("segmentation", segmentation)
                    cv2.imwrite(f"out/datasets/{name}/raw/{idx}.jpg", frame)
                    cv2.imwrite(f"out/datasets/{name}/masks/{idx}.jpg", segmentation)
                    print("Captured image #", idx)
                    idx += 1

        # Break the loop
        else:
            break