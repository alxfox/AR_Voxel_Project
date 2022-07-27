import cv2
import numpy as np
from torchvision import transforms as T

COCO_INSTANCE_CATEGORY_NAMES = [
    "__background__",
    "person",
    "bicycle",
    "car",
    "motorcycle",
    "airplane",
    "bus",
    "train",
    "truck",
    "boat",
    "traffic light",
    "fire hydrant",
    "N/A",
    "stop sign",
    "parking meter",
    "bench",
    "bird",
    "cat",
    "dog",
    "horse",
    "sheep",
    "cow",
    "elephant",
    "bear",
    "zebra",
    "giraffe",
    "N/A",
    "backpack",
    "umbrella",
    "N/A",
    "N/A",
    "handbag",
    "tie",
    "suitcase",
    "frisbee",
    "skis",
    "snowboard",
    "sports ball",
    "kite",
    "baseball bat",
    "baseball glove",
    "skateboard",
    "surfboard",
    "tennis racket",
    "bottle",
    "N/A",
    "wine glass",
    "cup",
    "fork",
    "knife",
    "spoon",
    "bowl",
    "banana",
    "apple",
    "sandwich",
    "orange",
    "broccoli",
    "carrot",
    "hot dog",
    "pizza",
    "donut",
    "cake",
    "chair",
    "couch",
    "potted plant",
    "bed",
    "N/A",
    "dining table",
    "N/A",
    "N/A",
    "toilet",
    "N/A",
    "tv",
    "laptop",
    "mouse",
    "remote",
    "keyboard",
    "cell phone",
    "microwave",
    "oven",
    "toaster",
    "sink",
    "refrigerator",
    "N/A",
    "book",
    "clock",
    "vase",
    "scissors",
    "teddy bear",
    "hair drier",
    "toothbrush",
]


def get_prediction(img, model, threshold=0.5):
    transform = T.Compose([T.ToTensor()])
    img = transform(img)
    img = img.cuda()
    pred = model([img])
    pred_score = list(pred[0]["scores"].detach().cpu().numpy())
    pred_t = [pred_score.index(x) for x in pred_score if x > threshold]
    if len(pred_t):
        pred_t = [pred_score.index(x) for x in pred_score if x > threshold][-1]
    else:
        return None
    masks = (pred[0]["masks"] > 0.5).squeeze().detach().cpu().numpy()
    pred_class = [COCO_INSTANCE_CATEGORY_NAMES[i] for i in list(pred[0]["labels"].cpu().numpy())]
    masks = masks[: pred_t + 1]
    pred_class = pred_class[: pred_t + 1]
    masks = masks[[p == "person" for p in pred_class]]
    return masks


def random_color_masks(image):
    r = np.zeros_like(image).astype(np.uint8)
    g = np.zeros_like(image).astype(np.uint8)
    b = np.zeros_like(image).astype(np.uint8)
    r[image == 1], g[image == 1], b[image == 1] = [255, 255, 255]
    colored_mask = np.stack([r, g, b], axis=2)
    return colored_mask


def instance_segmentation(img, model, threshold=0.5):
    masks = get_prediction(img, model, threshold=threshold)
    if masks is None:
        return None
    rgb_mask = None
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # For working with RGB images instead of BGR
    for i in range(len(masks)):
        rgb_mask = random_color_masks(masks[i])
        img = cv2.addWeighted(img, 1, rgb_mask, 0.5, 0)
    return rgb_mask
