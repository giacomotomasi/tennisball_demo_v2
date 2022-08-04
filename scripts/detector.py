#!/usr/bin/env python3

import rospy
import detectron2
from detectron2.utils.logger import setup_logger
setup_logger()
# import some common libraries
import numpy as np
import cv2
# import some common detectron2 utilities
from detectron2 import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg
from detectron2.utils.visualizer import Visualizer
from detectron2.data import MetadataCatalog, DatasetCatalog

class Detector(object):
    def __init__(self):
        self.path = "/home/giacomo/Documents/detectron2/demo/tests/"
        self.image_name = "iOS7.jpg"
        self.im = cv2.imread(self.path+self.image_name)

    def detect(self):
        print("Detecting..")
        # create a detectron2 config and a detectron2 DefaultPredictor to run inference on the image preiously loaded
        cfg = get_cfg()
        # add project-specific config (e.g., TensorMask) here if you're not running a model in detectron2's core library
        cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
        cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.9  # set threshold for this model
        # Remove line below if you use GPU
        cfg.MODEL.DEVICE = "cpu"
        cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml")
        predictor = DefaultPredictor(cfg)
        outputs = predictor(self.im)
        print(len(outputs["instances"].pred_classes), "objects have been detected!")

        # extract class labels
        pred_classes = outputs['instances'].pred_classes.cpu().tolist()
        class_names = MetadataCatalog.get(cfg.DATASETS.TRAIN[0]).thing_classes
        pred_class_names = list(map(lambda x: class_names[x], pred_classes))
        # print results by labels
        self.print_labels(pred_class_names)

        print(outputs['instances'].get('pred_boxes')[0]) # print bbox coordinates (x1,y1,x2,y2) of the first object
        print(outputs['instances'].get('pred_boxes')[0].get_centers()) # print bbox center coordinates (xc,yc) of the first object

        ################# VISUALIZATION #############
        # We can use `Visualizer` to draw the predictions on the image.
        v = Visualizer(self.im[:, :, ::-1], MetadataCatalog.get(cfg.DATASETS.TRAIN[0]), scale=0.2)
        out = v.draw_instance_predictions(outputs["instances"].to("cpu"))
        # # concatenate image Vertically
        # Verti = np.concatenate((im, out.get_image()[:, :, ::-1]), axis=0)
        # cv2.imshow('HORIZONTAL', Verti)
        #cv2.imshow("input", im)
        cv2.imshow("result", out.get_image()[:, :, ::-1])
        print("PRESS ANY BUTTON TO ADVANCE..")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        print("Process terminated!")

########## HELPER FUNCTIONS ###################################################################################################################################

    def remove_duplicates(self,l):
        return list(dict.fromkeys(l))

    def count_classes(self, classes):
        result = []
        count = 0
        objects = self.remove_duplicates(classes)
        for object in objects:
            for elem in classes:
                if object == elem:
                    count += 1
            result.append([count, object])
            count = 0
        return result

    def print_labels(self,input):
        print("\n========= OBJECTS: =========")
        for obj in self.count_classes(input):
            print(obj[0], obj[1])
        print("============================\n")

################################################################################################################################################################



def main():
    rospy.init_node('vision_node', anonymous=True)
    print("Vision node started!")
    d = Detector()
    d.detect()
    rospy.spin()

if __name__ == '__main__':
    main()



