import numpy as np
import cv2
from rknnlite.api import RKNNLite
import sys


class rknn:
    def __init__(self, model_path='wk.rknn', obj=0.25, nms=0.45, obj_class=("red", "green", "blue")):
        self.RKNN_MODEL = model_path
        self.QUANTIZE_ON = True
        self.OBJ_THRESH = obj

        self.NMS_THRESH = nms
        self.IMG_SIZE = 640
        self.CLASSES = obj_class
        self.rknn_init()


    def sigmoid(self, x):
        return 1 / (1 + np.exp(-x))

    def xywh2xyxy(self, x):
        # Convert [x, y, w, h] to [x1, y1, x2, y2]
        y = np.copy(x)
        y[:, 0] = x[:, 0] - x[:, 2] / 2  # top left x
        y[:, 1] = x[:, 1] - x[:, 3] / 2  # top left y
        y[:, 2] = x[:, 0] + x[:, 2] / 2  # bottom right x
        y[:, 3] = x[:, 1] + x[:, 3] / 2  # bottom right y
        return y

    def process(self, input, mask, anchors):

        anchors = [anchors[i] for i in mask]
        grid_h, grid_w = map(int, input.shape[0:2])

        box_confidence = self.sigmoid(input[..., 4])
        box_confidence = np.expand_dims(box_confidence, axis=-1)

        box_class_probs = self.sigmoid(input[..., 5:])

        box_xy = self.sigmoid(input[..., :2]) * 2 - 0.5

        col = np.tile(np.arange(0, grid_w), grid_w).reshape(-1, grid_w)
        row = np.tile(np.arange(0, grid_h).reshape(-1, 1), grid_h)
        col = col.reshape(grid_h, grid_w, 1, 1).repeat(3, axis=-2)
        row = row.reshape(grid_h, grid_w, 1, 1).repeat(3, axis=-2)
        grid = np.concatenate((col, row), axis=-1)
        box_xy += grid
        box_xy *= int(self.IMG_SIZE / grid_h)

        box_wh = pow(self.sigmoid(input[..., 2:4]) * 2, 2)
        box_wh = box_wh * anchors

        box = np.concatenate((box_xy, box_wh), axis=-1)

        return box, box_confidence, box_class_probs

    def filter_boxes(self, boxes, box_confidences, box_class_probs):
        """Filter boxes with box threshold. It's a bit different with origin yolov5 post process!

        # Arguments
            boxes: ndarray, boxes of objects.
            box_confidences: ndarray, confidences of objects.
            box_class_probs: ndarray, class_probs of objects.

        # Returns
            boxes: ndarray, filtered boxes.
            classes: ndarray, classes for boxes.
            scores: ndarray, scores for boxes.
        """
        boxes = boxes.reshape(-1, 4)
        box_confidences = box_confidences.reshape(-1)
        box_class_probs = box_class_probs.reshape(-1, box_class_probs.shape[-1])

        _box_pos = np.where(box_confidences >= self.OBJ_THRESH)
        boxes = boxes[_box_pos]
        box_confidences = box_confidences[_box_pos]
        box_class_probs = box_class_probs[_box_pos]

        class_max_score = np.max(box_class_probs, axis=-1)
        classes = np.argmax(box_class_probs, axis=-1)
        _class_pos = np.where(class_max_score >= self.OBJ_THRESH)

        boxes = boxes[_class_pos]
        classes = classes[_class_pos]
        scores = (class_max_score * box_confidences)[_class_pos]

        return boxes, classes, scores

    def nms_boxes(self, boxes, scores):
        """Suppress non-maximal boxes.

        # Arguments
            boxes: ndarray, boxes of objects.
            scores: ndarray, scores of objects.

        # Returns
            keep: ndarray, index of effective boxes.
        """
        x = boxes[:, 0]
        y = boxes[:, 1]
        w = boxes[:, 2] - boxes[:, 0]
        h = boxes[:, 3] - boxes[:, 1]

        areas = w * h
        order = scores.argsort()[::-1]

        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)

            xx1 = np.maximum(x[i], x[order[1:]])
            yy1 = np.maximum(y[i], y[order[1:]])
            xx2 = np.minimum(x[i] + w[i], x[order[1:]] + w[order[1:]])
            yy2 = np.minimum(y[i] + h[i], y[order[1:]] + h[order[1:]])

            w1 = np.maximum(0.0, xx2 - xx1 + 0.00001)
            h1 = np.maximum(0.0, yy2 - yy1 + 0.00001)
            inter = w1 * h1

            ovr = inter / (areas[i] + areas[order[1:]] - inter)
            inds = np.where(ovr <= self.NMS_THRESH)[0]
            order = order[inds + 1]
        keep = np.array(keep)
        return keep

    def yolov5_post_process(self, input_data):
        masks = [[0, 1, 2], [3, 4, 5], [6, 7, 8]]
        anchors = [[10, 13], [16, 30], [33, 23], [30, 61], [62, 45],
                   [59, 119], [116, 90], [156, 198], [373, 326]]

        boxes, classes, scores = [], [], []
        for input, mask in zip(input_data, masks):
            b, c, s = self.process(input, mask, anchors)
            b, c, s = self.filter_boxes(b, c, s)
            boxes.append(b)
            classes.append(c)
            scores.append(s)

        boxes = np.concatenate(boxes)
        boxes = self.xywh2xyxy(boxes)
        classes = np.concatenate(classes)
        scores = np.concatenate(scores)

        nboxes, nclasses, nscores = [], [], []
        for c in set(classes):
            inds = np.where(classes == c)
            b = boxes[inds]
            c = classes[inds]
            s = scores[inds]

            keep = self.nms_boxes(b, s)

            nboxes.append(b[keep])
            nclasses.append(c[keep])
            nscores.append(s[keep])

        if not nclasses and not nscores:
            return None, None, None

        boxes = np.concatenate(nboxes)
        classes = np.concatenate(nclasses)
        scores = np.concatenate(nscores)

        return boxes, classes, scores

    def draw(self, image, boxes, scores, classes):
        """Draw the boxes on the image.

        # Argument:
            image: original image.
            boxes: ndarray, boxes of objects.
            classes: ndarray, classes of objects.
            scores: ndarray, scores of objects.
            all_classes: all classes name.
        """



        max_in=np.argmax(scores)

        # for box, score, cl in zip(boxes, scores, classes):
        box=boxes[max_in]
        cl=classes[max_in]
        score=scores[max_in]
        top, left, right, bottom = box
        print('class: {}, score: {}'.format(self.CLASSES[cl], score))
        print('box coordinate left,top,right,down: [{}, {}, {}, {}]'.format(top, left, right, bottom))
        top = int(top)
        left = int(left)
        right = int(right)
        bottom = int(bottom)

        cv2.rectangle(image, (top, left), (right, bottom), (255, 0, 0), 2)
        cv2.putText(image, '{0} {1:.2f}'.format(self.CLASSES[cl], score),
                    (top, left - 6),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 0, 255), 2)
        return box,score,cl

    def drawtarget(self, image, boxes, scores, classes,target):

        indices_class_1 = np.where(classes == target)[0]
        print(f"class :{indices_class_1}")
        # 如果找到了至少一个类别为1的box
        if indices_class_1.size > 0:
            # 在这些索引对应的scores中找到最大值
            max_score_index = np.argmax(scores[indices_class_1])
            print(max_score_index)
            # 使用这个索引找到对应的原始索引（在原始数组中的位置）
            max_in = indices_class_1[max_score_index]

            # 提取box, score, class
            box = boxes[max_in]
            cl = classes[max_in]  # 注意这里cl的值已经是1了，因为我们是根据class为1筛选的
            score = scores[max_in]
            print(f"box:{box},cl{cl},score{score}")
            # 分解box
            top, left, right, bottom = box

            top = int(top)
            left = int(left)
            right = int(right)
            bottom = int(bottom)

            cv2.rectangle(image, (top, left), (right, bottom), (255, 0, 0), 2)
            cv2.putText(image, '{0} {1:.2f}'.format(self.CLASSES[cl], score),
                        (top, left - 6),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 0, 255), 2)
            return box, score, cl
        else:
            return None,None,None

    # result = inference(image)
    def letterbox(self, im, new_shape=(640, 640), color=(0, 0, 0)):
        # Resize and pad image while meeting stride-multiple constraints
        shape = im.shape[:2]  # current shape [height, width]
        if isinstance(new_shape, int):
            new_shape = (new_shape, new_shape)

        # Scale ratio (new / old)
        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])

        # Compute padding
        ratio = r, r  # width, height ratios
        new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
        dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding

        dw /= 2  # divide padding into 2 sides
        dh /= 2

        if shape[::-1] != new_unpad:  # resize
            im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
        return im, ratio, (dw, dh)

    def rknn_init(self):

        # Create RKNN object
        self.rknn = RKNNLite(verbose=False)

        # load RKNN model
        print('--> Load RKNN model')
        self.ret = self.rknn.load_rknn(self.RKNN_MODEL)

        # Init runtime environment
        print('--> Init runtime environment')
        self.ret = self.rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0_1_2)  # 使用0 1 2三个NPU核心
        if self.ret != 0:
            print('Init runtime environment failed!')
            exit(self.ret)
        print('done')

    def rknn_detect(self, frame,target=-1):
        # Set inputs
        img = frame
        # img, ratio, (dw, dh) = self.letterbox(img, new_shape=(self.IMG_SIZE, self.IMG_SIZE))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (self.IMG_SIZE, self.IMG_SIZE))

        # Inference
        outputs = self.rknn.inference(inputs=[img])
        # outputs = self.rknn.inference(inputs=[img])
        # outputs = self.inference(inputs=[img])
        # post process
        input0_data = outputs[0]
        input1_data = outputs[1]
        input2_data = outputs[2]

        input0_data = input0_data.reshape([3, -1] + list(input0_data.shape[-2:]))
        input1_data = input1_data.reshape([3, -1] + list(input1_data.shape[-2:]))
        input2_data = input2_data.reshape([3, -1] + list(input2_data.shape[-2:]))

        input_data = list()
        input_data.append(np.transpose(input0_data, (2, 3, 0, 1)))
        input_data.append(np.transpose(input1_data, (2, 3, 0, 1)))
        input_data.append(np.transpose(input2_data, (2, 3, 0, 1)))

        boxes, classes, scores = self.yolov5_post_process(input_data)

        img_1 = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        if boxes is not None :
            if target<0:
                boxs,score ,classe=self.draw(img_1, boxes, scores, classes)
                cv2.imshow(f"{self.RKNN_MODEL}", img_1)
                cv2.waitKey(1)
                return boxs,score ,classe
            else:
                boxs,score ,classe=self.drawtarget(img_1, boxes, scores, classes,target)
                if boxs is not None:
                    cv2.imshow(f"{self.RKNN_MODEL}", img_1)
                    cv2.waitKey(1)
                    return boxs,score ,classe
                else:
                    return None, None,None


        else:
            #cv2.imshow(f"{self.RKNN_MODEL}", img_1)
            #cv2.waitKey(1)

            return None, None,None
        # show output

        # if cv2.waitKey(1) & 0xFF == ord("q"):
        #     cv2.destroyAllWindows()
        #     self.rknn.release()

    def rknn_boxes_lvbo(self, boxes, classes):
        pass


if __name__ == '__main__':

    rknn1 = rknn("wk.rknn", 0.6, 0.6)

    cap = cv2.VideoCapture(2)

    while True:
        _,img=cap.read()
        # cv2.imshow("img",img)
        # # img=img.reshape(())
        #
        # print(img.shape)
        boxs,scores,classes=rknn1.rknn_detect(img)

        # if boxs is not None:
        #     max_in=np.argmax(scores)
        #
        #     #for box, score, cl in zip(boxes, scores, classes):
        if boxs is not None:
            top, left, right, bottom = boxs
            top = int(top)
            left = int(left)
            right = int(right)
            bottom = int(bottom)
            print('class in main: {}, score: {}'.format(classes, scores))
            print('box coordinate left,top,right,down in main: [{}, {}, {}, {}]'.format(top, left, right, bottom))
        #
        #     cv2.rectangle(img, (top, left), (right, bottom), (255, 0, 0), 2)
        #     cv2.putText(img, '{0} {1:.2f}'.format(classes[max_in], scores[max_in]),
        #                 (top, left - 6),
        #                 cv2.FONT_HERSHEY_SIMPLEX,
        #                 0.6, (0, 0, 255), 2)
        # cv2.imshow("img",img)
        #if boxs is not None:
         #   print("boxs:"+boxs)
          #  print("classed:")
        # print(img.shape)
        cv2.waitKey(1)