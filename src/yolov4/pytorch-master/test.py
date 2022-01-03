#!/usr/bin/env python3

import cv2

if __name__ == '__main__':
    img = cv2.imread("./datasets/tea0/0.jpg")
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    cv2.imshow(img)
    cv2.waitKey(0)