import numpy as np
import cv2
import yaml
from ament_index_python.packages import get_package_share_directory
import os


class Camera:
    def __init__(self, cfg, calibrated=True, open_camera=True):
        sensor_id = cfg['sensor_id']
        camera_setting = cfg['camera_setting']
        if open_camera:
            camera_channel = camera_setting['camera_channel']
            self.cap = cv2.VideoCapture(camera_channel)
            if not self.cap.isOpened():
                raise RuntimeError(f"Failed to open camera at index {camera_channel}")
            print('------Camera is open--------')
            raw_img_width = camera_setting['resolution'][0]
            raw_img_height = camera_setting['resolution'][1]
            fps = camera_setting['fps']
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, raw_img_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, raw_img_height)
            self.cap.set(cv2.CAP_PROP_FPS, fps)
        else:
            self.cap = None

        package_share = get_package_share_directory('9dtact')
        camera_calibration = cfg['camera_calibration']

        self.camera_calibration_dir = os.path.join(
            package_share,
            'shape_reconstruction',
            'calibration',
            f"sensor_{cfg['sensor_id']}",
            camera_calibration['camera_calibration_dir']
        )

        self.row_index_path = os.path.join(self.camera_calibration_dir, camera_calibration['row_index_path'])
        self.col_index_path = os.path.join(self.camera_calibration_dir, camera_calibration['col_index_path'])
        self.position_scale_path = os.path.join(self.camera_calibration_dir, camera_calibration['position_scale_path'])

        self.crop_img_height = camera_calibration['crop_size'][0]
        self.crop_img_width = camera_calibration['crop_size'][1]

        if calibrated:
            self.row_index = np.load(self.row_index_path)
            self.col_index = np.load(self.col_index_path)
            position_scale = np.load(self.position_scale_path)
            center_position = position_scale[0:2]
            self.pixel_per_mm = position_scale[2]
            self.height_begin = int(center_position[0] - self.crop_img_height / 2)
            self.height_end = int(center_position[0] + self.crop_img_height / 2)
            self.width_begin = int(center_position[1] - self.crop_img_width / 2)
            self.width_end = int(center_position[1] + self.crop_img_width / 2)

    def get_raw_image(self):
        return self.cap.read()[1]

    def rectify_image(self, img):
        return img[self.row_index, self.col_index]

    def crop_image(self, img):
        return img[self.height_begin:self.height_end, self.width_begin:self.width_end]

    def rectify_crop_image(self, img):
        return self.crop_image(self.rectify_image(img))

    def get_rectify_image(self):
        return self.rectify_image(self.get_raw_image())

    def get_rectify_crop_image(self):
        return self.crop_image(self.get_rectify_image())

    def get_raw_avg_image(self, n=10):
        img_add = None
        for _ in range(n):
            ret, img = self.cap.read()
            if not ret:
                continue
            if img_add is None:
                img_add = np.zeros_like(img, dtype=np.float32)
            img_add += img.astype(np.float32)
        return (img_add / n).astype(np.uint8)

    def get_rectify_avg_image(self, n=10):
        img_add = None
        for _ in range(n):
            img = self.get_rectify_image()
            if img_add is None:
                img_add = np.zeros_like(img, dtype=np.float32)
            img_add += img.astype(np.float32)
        return (img_add / n).astype(np.uint8)

    def get_rectify_crop_avg_image(self, n=10):
        img_add = None
        for _ in range(n):
            img = self.get_rectify_crop_image()
            if img_add is None:
                img_add = np.zeros_like(img, dtype=np.float32)
            img_add += img.astype(np.float32)
        return (img_add / n).astype(np.uint8)

    def img_list_avg_rectify(self, img_list):
        img_add = None
        for img_path in img_list:
            img = cv2.imread(img_path)
            if img_add is None:
                img_add = np.zeros_like(img, dtype=np.float32)
            img_add += img.astype(np.float32)
        img_avg = (img_add / len(img_list)).astype(np.uint8)
        return self.rectify_image(img_avg)


if __name__ == '__main__':
    f = open("shape_config.yaml", 'r', encoding='utf-8')
    cfg = yaml.load(f, Loader=yaml.FullLoader)
    camera = Camera(cfg)

    while True:
        raw_img = camera.get_raw_image()
        cv2.imshow('raw_img', raw_img)
        cv2.imshow('raw_img_GRAY', cv2.cvtColor(raw_img, cv2.COLOR_BGR2GRAY))

        rectify_img = camera.get_rectify_image()
        cv2.imshow('rectify_img', rectify_img)

        rectify_crop_img = camera.get_rectify_crop_image()
        cv2.imshow('rectify_crop_img', rectify_crop_img)
        cv2.imshow('rectify_crop_GRAY', cv2.cvtColor(rectify_crop_img, cv2.COLOR_BGR2GRAY))

        if cv2.waitKey(1) == ord('q'):
            break

    cv2.destroyAllWindows()
