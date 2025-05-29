import numpy as np
import cv2

def process_depth_image(depth, crop_size, out_size=300, return_mask=False, crop_y_offset=0):
    imh, imw = depth.shape
    
    # Crop image from v*u to v*v
    depth_crop = depth[(imh - crop_size) // 2 - crop_y_offset:(imh - crop_size) // 2 + crop_size - crop_y_offset,
                        (imw - crop_size) // 2:(imw - crop_size) // 2 + crop_size]
 
    # Inpaint the depth image
    # OpenCV inpainting does weird things at the border.
    depth_crop = cv2.copyMakeBorder(depth_crop, 1, 1, 1, 1, cv2.BORDER_DEFAULT)
    depth_nan_mask = np.isnan(depth_crop).astype(np.uint8)

    depth_crop[depth_nan_mask==1] = 0

    # Scale to keep as float, but has to be in bounds -1:1 to keep opencv happy.
    depth_scale = np.abs(depth_crop).max()
    depth_crop = depth_crop.astype(np.float32) / depth_scale  # Has to be float32, 64 not supported.

    # Inpaint
    depth_crop = cv2.inpaint(depth_crop, depth_nan_mask, 1, cv2.INPAINT_NS)

    # Back to original size and value range.
    depth_crop = depth_crop[1:-1, 1:-1]
    depth_crop = depth_crop * depth_scale

    # Resize cropped depth image 
    depth_crop = cv2.resize(depth_crop, (out_size, out_size), interpolation=cv2.INTER_AREA)

    if return_mask:
        # Resize Nan mask 
        depth_nan_mask = depth_nan_mask[1:-1, 1:-1]
        depth_nan_mask = cv2.resize(depth_nan_mask, (out_size, out_size), interpolation=cv2.INTER_NEAREST)
        return depth_crop, depth_nan_mask
    else:
        return depth_crop

def process_rgb(rgb, crop_size, out_size=300, crop_y_offset=0):
    imh, imw, _ = rgb.shape

    # Calculate crop coordinates
    top = (imh - crop_size) // 2 - crop_y_offset
    left = (imw - crop_size) // 2
    bottom = top + crop_size
    right = left + crop_size

    # Crop the image
    rgb_crop = rgb[top:bottom, left:right, :]

    # Resize the image
    rgb_crop = cv2.resize(rgb_crop, (out_size, out_size), interpolation=cv2.INTER_AREA)

    return rgb_crop