import pybullet as p
import numpy as np

def get_object_position_from_camera(object_id, env_params):
    """Return the 3D position of an object using camera and segmentation mask."""
    cam_params = env_params["camera_params"]
    width = cam_params["width"]
    height = cam_params["height"]
    view_matrix = cam_params["view_matrix"]
    projection_matrix = cam_params["projection_matrix"]

    (_, _, _, depth_buffer, seg_mask) = p.getCameraImage(
        width=width,
        height=height,
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix,
        renderer=p.ER_BULLET_HARDWARE_OPENGL
    )

    seg_mask = np.array(seg_mask).reshape(height, width)
    depth_buffer = np.array(depth_buffer).reshape(height, width)
    object_pixels = np.argwhere(seg_mask == object_id)

    if len(object_pixels) == 0:
        return None

    center_pixel_y, center_pixel_x = np.mean(object_pixels, axis=0).astype(int)
    depth_at_center = depth_buffer[center_pixel_y, center_pixel_x]

    x_ndc = (2.0 * center_pixel_x / width) - 1.0
    y_ndc = 1.0 - (2.0 * center_pixel_y / height)
    z_ndc = 2.0 * depth_at_center - 1.0
    ndc_point = np.array([x_ndc, y_ndc, z_ndc, 1.0])

    view_matrix_np = np.array(view_matrix).reshape(4, 4, order='F')
    projection_matrix_np = np.array(projection_matrix).reshape(4, 4, order='F')
    view_proj_matrix = np.matmul(projection_matrix_np, view_matrix_np)
    inverse_view_proj_matrix = np.linalg.inv(view_proj_matrix)

    world_point = np.matmul(inverse_view_proj_matrix, ndc_point)
    world_position = world_point[:3] / world_point[3]

    return world_position
