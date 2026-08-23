import numpy as np

def proccesing_img_simu(frame_raw) -> tuple[np.ndarray, str, dict[str:float]]:
    frame_pro = frame_raw
    message = 'Error pero ha funcionado.'
    pose = {'x': 100.0, 'y':100.0}
    return frame_pro, message, pose