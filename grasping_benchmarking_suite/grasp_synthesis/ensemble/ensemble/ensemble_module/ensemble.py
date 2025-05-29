import torch
import numpy as np
from skimage.filters import gaussian

from os import path
import sys

here = path.dirname(path.abspath(__file__))
sys.path.append(here)

from Ensembles.HiFormer.HiFormer import HiFormer


class Ensemble:
    def __init__(self):
        """
        expert1 - ggcnn_cornell
        expert2 - ggcnn_jacquard
        expert3 - gpnn_cornell
        expert4 - gpnn_jacquard
        expert5 - grcnn_cornell
        expert6 - grcnn_jacquard
        ensemble - hiformer
        """
        self.models = {
            'expert1': None,
            'expert2': None,
            'expert3' : None,
            'expert4': None,
            'expert5': None,
            'expert6': None,
            'ensemble' : None
        }
    
    def predict_model(self, model:torch.nn.Module, image_tensor:torch.Tensor):
        image_tensor = image_tensor.to(next(model.parameters()).device)
        model.eval()

        with torch.no_grad():
            pos_output, cos_output, sin_output, width_output =  model(image_tensor)
        q_img, ang_img, width_img = self._post_process_output(pos_output, cos_output, sin_output, width_output) # post process converts them to cpu
        pred = torch.stack([torch.from_numpy(q_img), torch.from_numpy(ang_img), torch.from_numpy(width_img)], dim=0)

        return pred

    def predict(self, rgb_img, depth_img):
        depth_img = np.clip((depth_img - depth_img.mean()), -1, 1)
        depth_tensor = torch.from_numpy(depth_img.reshape(1, 1, depth_img.shape[0], depth_img.shape[1]).astype(np.float32))
        
        rgb_img = rgb_img.astype(np.float32) / 255.0
        rgb_img -= rgb_img.mean()
        rgb_img = rgb_img.transpose((2, 0, 1))

        rgb_tensor = torch.from_numpy(rgb_img.reshape(1, 3, depth_img.shape[0], depth_img.shape[1]).astype(np.float32))

        rgbd_tensor = torch.cat((rgb_tensor, depth_tensor), dim=1)

        preds = []
        # preds.append(self.predict_model(self.models['expert1'], depth_tensor))  # [3, H, W]
        # preds.append(self.predict_model(self.models['expert2'], depth_tensor))
        # preds.append(self.predict_model(self.models['expert3'], rgbd_tensor))
        # preds.append(self.predict_model(self.models['expert4'], rgbd_tensor))
        # preds.append(self.predict_model(self.models['expert5'], rgbd_tensor))
        # preds.append(self.predict_model(self.models['expert6'], rgbd_tensor))
        preds.append(self.predict_model(self.models['expert7'], rgbd_tensor))

        # # Stack predictions to shape [1, 18, 300, 300]
        # pred_tensor = torch.cat([p.unsqueeze(0) for p in preds], dim=1)  # each p: [3, H, W], becomes [1, 3, H, W], then concat

        # # Final stacking with rgbd_tensor: [1, 18 + 4, 300, 300] = [1, 22, 300, 300]
        # combined_tensor = torch.cat([pred_tensor, rgbd_tensor], dim=1)
        # preds.append(self.predict_model(self.models['ensemble'], combined_tensor))
        
        pred = preds[-1]
        q_img = pred[0].numpy()
        ang_img = pred[1].numpy()
        width_img = pred[2].numpy()
        
        return q_img, ang_img, width_img


    def init_model(self, model_name:str, model_path:str, model_device:str='cpu', is_hiformer=False, in_channels=21):
        self.models[model_name] = self._get_torch_model(model_path, model_device, is_hiformer, in_channels)

    def _get_torch_model(self, MODEL_FILE:str, device:str='cpu', is_hiformer=False, in_channels=21)->None:
        if not is_hiformer:
            return torch.load(MODEL_FILE, map_location=device)
        else:
            model = HiFormer(in_chans=in_channels)
            checkpoint = torch.load(MODEL_FILE, map_location=device)
            model.load_state_dict(checkpoint)
            return model
    
    def _post_process_output(self, q_img, cos_img, sin_img, width_img):
        """
        Post-process the raw output of the GG-CNN, convert to numpy arrays, apply filtering.
        :param q_img: Q output of GG-CNN (as torch Tensors)
        :param cos_img: cos output of GG-CNN
        :param sin_img: sin output of GG-CNN
        :param width_img: Width output of GG-CNN
        :return: Filtered Q output, Filtered Angle output, Filtered Width output
        """
        q_img = q_img.cpu().numpy().squeeze()
        ang_img = (torch.atan2(sin_img, cos_img) / 2.0).cpu().numpy().squeeze()
        width_img = width_img.cpu().numpy().squeeze() * 150.0

        q_img = gaussian(q_img, 2.0, preserve_range=True)
        ang_img = gaussian(ang_img, 2.0, preserve_range=True)
        width_img = gaussian(width_img, 1.0, preserve_range=True)

        return q_img, ang_img, width_img
