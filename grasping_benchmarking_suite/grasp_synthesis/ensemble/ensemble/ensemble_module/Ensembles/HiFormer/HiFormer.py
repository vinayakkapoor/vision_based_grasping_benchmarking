import torch.nn as nn
from einops.layers.torch import Rearrange

from .Encoder import All2Cross
from .Decoder import ConvUpsample, SegmentationHead

from .HiFormer_configs import *

import torch.nn.functional as F

class HiFormer(nn.Module):
    def __init__(self, img_size=224, in_chans=3, n_classes=64, size='s'):
        super().__init__()
        self.img_size = img_size
        # self.img_size = 224
        self.patch_size = [4, 16]
        self.n_classes = n_classes
        if size == 's':
            config = get_hiformer_s_configs()
        elif size == 'b':
            config = get_hiformer_b_configs()
        elif size == 'l':
            config = get_hiformer_l_configs()
        else:
            print("Invalid config")
            raise Exception
        self.All2Cross = All2Cross(config = config, img_size = img_size, in_chans=in_chans)
        
        self.ConvUp_s = ConvUpsample(in_chans=384, out_chans=[128,128], upsample=True)
        self.ConvUp_l = ConvUpsample(in_chans=96, upsample=False)

        self.upsample = nn.Upsample(size=(300, 300), mode='bilinear', align_corners=False)
    
        self.segmentation_head = SegmentationHead(
            in_channels=16,
            out_channels=n_classes,
            kernel_size=3,
        )    

        self.conv_pred = nn.Sequential(
            nn.Conv2d(
                128, 16,
                kernel_size=1, stride=1,
                padding=0, bias=True),
            # nn.GroupNorm(8, 16), 
            nn.ReLU(inplace=True),
            nn.Upsample(scale_factor=4, mode='bilinear', align_corners=False)
        )
        self.in_downsample = InConv(in_chans)
        self.pos_output = (OutConv(n_classes, 1))
        self.cos_output = (OutConv(n_classes, 1))
        self.sin_output = (OutConv(n_classes, 1))
        self.width_output = (OutConv(n_classes, 1))
    
    def forward(self, x):
        x = self.in_downsample(x)
        # print(x.shape)
        xs = self.All2Cross(x)
        embeddings = [x[:, 1:] for x in xs]
        reshaped_embed = []
        for i, embed in enumerate(embeddings):

            embed = Rearrange('b (h w) d -> b d h w', h=(self.img_size//self.patch_size[i]), w=(self.img_size//self.patch_size[i]))(embed)
            embed = self.ConvUp_l(embed) if i == 0 else self.ConvUp_s(embed)
            
            reshaped_embed.append(embed)

        C = reshaped_embed[0] + reshaped_embed[1]
        C = self.conv_pred(C)

        # upsampled = self.upsample(C) #changed

        out = self.segmentation_head(C)

        pos_output = self.pos_output(out)
        cos_output = self.cos_output(out)
        sin_output = self.sin_output(out)
        width_output = self.width_output(out)

        return pos_output, cos_output, sin_output, width_output
        

    def compute_loss(self, xc, yc):
        y_pos, y_cos, y_sin, y_width = yc
        pos_pred, cos_pred, sin_pred, width_pred = self(xc)

        p_loss = F.mse_loss(pos_pred, y_pos)
        cos_loss = F.mse_loss(cos_pred, y_cos)
        sin_loss = F.mse_loss(sin_pred, y_sin)
        width_loss = F.mse_loss(width_pred, y_width)

        return {
            'loss': p_loss + cos_loss + sin_loss + width_loss,
            'losses': {
                'p_loss': p_loss,
                'cos_loss': cos_loss,
                'sin_loss': sin_loss,
                'width_loss': width_loss
            },
            'pred': {
                'pos': pos_pred,
                'cos': cos_pred,
                'sin': sin_pred,
                'width': width_pred
            }
        }
    
    def predict(self, xc):
        pos_pred, cos_pred, sin_pred, width_pred = self(xc)
        return {
            'pos': pos_pred,
            'cos': cos_pred,
            'sin': sin_pred,
            'width': width_pred
        } 

# class OutConv(nn.Module):
#     def __init__(self, in_channels, out_channels):
#         super(OutConv, self).__init__()
#         self.pool1 = nn.AdaptiveMaxPool2d((250, 250))
#         self.residual_block1 = ResidualBlock(in_channels, in_channels)

#         self.pool2 = nn.AdaptiveMaxPool2d((275, 275))
#         self.residual_block2 = ResidualBlock(in_channels, in_channels)

#         self.pool3 = nn.AdaptiveMaxPool2d((300, 300))

#         self.conv = nn.Conv2d(in_channels, out_channels, kernel_size=1)

#     def forward(self, x):
#         x = self.pool1(x)
#         x = self.residual_block1(x)
#         x = self.pool2(x)
#         x = self.residual_block2(x)
#         x = self.pool3(x)
#         x = self.conv(x)
        
#         return x


# class InConv(nn.Module):
#     def __init__(self, in_channels):
#         super(InConv, self).__init__()
#         self.pool1 = nn.AdaptiveMaxPool2d((275, 275))
#         self.residual_block1 = ResidualBlock(in_channels, in_channels)

#         self.pool2 = nn.AdaptiveMaxPool2d((250, 250))
#         self.residual_block2 = ResidualBlock(in_channels, in_channels)

#         self.pool3 = nn.AdaptiveMaxPool2d((224, 224))

#     def forward(self, x):
#         x = self.pool1(x)
#         x = self.residual_block1(x)
#         x = self.pool2(x)
#         x = self.residual_block2(x)
#         x = self.pool3(x)

#         return x

# class ResidualBlock(nn.Module):
#     def __init__(self, in_channels, out_channels, stride=1):
#         super(ResidualBlock, self).__init__()
#         self.conv1 = nn.Conv2d(in_channels, out_channels, kernel_size=3, stride=stride, padding=1, bias=False)
#         self.bn1 = nn.BatchNorm2d(out_channels)
#         self.relu = nn.ReLU(inplace=True)
#         self.conv2 = nn.Conv2d(out_channels, out_channels, kernel_size=3, stride=1, padding=1, bias=False)
#         self.bn2 = nn.BatchNorm2d(out_channels)

#     def forward(self, x):
#         identity = x

#         out = self.conv1(x)
#         out = self.bn1(out)
#         out = self.relu(out)
#         out = self.conv2(out)
#         out = self.bn2(out)

#         out += identity
#         out = self.relu(out)

#         return out

class InConv(nn.Module):
    def __init__(self, in_channels):
        super(InConv, self).__init__()
        self.start_h = (300 - 224) // 2
        self.start_w = (300 - 224) // 2  # (38, 38)
        self.end_h = self.start_h + 224
        self.end_w = self.start_w + 224  # (262, 262)

    def forward(self, x):
        x = x[:, :, self.start_h:self.end_h, self.start_w:self.end_w]
        return x


class OutConv(nn.Module):
    def __init__(self, in_channels, out_channels):
        super(OutConv, self).__init__()
        self.conv = nn.Conv2d(in_channels, out_channels, kernel_size=1)

    def forward(self, x):
        x = F.pad(x, (38, 38, 38, 38), mode='constant', value=0)
        x = self.conv(x)        
        return x
