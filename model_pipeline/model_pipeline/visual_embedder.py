# model_pipeline/visual_embedder.py
import torch
import torch.nn as nn
from torchvision import transforms, models
import numpy as np
import cv2

class VisualEmbedder:
    def __init__(self, backbone="resnet18", device=None, pretrained=True, out_dim=None):
        """
        backbone: "resnet18" or "mobilenet_v3_small" (or "efficientnet_b0")
        device: "cpu" or "cuda"
        out_dim: if set, will project features to this dimension with a linear layer.
        """
        if device is None:
            device = "cuda" if torch.cuda.is_available() else "cpu"
        self.device = torch.device(device)
        self.backbone_name = backbone

        if backbone == "resnet18":
            model = models.resnet18(pretrained=pretrained)
            feat_dim = 512
            modules = list(model.children())[:-1]  # remove fc
            self.encoder = nn.Sequential(*modules)  # outputs (B,512,1,1)
            self.post_flatten = lambda t: t.reshape(t.shape[0], -1)
        elif backbone == "mobilenet_v3_small":
            model = models.mobilenet_v3_small(pretrained=pretrained)
            feat_dim = model.classifier[0].in_features
            modules = list(model.features)
            # Build a small wrapper to do features -> pooled vector
            self.encoder = nn.Sequential(*modules, nn.AdaptiveAvgPool2d((1,1)))
            self.post_flatten = lambda t: t.reshape(t.shape[0], -1)
        else:
            raise ValueError("Unsupported backbone")

        self.encoder = self.encoder.to(self.device).eval()
        # optional projection
        if out_dim is not None and out_dim != feat_dim:
            self.project = nn.Linear(feat_dim, out_dim).to(self.device)
            self.out_dim = out_dim
        else:
            self.project = None
            self.out_dim = feat_dim

        # transforms: resize -> center crop -> to tensor -> normalize with ImageNet mean/std
        self.transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((224,224)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485,0.456,0.406],
                                 std=[0.229,0.224,0.225]),
        ])

    @torch.no_grad()
    def embed_rgb(self, img_bgr):
        """
        img_bgr: HxWx3 BGR numpy (as from cv2.imread). Returns numpy vector (out_dim,)
        """
        if img_bgr is None:
            return None
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        t = self.transform(img_rgb).unsqueeze(0).to(self.device)  # 1,C,H,W
        feats = self.encoder(t)           # (1,feat_dim,1,1) or (1,feat_dim,H',W')
        feats = self.post_flatten(feats)  # (1, feat_dim)
        if self.project is not None:
            feats = self.project(feats)
        return feats.squeeze(0).cpu().numpy()

    @torch.no_grad()
    def embed_depth(self, depth_img, mode="replicate"):
        """
        depth_img: HxW single-channel numpy (uint8 or float) - normalized or not.
        mode: how to feed depth to RGB encoder.
          - "replicate": stack depth into 3 channels and use RGB encoder (quick).
          - "small_cnn": (not implemented here) use a tiny conv for depth.
        """
        if depth_img is None:
            return None
        # ensure depth is 0..255 uint8
        if depth_img.dtype != np.uint8:
            # normalize to 0..255
            dmin, dmax = float(depth_img.min()), float(depth_img.max())
            if dmax - dmin > 1e-6:
                depth_u8 = ((depth_img - dmin) / (dmax - dmin) * 255.0).astype(np.uint8)
            else:
                depth_u8 = (depth_img * 0).astype(np.uint8)
        else:
            depth_u8 = depth_img
        # replicate to 3 channels
        if mode == "replicate":
            depth_rgb = cv2.cvtColor(depth_u8, cv2.COLOR_GRAY2BGR)
            return self.embed_rgb(depth_rgb)
        else:
            raise NotImplementedError("mode not implemented")

    def to_torchscript(self, sample_input_rgb):
        # optional: torch.jit.trace export for faster inference
        self.encoder.eval()
        example = torch.from_numpy(sample_input_rgb).unsqueeze(0).to(self.device)
        # Can't easily trace because of transforms; leave this as a TODO if needed.
        raise NotImplementedError("Use tracing on a pipeline wrapper if needed.")
