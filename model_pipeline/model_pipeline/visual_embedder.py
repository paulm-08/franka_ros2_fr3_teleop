# model_pipeline/visual_embedder.py
import torch
import torch.nn as nn
from torchvision import transforms, models
import numpy as np
import cv2

class VisualEmbedder:
    def __init__(self, backbone="resnet18", device=None, pretrained=True, out_dim=None, global_depth_range=None):
        if device is None:
            device = "cuda" if torch.cuda.is_available() else "cpu"
        self.device = torch.device(device)
        self.backbone_name = backbone

        # --- CNN Encoder (remains the same) ---
        if backbone == "resnet18":
            model = models.resnet18(pretrained=pretrained)
            self.feat_dim = 512
            modules = list(model.children())[:-1]
            self.encoder = nn.Sequential(*modules)
            self.post_flatten = lambda t: t.reshape(t.shape[0], -1)
        else: # mobilenet_v3_small
            model = models.mobilenet_v3_small(pretrained=pretrained)
            self.feat_dim = model.classifier[0].in_features
            modules = list(model.features)
            self.encoder = nn.Sequential(*modules, nn.AdaptiveAvgPool2d((1,1)))
            self.post_flatten = lambda t: t.reshape(t.shape[0], -1)

        self.encoder = self.encoder.to(self.device).eval()

        # --- FIX: Always use a dictionary for out_dim and create separate projectors ---
        if out_dim is None:
            # Provide a default dictionary if none is given
            out_dim = {'rgb': 256, 'depth': 128}
        self.out_dim = out_dim
        
        # Create a specific projection layer for RGB features
        self.project_rgb = nn.Linear(self.feat_dim, self.out_dim['rgb']).to(self.device)
        # Create a specific projection layer for Depth features
        self.project_depth = nn.Linear(self.feat_dim, self.out_dim['depth']).to(self.device)
        # ---

        # Store the global depth range
        if global_depth_range:
            self.global_min_depth, self.global_max_depth = global_depth_range
        else:
            self.global_min_depth, self.global_max_depth = 0.0, 1.0 # Fallback

        # ImageNet normalization transforms
        self.transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                 std=[0.229, 0.224, 0.225]),
        ])

    @torch.no_grad()
    def embed_rgb(self, img_bgr):
        if img_bgr is None:
            return None
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        t = self.transform(img_rgb).unsqueeze(0).to(self.device)
        feats = self.encoder(t)
        feats = self.post_flatten(feats)
        
        # FIX: Use the dedicated RGB projector
        projected_feats = self.project_rgb(feats)
        return projected_feats.squeeze(0).cpu().numpy()

    @torch.no_grad()
    def embed_depth(self, depth_img):
        if depth_img is None:
            return None
        
        # Clip and normalize using global range
        depth_clipped = np.clip(depth_img, self.global_min_depth, self.global_max_depth)
        depth_normalized = (depth_clipped - self.global_min_depth) / (self.global_max_depth - self.global_min_depth + 1e-9)
        depth_u8 = (depth_normalized * 255.0).astype(np.uint8)
        
        # FIX: Simplified the redundant conversion
        depth_rgb_for_model = cv2.cvtColor(depth_u8, cv2.COLOR_GRAY2RGB)
        
        # Process through the same pipeline as embed_rgb but use the dedicated depth projector
        t = self.transform(depth_rgb_for_model).unsqueeze(0).to(self.device)
        feats = self.encoder(t)
        feats = self.post_flatten(feats)
        
        # FIX: Use the dedicated Depth projector
        projected_feats = self.project_depth(feats)
        return projected_feats.squeeze(0).cpu().numpy()

    def to_torchscript(self, sample_input_rgb):
        # optional: torch.jit.trace export for faster inference
        self.encoder.eval()
        example = torch.from_numpy(sample_input_rgb).unsqueeze(0).to(self.device)
        # Can't easily trace because of transforms; leave this as a TODO if needed.
        raise NotImplementedError("Use tracing on a pipeline wrapper if needed.")
