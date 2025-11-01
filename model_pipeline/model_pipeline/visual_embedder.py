# model_pipeline/visual_embedder.py
import torch
import torch.nn as nn
from torchvision import transforms, models
import numpy as np
import cv2

class VisualEmbedder(nn.Module):
    """
    A deterministic CNN-based visual feature extractor for RGB and depth images.
    Wraps a pretrained backbone (ResNet18 or MobileNetV3) and projects to fixed embeddings.
    """

    def __init__(
        self,
        backbone: str = "resnet18",
        device: str | None = None,
        pretrained: bool = True,
        out_dim: dict[str, int] | None = None,
        global_depth_range: tuple[float, float] | None = None,
    ):
        super().__init__()

        # --- Device setup ---
        if device is None:
            device = "cuda" if torch.cuda.is_available() else "cpu"
        self.device = torch.device(device)
        self.backbone_name = backbone

        # --- Load backbone ---
        if backbone == "resnet18":
            weights = models.ResNet18_Weights.DEFAULT if pretrained else None
            base_model = models.resnet18(weights=weights)
            self.feat_dim = 512
            modules = list(base_model.children())[:-1]  # drop final FC
            self.encoder = nn.Sequential(*modules)
            self.post_flatten = lambda t: t.reshape(t.shape[0], -1)
        elif backbone == "mobilenet_v3_small":
            weights = models.MobileNet_V3_Small_Weights.DEFAULT if pretrained else None
            base_model = models.mobilenet_v3_small(weights=weights)
            self.feat_dim = base_model.classifier[0].in_features
            modules = list(base_model.features)
            self.encoder = nn.Sequential(*modules, nn.AdaptiveAvgPool2d((1, 1)))
            self.post_flatten = lambda t: t.reshape(t.shape[0], -1)
        else:
            raise ValueError(f"Unsupported backbone: {backbone}")

        # --- Projection layers ---
        if out_dim is None:
            out_dim = {"rgb": 256, "depth": 128}
        self.out_dim = out_dim

        self.project_rgb = nn.Linear(self.feat_dim, self.out_dim["rgb"])
        self.project_depth = nn.Linear(self.feat_dim, self.out_dim["depth"])

        # --- Global depth range ---
        if global_depth_range:
            self.global_min_depth, self.global_max_depth = global_depth_range
        else:
            self.global_min_depth, self.global_max_depth = 0.0, 1.0  # fallback

        # --- Image normalization ---
        self.transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(
                mean=[0.485, 0.456, 0.406],
                std=[0.229, 0.224, 0.225],
            ),
        ])

        # --- Determinism and eval mode ---
        self.to(self.device)
        self.eval()
        torch.set_grad_enabled(False)
        torch.backends.cudnn.benchmark = False
        torch.backends.cudnn.deterministic = True

    # ------------------------------------------------------------
    # Forward utilities
    # ------------------------------------------------------------

    @torch.no_grad()
    def embed_rgb(self, img_bgr: np.ndarray) -> np.ndarray | None:
        """Embed a single BGR RGB image into a fixed-length feature vector."""
        if img_bgr is None:
            return None
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        t = self.transform(img_rgb).unsqueeze(0).to(self.device)
        feats = self.encoder(t)
        feats = self.post_flatten(feats)
        projected_feats = self.project_rgb(feats)
        return projected_feats.squeeze(0).cpu().numpy()

    @torch.no_grad()
    def embed_depth(self, depth_img: np.ndarray) -> np.ndarray | None:
        """Embed a single depth image into a fixed-length feature vector."""
        if depth_img is None:
            return None

        # Clip and normalize using global range
        dmin, dmax = self.global_min_depth, self.global_max_depth
        depth_clipped = np.clip(depth_img, dmin, dmax)
        depth_normalized = (depth_clipped - dmin) / (dmax - dmin + 1e-9)
        depth_u8 = (depth_normalized * 255.0).astype(np.uint8)

        depth_rgb = cv2.cvtColor(depth_u8, cv2.COLOR_GRAY2RGB)
        t = self.transform(depth_rgb).unsqueeze(0).to(self.device)
        feats = self.encoder(t)
        feats = self.post_flatten(feats)
        projected_feats = self.project_depth(feats)
        return projected_feats.squeeze(0).cpu().numpy()

    # ------------------------------------------------------------
    # TorchScript export (optional)
    # ------------------------------------------------------------

    def to_torchscript(self, sample_input_rgb: np.ndarray):
        """(Optional) TorchScript export stub."""
        example = torch.from_numpy(sample_input_rgb).unsqueeze(0).to(self.device)
        self.eval()
        raise NotImplementedError("Tracing must be done from a pipeline wrapper.")
