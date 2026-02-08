"""MinkLoc3D-style place recognition with GeM pooling for LiDAR point clouds"""

from __future__ import annotations

import logging
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

logger = logging.getLogger(__name__)

try:
    import MinkowskiEngine as ME

    HAS_MINKOWSKI = True
except ImportError:
    HAS_MINKOWSKI = False


class GeM(nn.Module):
    """generalized mean pooling (learnable power parameter)"""

    def __init__(self, p: float = 3.0, eps: float = 1e-6) -> None:
        super().__init__()
        self.p = nn.Parameter(torch.ones(1) * p)
        self.eps = eps

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x_clamped = x.clamp(min=self.eps).pow(self.p)
        return x_clamped.mean(dim=0, keepdim=True).pow(1.0 / self.p)


class MinkLoc3D(nn.Module):
    """MinkLoc3D: sparse 3D conv backbone + GeM pooling for global descriptors"""

    def __init__(
        self,
        in_channels: int = 1,
        feature_dim: int = 256,
        backbone_channels: tuple[int, ...] = (32, 64, 64),
        voxel_size: float = 0.1,
    ) -> None:
        super().__init__()

        if not HAS_MINKOWSKI:
            raise RuntimeError(
                "MinkowskiEngine is required for MinkLoc3D. "
                "Install with: pip install MinkowskiEngine"
            )

        from src.models.feature_extraction import MinkFPN

        self.voxel_size = voxel_size
        self.backbone = MinkFPN(
            in_channels=in_channels,
            out_channels=feature_dim,
            block_channels=backbone_channels,
        )
        self.pool = GeM()

    def forward(self, x: ME.SparseTensor) -> torch.Tensor:
        features = self.backbone(x)  # SparseTensor

        # pool per-batch
        batch_size = features.C[:, 0].max().item() + 1
        descriptors = []
        for b in range(batch_size):
            mask = features.C[:, 0] == b
            batch_feats = features.F[mask]
            pooled = self.pool(batch_feats)
            descriptors.append(pooled)

        descriptors = torch.cat(descriptors, dim=0)
        # l2 normalize
        descriptors = F.normalize(descriptors, p=2, dim=1)
        return descriptors


class PlaceRecognitionWrapper(nn.Module):
    """unified place recognition interface (minkloc3d and pointnet backends)"""

    def __init__(
        self,
        model_type: str = "minkloc3d",
        feature_dim: int = 256,
        voxel_size: float = 0.1,
    ) -> None:
        super().__init__()
        self.model_type = model_type
        self.voxel_size = voxel_size

        if model_type == "minkloc3d":
            self.model = MinkLoc3D(
                feature_dim=feature_dim, voxel_size=voxel_size,
            )
        elif model_type == "pointnet":
            from src.models.feature_extraction import PointNetSimple

            self.model = PointNetSimple(
                in_channels=3, out_channels=feature_dim,
            )
        else:
            raise ValueError(f"Unknown model type: {model_type}")

    def forward(
        self, points: torch.Tensor | list[np.ndarray],
    ) -> torch.Tensor:
        """extract (B, feature_dim) normalized descriptors from point clouds"""
        if self.model_type == "minkloc3d":
            if isinstance(points, list):
                from src.models.feature_extraction import (
                    pointcloud_to_sparse_tensor,
                )

                device = next(self.model.parameters()).device
                coords_list = []
                feats_list = []
                for pc in points:
                    c = np.floor(pc[:, :3] / self.voxel_size).astype(np.int32)
                    coords_list.append(torch.from_numpy(c).int())
                    feats_list.append(
                        torch.ones((len(c), 1), dtype=torch.float32)
                    )

                coords = ME.utils.batched_coordinates(coords_list)
                feats = torch.cat(feats_list, dim=0)
                points = ME.SparseTensor(
                    features=feats, coordinates=coords, device=device,
                )

            return self.model(points)
        else:
            if isinstance(points, list):
                # pad and stack
                max_n = max(len(p) for p in points)
                batch = np.zeros((len(points), max_n, 3), dtype=np.float32)
                for i, p in enumerate(points):
                    batch[i, : len(p), :3] = p[:, :3]
                points = torch.from_numpy(batch)
                device = next(self.model.parameters()).device
                points = points.to(device)
            return F.normalize(self.model(points), p=2, dim=1)


class TripletLoss(nn.Module):
    """triplet margin loss for metric learning"""

    def __init__(self, margin: float = 0.2, mining: str = "hard") -> None:
        super().__init__()
        self.margin = margin
        self.mining = mining
        self.loss_fn = nn.TripletMarginLoss(margin=margin, p=2)

    def forward(
        self,
        anchor: torch.Tensor,
        positive: torch.Tensor,
        negative: torch.Tensor,
    ) -> torch.Tensor:
        return self.loss_fn(anchor, positive, negative)


def load_model(
    checkpoint_path: str | Path,
    model_type: str = "minkloc3d",
    feature_dim: int = 256,
    device: str = "cpu",
) -> PlaceRecognitionWrapper:
    """load trained place recognition model from checkpoint into eval mode"""
    model = PlaceRecognitionWrapper(
        model_type=model_type, feature_dim=feature_dim,
    )

    checkpoint = torch.load(checkpoint_path, map_location=device, weights_only=True)
    if "model_state_dict" in checkpoint:
        model.load_state_dict(checkpoint["model_state_dict"])
    else:
        model.load_state_dict(checkpoint)

    model = model.to(device)
    model.eval()
    logger.info("Loaded %s model from %s", model_type, checkpoint_path)
    return model
