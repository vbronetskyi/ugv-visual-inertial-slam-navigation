"""sparse 3D convolutional feature extraction using MinkowskiEngine"""

from __future__ import annotations

import logging
from typing import Any

import numpy as np
import torch
import torch.nn as nn

logger = logging.getLogger(__name__)

try:
    import MinkowskiEngine as ME

    HAS_MINKOWSKI = True
except ImportError:
    HAS_MINKOWSKI = False
    logger.warning(
        "MinkowskiEngine not installed. Sparse conv models unavailable. "
        "Install with: pip install MinkowskiEngine"
    )


class MinkFPN(nn.Module):
    """U-Net style sparse 3D convolutional FPN for multi-scale point cloud features"""

    def __init__(
        self,
        in_channels: int = 1,
        out_channels: int = 256,
        num_top_down: int = 2,
        conv0_kernel_size: int = 5,
        block_channels: tuple[int, ...] = (32, 64, 64),
    ) -> None:
        super().__init__()

        if not HAS_MINKOWSKI:
            raise RuntimeError(
                "MinkowskiEngine is required for MinkFPN. "
                "Install with: pip install MinkowskiEngine"
            )

        self.in_channels = in_channels
        self.out_channels = out_channels

        # encoder (bottom-up)
        self.conv0 = ME.MinkowskiConvolution(
            in_channels, block_channels[0],
            kernel_size=conv0_kernel_size, dimension=3,
        )
        self.bn0 = ME.MinkowskiBatchNorm(block_channels[0])
        self.relu = ME.MinkowskiReLU(inplace=True)

        self.encoder_blocks = nn.ModuleList()
        self.encoder_pools = nn.ModuleList()
        for i in range(len(block_channels) - 1):
            self.encoder_blocks.append(
                self._make_block(block_channels[i], block_channels[i + 1])
            )
            self.encoder_pools.append(
                ME.MinkowskiConvolution(
                    block_channels[i + 1], block_channels[i + 1],
                    kernel_size=2, stride=2, dimension=3,
                )
            )

        # decoder (top-down)
        self.decoder_blocks = nn.ModuleList()
        self.decoder_ups = nn.ModuleList()
        for i in range(min(num_top_down, len(block_channels) - 1)):
            ch = block_channels[-(i + 1)]
            ch_skip = block_channels[-(i + 2)]
            self.decoder_ups.append(
                ME.MinkowskiConvolutionTranspose(
                    ch, ch_skip, kernel_size=2, stride=2, dimension=3,
                )
            )
            self.decoder_blocks.append(
                self._make_block(ch_skip * 2, ch_skip)
            )

        # final projection
        final_ch = block_channels[0]
        self.final = ME.MinkowskiConvolution(
            final_ch, out_channels, kernel_size=1, dimension=3,
        )

    def _make_block(self, in_ch: int, out_ch: int) -> nn.Sequential:
        return nn.Sequential(
            ME.MinkowskiConvolution(in_ch, out_ch, kernel_size=3, dimension=3),
            ME.MinkowskiBatchNorm(out_ch),
            ME.MinkowskiReLU(inplace=True),
            ME.MinkowskiConvolution(out_ch, out_ch, kernel_size=3, dimension=3),
            ME.MinkowskiBatchNorm(out_ch),
            ME.MinkowskiReLU(inplace=True),
        )

    def forward(self, x: Any) -> Any:
        # initial conv
        out = self.relu(self.bn0(self.conv0(x)))

        # encoder
        skip_connections = [out]
        for block, pool in zip(self.encoder_blocks, self.encoder_pools):
            out = block(out)
            skip_connections.append(out)
            out = pool(out)

        # decoder
        for up, block in zip(self.decoder_ups, self.decoder_blocks):
            out = up(out)
            skip = skip_connections.pop()
            out = ME.cat(out, skip)
            out = block(out)

        out = self.final(out)
        return out


def pointcloud_to_sparse_tensor(
    points: np.ndarray,
    voxel_size: float = 0.1,
    device: torch.device | str = "cpu",
) -> Any:
    """convert (N, 3+) numpy point cloud to MinkowskiEngine SparseTensor"""
    if not HAS_MINKOWSKI:
        raise RuntimeError("MinkowskiEngine is required.")

    coords = np.floor(points[:, :3] / voxel_size).astype(np.int32)
    # occupancy features
    feats = np.ones((len(coords), 1), dtype=np.float32)

    # remove duplicate voxels
    coords_tensor = torch.from_numpy(coords).int()
    feats_tensor = torch.from_numpy(feats).float()

    sparse_tensor = ME.SparseTensor(
        features=feats_tensor,
        coordinates=ME.utils.batched_coordinates([coords_tensor]),
        device=device,
    )
    return sparse_tensor


class PointNetSimple(nn.Module):
    """lightweight PointNet-style feature extractor (no MinkowskiEngine needed)"""

    def __init__(self, in_channels: int = 3, out_channels: int = 256) -> None:
        super().__init__()
        self.mlp = nn.Sequential(
            nn.Linear(in_channels, 64),
            nn.ReLU(inplace=True),
            nn.Linear(64, 128),
            nn.ReLU(inplace=True),
            nn.Linear(128, 256),
            nn.ReLU(inplace=True),
            nn.Linear(256, out_channels),
        )

    def forward(self, points: torch.Tensor) -> torch.Tensor:
        """return (B, out_channels) global descriptor from (B, N, C) point clouds"""
        point_features = self.mlp(points)  # (B, N, out_channels)
        global_feat = point_features.max(dim=1)[0]  # (B, out_channels)
        return global_feat
