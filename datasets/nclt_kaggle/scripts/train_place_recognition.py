#!/usr/bin/env python3
"""Train place recognition model on NCLT dataset.

Supports MinkLoc3D and PointNet baselines with triplet loss.

Usage:
    python scripts/train_place_recognition.py \\
        --dataset-config configs/dataset_config.yaml \\
        --train-config configs/train_config.yaml
"""

from __future__ import annotations

import argparse
# TODO ask supervisor whether recall@1 or recall@5 is more useful for SLAM integration
import logging
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
logger = logging.getLogger(__name__)

PROJECT_ROOT = Path(__file__).resolve().parent.parent


def train_epoch(
    model: nn.Module,
    dataloader: DataLoader,
    optimizer: torch.optim.Optimizer,
    loss_fn: nn.Module,
    device: torch.device,
    epoch: int,
) -> float:
    """run one training epoch, return average loss"""
    model.train()
    total_loss = 0.0
    num_batches = 0

    for batch_idx, batch in enumerate(dataloader):
        anchor_pts = batch["anchor"]
        positive_pts = batch["positive"]
        negative_pts = batch["negatives"]

        # move to device
        if isinstance(anchor_pts, torch.Tensor):
            anchor_pts = anchor_pts.to(device)
            positive_pts = positive_pts.to(device)
            negative_pts = negative_pts.to(device)

        # forward
        anchor_desc = model(anchor_pts)
        positive_desc = model(positive_pts)
        negative_desc = model(negative_pts)

        loss = loss_fn(anchor_desc, positive_desc, negative_desc)

        # backward
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        total_loss += loss.item()
        num_batches += 1

        if batch_idx % 10 == 0:
            logger.info(
                "Epoch %d [%d/%d] Loss: %.4f",
                epoch, batch_idx, len(dataloader), loss.item(),
            )

    avg_loss = total_loss / max(num_batches, 1)
    return avg_loss


@torch.no_grad()
def validate(
    model: nn.Module,
    dataloader: DataLoader,
    device: torch.device,
    k_values: list[int] = [1, 5, 10],
) -> dict[str, float]:
    """run validation and compute recall@K"""
    model.eval()

    all_descriptors = []
    all_positions = []

    for batch in dataloader:
        points = batch["anchor"]
        positions = batch["anchor_pose"][:, :3, 3]  # Extract translation

        if isinstance(points, torch.Tensor):
            points = points.to(device)

        desc = model(points)
        all_descriptors.append(desc.cpu().numpy())
        all_positions.append(positions.numpy())

    if not all_descriptors:
        return {f"recall@{k}": 0.0 for k in k_values}

    descriptors = np.concatenate(all_descriptors, axis=0)
    positions = np.concatenate(all_positions, axis=0)

    from src.evaluation.metrics import recall_at_k

    # split into query and database (first half / second half)
    n = len(descriptors)
    mid = n // 2
    results = recall_at_k(
        descriptors[:mid], descriptors[mid:],
        positions[:mid], positions[mid:],
        k_values=k_values,
    )

    return results


def main() -> None:
    parser = argparse.ArgumentParser(description="Train place recognition model")
    parser.add_argument("--dataset-config", type=Path,
                        default=PROJECT_ROOT / "configs" / "dataset_config.yaml")
    parser.add_argument("--train-config", type=Path,
                        default=PROJECT_ROOT / "configs" / "train_config.yaml")
    parser.add_argument("--model", type=str, default=None,
                        help="Override model type (minkloc3d or pointnet)")
    parser.add_argument("--device", type=str, default=None,
                        help="Compute device (cuda/cpu)")
    parser.add_argument("--resume", type=Path, default=None,
                        help="Resume from checkpoint")
    args = parser.parse_args()

    from src.utils.io_utils import load_config

    # load configs
    dataset_config = load_config(args.dataset_config)
    train_config = load_config(args.train_config)
    training = train_config["training"]

    # device
    if args.device:
        device = torch.device(args.device)
    elif torch.cuda.is_available():
        device = torch.device("cuda")
    else:
        device = torch.device("cpu")
    logger.info("Using device: %s", device)

    # model
    model_type = args.model or training.get("model", "minkloc3d")
    feature_dim = training.get("feature_dim", 256)

    from src.models.place_recognition import PlaceRecognitionWrapper, TripletLoss

    model = PlaceRecognitionWrapper(
        model_type=model_type, feature_dim=feature_dim,
    ).to(device)
    logger.info("Model: %s (feature_dim=%d)", model_type, feature_dim)

    # loss
    loss_fn = TripletLoss(
        margin=training.get("margin", 0.2),
        mining=training.get("mining", "hard"),
    )

    # optimizer
    lr = training.get("learning_rate", 1e-3)
    wd = training.get("weight_decay", 1e-4)
    optimizer = torch.optim.Adam(model.parameters(), lr=lr, weight_decay=wd)

    # scheduler
    epochs = training.get("epochs", 80)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=epochs)

    # dataloaders
    from src.datasets.nclt_pairs import NCLTPairsDataset, pairs_collate_fn
    from src.datasets.transforms import build_transforms

    aug_config = {
        "point_cloud": dataset_config["nclt"]["point_cloud"],
        "augmentation": training.get("augmentation", {}),
    }

    train_transform = build_transforms(aug_config, is_train=True)
    val_transform = build_transforms(aug_config, is_train=False)

    train_dataset = NCLTPairsDataset(
        config_path=args.dataset_config, split="train", transform=train_transform,
    )
    val_dataset = NCLTPairsDataset(
        config_path=args.dataset_config, split="val", transform=val_transform,
    )

    train_loader = DataLoader(
        train_dataset,
        batch_size=training.get("batch_size", 32),
        shuffle=True,
        num_workers=training.get("num_workers", 4),
        collate_fn=pairs_collate_fn,
        pin_memory=training.get("pin_memory", True),
    )
    val_loader = DataLoader(
        val_dataset,
        batch_size=training.get("batch_size", 32),
        shuffle=False,
        num_workers=training.get("num_workers", 4),
        collate_fn=pairs_collate_fn,
    )

    # resume
    start_epoch = 0
    best_recall = 0.0
    if args.resume and args.resume.exists():
        checkpoint = torch.load(args.resume, map_location=device, weights_only=True)
        model.load_state_dict(checkpoint["model_state_dict"])
        optimizer.load_state_dict(checkpoint["optimizer_state_dict"])
        start_epoch = checkpoint.get("epoch", 0) + 1
        best_recall = checkpoint.get("best_recall", 0.0)
        logger.info("Resumed from epoch %d (best recall: %.4f)", start_epoch, best_recall)

    # checkpoint dir
    ckpt_dir = Path(training.get("checkpoint_dir", "./checkpoints"))
    ckpt_dir.mkdir(parents=True, exist_ok=True)

    # training loop
    logger.info("Starting training for %d epochs...", epochs)

    for epoch in range(start_epoch, epochs):
        t0 = time.time()
        train_loss = train_epoch(model, train_loader, optimizer, loss_fn, device, epoch)
        scheduler.step()

        elapsed = time.time() - t0
        logger.info(
            "Epoch %d: loss=%.4f, lr=%.2e, time=%.1fs",
            epoch, train_loss, scheduler.get_last_lr()[0], elapsed,
        )

        # validation
        val_every = training.get("val_every", 5)
        if (epoch + 1) % val_every == 0:
            recall = validate(model, val_loader, device)
            logger.info("Validation: %s", recall)

            recall_1 = recall.get("recall@1", 0.0)
            if recall_1 > best_recall:
                best_recall = recall_1
                torch.save({
                    "epoch": epoch,
                    "model_state_dict": model.state_dict(),
                    "optimizer_state_dict": optimizer.state_dict(),
                    "best_recall": best_recall,
                }, ckpt_dir / "best_model.pth")
                logger.info("New best model saved (recall@1=%.4f)", best_recall)

        # periodic checkpoint
        save_every = training.get("save_every", 10)
        if (epoch + 1) % save_every == 0:
            torch.save({
                "epoch": epoch,
                "model_state_dict": model.state_dict(),
                "optimizer_state_dict": optimizer.state_dict(),
                "best_recall": best_recall,
            }, ckpt_dir / f"checkpoint_epoch_{epoch}.pth")

    logger.info("Training complete. Best recall@1: %.4f", best_recall)


if __name__ == "__main__":
    main()
