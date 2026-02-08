"""pose graph optimization for SLAM using Open3D"""

from __future__ import annotations

import logging
from dataclasses import dataclass

import numpy as np

logger = logging.getLogger(__name__)

try:
    import open3d as o3d

    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False
    logger.warning("Open3D not installed. Pose graph optimization unavailable.")


@dataclass
class PoseGraphConfig:
    """pose graph optimization config"""

    optimizer: str = "levenberg_marquardt"
    max_iterations: int = 100
    odometry_information_scale: float = 1.0
    loop_information_scale: float = 0.5

    @classmethod
    def from_dict(cls, config: dict) -> PoseGraphConfig:
        return cls(**{k: v for k, v in config.items() if k in cls.__dataclass_fields__})


class SLAMPoseGraph:
    """pose graph with odometry and loop closure edges, optimized via Open3D"""

    def __init__(self, config: PoseGraphConfig | None = None) -> None:
        if not HAS_OPEN3D:
            raise RuntimeError("Open3D is required for pose graph optimization.")

        self.config = config or PoseGraphConfig()
        self._pose_graph = o3d.pipelines.registration.PoseGraph()
        self._num_nodes = 0
        self._loop_closures: list[tuple[int, int, np.ndarray, float]] = []

        logger.info("Pose graph initialized.")

    def add_node(self, pose: np.ndarray) -> int:
        """add pose node, return index"""
        node = o3d.pipelines.registration.PoseGraphNode(pose.copy())
        self._pose_graph.nodes.append(node)
        node_idx = self._num_nodes
        self._num_nodes += 1
        return node_idx

    def add_odometry_edge(
        self,
        source_idx: int,
        target_idx: int,
        transform: np.ndarray,
        information: np.ndarray | None = None,
        uncertain: bool = False,
    ) -> None:
        """add sequential odometry edge"""
        if information is None:
            scale = self.config.odometry_information_scale
            information = np.eye(6, dtype=np.float64) * scale

        edge = o3d.pipelines.registration.PoseGraphEdge(
            source_node_id=source_idx,
            target_node_id=target_idx,
            transformation=transform.copy(),
            information=information,
            uncertain=uncertain,
        )
        self._pose_graph.edges.append(edge)

    def add_loop_closure(
        self,
        source_idx: int,
        target_idx: int,
        transform: np.ndarray,
        confidence: float = 1.0,
        information: np.ndarray | None = None,
    ) -> None:
        """add loop closure edge"""
        if information is None:
            scale = self.config.loop_information_scale * confidence
            information = np.eye(6, dtype=np.float64) * scale

        edge = o3d.pipelines.registration.PoseGraphEdge(
            source_node_id=source_idx,
            target_node_id=target_idx,
            transformation=transform.copy(),
            information=information,
            uncertain=True,
        )
        self._pose_graph.edges.append(edge)
        self._loop_closures.append(
            (source_idx, target_idx, transform.copy(), confidence)
        )

        logger.info(
            "Loop closure added: %d -> %d (confidence=%.3f)",
            source_idx, target_idx, confidence,
        )

    def optimize(self) -> np.ndarray:
        """optimize pose graph, return (N, 4, 4) poses"""
        logger.info(
            "Optimizing pose graph with %d nodes, %d edges (%d loop closures)",
            self._num_nodes,
            len(self._pose_graph.edges),
            len(self._loop_closures),
        )

        option = o3d.pipelines.registration.GlobalOptimizationOption(
            max_correspondence_distance=1.0,
            edge_prune_threshold=0.25,
            reference_node=0,
        )

        if self.config.optimizer == "levenberg_marquardt":
            method = o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt()
        else:
            method = o3d.pipelines.registration.GlobalOptimizationGaussNewton()

        convergence = o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria()
        convergence.max_iteration = self.config.max_iterations

        o3d.pipelines.registration.global_optimization(
            self._pose_graph, method, convergence, option,
        )

        optimized = self.get_poses()
        logger.info("Pose graph optimization complete.")
        return optimized

    def get_poses(self) -> np.ndarray:
        poses = np.array([
            node.pose for node in self._pose_graph.nodes
        ])
        return poses

    def get_num_nodes(self) -> int:
        return self._num_nodes

    def get_num_edges(self) -> int:
        return len(self._pose_graph.edges)

    def get_num_loop_closures(self) -> int:
        return len(self._loop_closures)

    def build_from_odometry(
        self,
        poses: np.ndarray,
        timestamps: np.ndarray | None = None,
    ) -> None:
        """build pose graph from sequential odometry poses"""
        n = len(poses)
        if n == 0:
            return

        # asdd nodes
        for i in range(n):
            self.add_node(poses[i])

        # add sequential edges
        for i in range(n - 1):
            relative = np.linalg.inv(poses[i]) @ poses[i + 1]
            self.add_odometry_edge(i, i + 1, relative)

        logger.info("Built pose graph from %d odometry poses.", n)
