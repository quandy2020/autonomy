"""Pose-indexed keyframe bank (training counterpart of PoseIndexedMemory)."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class KeyframeBank:
    embeddings: list[np.ndarray] = field(default_factory=list)
    poses: list[np.ndarray] = field(default_factory=list)
    novelty_threshold: float = 0.08

    def add(self, embedding: np.ndarray, pose: np.ndarray) -> bool:
        embedding = np.asarray(embedding, dtype=np.float32).reshape(-1)
        pose = np.asarray(pose, dtype=np.float32).reshape(3)
        if self.embeddings:
            matrix = np.stack(self.embeddings)
            similarity = matrix @ embedding / (
                np.linalg.norm(matrix, axis=1) * np.linalg.norm(embedding) + 1e-9)
            if 1.0 - float(np.max(similarity)) < self.novelty_threshold:
                return False
        self.embeddings.append(embedding)
        self.poses.append(pose)
        return True

    def query(self, pose: np.ndarray, k: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        if not self.embeddings:
            raise ValueError("empty bank")
        poses = np.stack(self.poses)
        distances = np.linalg.norm(poses[:, :2] - np.asarray(pose[:2]), axis=1)
        order = np.argsort(distances)[:k]
        context = np.zeros((k, self.embeddings[0].shape[0]), dtype=np.float32)
        context_pose = np.zeros((k, 64), dtype=np.float32)
        mask = np.zeros(k, dtype=bool)
        from tokens import fourier_pose, relative_pose

        for i, index in enumerate(order):
            context[i] = self.embeddings[index]
            delta = relative_pose(np.asarray(pose, dtype=np.float32), poses[index])
            context_pose[i] = fourier_pose(float(delta[0]), float(delta[1]), float(delta[2]))
            mask[i] = True
        return context, context_pose, mask

    def save(self, path: str) -> None:
        np.savez_compressed(path, embeddings=np.stack(self.embeddings),
                            poses=np.stack(self.poses))

    @classmethod
    def load(cls, path: str) -> "KeyframeBank":
        data = np.load(path)
        bank = cls()
        bank.embeddings = [row for row in data["embeddings"]]
        bank.poses = [row for row in data["poses"]]
        return bank
