"""合法感知输入的缓冲、解码与消费游标。

本模块只适配 ``SensorBuffer`` 的公开快照，不决定地图楼层、任务状态或安全动作。
同一代数据是否可写图仍由上层在完成时间匹配与楼层守卫后显式提交。
"""
from __future__ import annotations

from dataclasses import dataclass

from danger_search.pose_provider import POSE_EXPIRED, POSE_PENDING
from danger_search.sensor_buffer import SensorBuffer, message_stamp_seconds


INPUT_NONE = "none"
INPUT_INVALID = "invalid"
INPUT_POSE_PENDING = "pose_pending"
INPUT_POSE_EXPIRED = "pose_expired"
INPUT_MATCHED = "matched"


@dataclass(frozen=True)
class CloudInput:
    message: object
    generation: int
    stamp: float | None


@dataclass(frozen=True)
class ImageInput:
    bundle: object
    generation: int
    stamps: tuple[float | None, float | None, float | None]


@dataclass(frozen=True)
class CloudPreparation:
    status: str
    cloud_input: CloudInput | None
    pose_lookup: object | None
    pose_sample: object


@dataclass(frozen=True)
class ImagePreparation:
    status: str
    image_input: ImageInput | None
    pose_lookup: object | None


class PerceptionRuntime:
    """拥有 ROS 感知缓冲和两类数据的消费游标。"""

    def __init__(self, sensor_buffer: SensorBuffer) -> None:
        self.sensor_buffer = sensor_buffer
        self._last_cloud_generation = 0
        self._last_image_generation = 0

    @classmethod
    def from_topics(cls, **topics) -> "PerceptionRuntime":
        return cls(SensorBuffer(**topics))

    def snapshot(self):
        return self.sensor_buffer.snapshot()

    def cloud_input(self, snapshot) -> CloudInput | None:
        current = self.current_cloud(snapshot)
        if current is None or current.generation == self._last_cloud_generation:
            return None
        return current

    def prepare_cloud(
        self,
        snapshot,
        *,
        localization_runtime,
        fallback_pose_sample,
    ) -> CloudPreparation:
        cloud_input = self.cloud_input(snapshot)
        if cloud_input is None:
            return CloudPreparation(
                INPUT_NONE,
                None,
                None,
                fallback_pose_sample,
            )
        if cloud_input.stamp is None:
            return CloudPreparation(
                INPUT_INVALID,
                cloud_input,
                None,
                fallback_pose_sample,
            )
        lookup = localization_runtime.lookup(
            cloud_input.stamp,
            allow_legacy_at=False,
        )
        sample = lookup.sample if lookup.sample is not None else fallback_pose_sample
        if lookup.status == POSE_PENDING:
            status = INPUT_POSE_PENDING
        elif lookup.status == POSE_EXPIRED:
            status = INPUT_POSE_EXPIRED
        else:
            status = INPUT_MATCHED
        return CloudPreparation(status, cloud_input, lookup, sample)

    @staticmethod
    def current_cloud(snapshot) -> CloudInput | None:
        message = snapshot.cloud_msg
        generation = int(
            getattr(snapshot, "cloud_generation", id(message) if message is not None else 0)
            or 0
        )
        if message is None or generation <= 0:
            return None
        return CloudInput(message, generation, message_stamp_seconds(message))

    def consume_cloud(self, generation: int) -> None:
        generation = int(generation)
        if generation <= 0:
            raise ValueError("点云 generation 必须为正整数")
        self._last_cloud_generation = max(self._last_cloud_generation, generation)

    def image_input(self, snapshot) -> ImageInput | None:
        bundle = snapshot.image_bundle
        if bundle is None:
            return None
        generation = int(
            getattr(snapshot, "image_generation", getattr(bundle, "generation", id(bundle)))
            or 0
        )
        if generation <= 0 or generation == self._last_image_generation:
            return None
        stamps = tuple(
            message_stamp_seconds(message)
            for message in (bundle.rgb_msg, bundle.depth_msg, bundle.camera_info_msg)
        )
        return ImageInput(bundle, generation, stamps)

    def prepare_image(
        self,
        snapshot,
        *,
        localization_runtime,
    ) -> ImagePreparation:
        image_input = self.image_input(snapshot)
        if image_input is None:
            return ImagePreparation(INPUT_NONE, None, None)
        stamps = image_input.stamps
        if (
            any(stamp is None for stamp in stamps)
            or max(stamps) - min(stamps) > self.sync_slop
        ):
            return ImagePreparation(INPUT_INVALID, image_input, None)
        lookup = localization_runtime.lookup(float(stamps[0]))
        if lookup.status == POSE_PENDING:
            status = INPUT_POSE_PENDING
        elif lookup.status == POSE_EXPIRED:
            status = INPUT_POSE_EXPIRED
        else:
            status = INPUT_MATCHED
        return ImagePreparation(status, image_input, lookup)

    def consume_image(self, generation: int) -> None:
        generation = int(generation)
        if generation <= 0:
            raise ValueError("图像 generation 必须为正整数")
        self._last_image_generation = max(self._last_image_generation, generation)

    def read_cloud_xyz(self, cloud_input: CloudInput, *, max_points: int, stride: int):
        return self.sensor_buffer.read_cloud_xyz(
            cloud_input.message,
            max_points=max_points,
            stride=stride,
        )

    @property
    def sync_slop(self) -> float:
        return float(getattr(self.sensor_buffer, "sync_slop", 0.08))
