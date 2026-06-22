from enum import Enum, auto


class TaskState(Enum):
    """任务状态"""
    IDLE = auto()
    DETECTING = auto()
    MOVING_TO_OBJECT = auto()
    GRASPING = auto()
    LIFTING = auto()
    MOVING_TO_TARGET = auto()
    RELEASING = auto()
    RETURNING = auto()
    ERROR = auto()
