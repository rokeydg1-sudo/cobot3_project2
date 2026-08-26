from collections import deque
from dataclasses import dataclass

from fms.NodeMapGraph import NodeData, PlannedNodeRoute


@dataclass(frozen=True)
class TaskLocation:
    node_id: int
    name: str
    x: float
    y: float
    z: float

    @classmethod
    def from_node(cls, node: NodeData) -> "TaskLocation":
        return cls(node.node_id, node.name, node.x, node.y, node.z)


@dataclass(frozen=True)
class TaskRoute:
    node_map_revision: int
    node_ids: tuple[int, ...]
    points: tuple[tuple[float, float, float], ...]
    total_cost: float

    @classmethod
    def from_planned_route(
        cls,
        revision: int,
        route: PlannedNodeRoute,
    ) -> "TaskRoute":
        return cls(revision, route.node_ids, route.points, route.total_cost)


@dataclass
class Task:
    task_id: str
    start: TaskLocation
    goal: TaskLocation
    kit_id: str = ""
    processing_time: float = 0.0
    status: str = "WAITING"
    route: TaskRoute | None = None


@dataclass(frozen=True)
class AMRState:
    amr_id: str
    state: str
    x: float
    y: float
    yaw: float
    load_state: str
    current_task_id: str = ""


@dataclass(frozen=True)
class OptimizationRequest:
    tasks: tuple[Task, ...]
    amr_state: AMRState


@dataclass(frozen=True)
class OrderedTask:
    sequence: int
    task_id: str
    route: PlannedNodeRoute


@dataclass(frozen=True)
class OptimizationResult:
    ordered_tasks: tuple[OrderedTask, ...]
    recovery_node_id: int
    total_cost: float


class TaskManager:
    def __init__(self, queue_capacity: int = 10) -> None:
        self.queue_capacity = queue_capacity
        self.waiting_tasks: deque[Task] = deque()
        self.active_tasks: dict[str, Task] = {}
        self._next_task_number = 1

    def create_task(
        self,
        start_node: NodeData,
        goal_node: NodeData,
        task_id: str | None = None,
        kit_id: str = "",
        processing_time: float = 0.0,
    ) -> Task:
        if start_node.node_id == goal_node.node_id:
            raise ValueError("Task start and goal Nodes must be different.")
        if not start_node.available or not goal_node.available:
            raise ValueError("Task start and goal Nodes must be available.")
        if processing_time < 0:
            raise ValueError("Task processing_time must be non-negative.")

        task = Task(
            task_id=task_id or self._create_task_id(),
            start=TaskLocation.from_node(start_node),
            goal=TaskLocation.from_node(goal_node),
            kit_id=kit_id,
            processing_time=float(processing_time),
        )
        self.add_task(task)
        return task

    def add_task(self, task: Task) -> None:
        if len(self.waiting_tasks) >= self.queue_capacity:
            raise OverflowError(f"Task queue is full ({self.queue_capacity}).")
        if self.is_waiting(task.task_id) or self.is_active(task.task_id):
            raise ValueError(f"Duplicate task_id: {task.task_id}")
        self.waiting_tasks.append(task)

    def get_waiting_tasks(self) -> tuple[Task, ...]:
        return tuple(self.waiting_tasks)

    def assign_task(
        self,
        task_id: str,
        route: PlannedNodeRoute,
        revision: int,
    ) -> Task:
        task = next(
            (item for item in self.waiting_tasks if item.task_id == task_id),
            None,
        )
        if task is None:
            raise ValueError(f"Waiting Task not found: {task_id}")
        if (
            route.start_node_id != task.start.node_id
            or route.goal_node_id != task.goal.node_id
        ):
            raise ValueError("Task Nodes and route endpoints do not match.")

        task.route = TaskRoute.from_planned_route(revision, route)
        task.status = "ASSIGNED"
        self.waiting_tasks.remove(task)
        self.active_tasks[task.task_id] = task
        return task

    def complete_task(self, task_id: str) -> Task | None:
        task = self.active_tasks.pop(task_id, None)
        if task is not None:
            task.status = "COMPLETED"
        return task

    def is_waiting(self, task_id: str) -> bool:
        return any(task.task_id == task_id for task in self.waiting_tasks)

    def is_active(self, task_id: str) -> bool:
        return task_id in self.active_tasks

    def _create_task_id(self) -> str:
        while True:
            task_id = f"task_{self._next_task_number:03d}"
            self._next_task_number += 1
            if not self.is_waiting(task_id) and not self.is_active(task_id):
                return task_id

    @property
    def waiting_count(self) -> int:
        return len(self.waiting_tasks)

    @property
    def active_count(self) -> int:
        return len(self.active_tasks)
