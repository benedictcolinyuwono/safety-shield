from dataclasses import dataclass
from typing import List, Set


@dataclass
class AGV:
    id: int
    pos: int
    goal: int


def get_occupied_cells(agvs: List[AGV]) -> Set[int]:
    return {agv.pos for agv in agvs}


def step_with_reservations(agvs: List[AGV], reservations: Set[int]) -> List[AGV]:
    updated_agvs = []
    for agv in agvs:
        next_cell = agv.pos + 1
        if next_cell in reservations:
            agv = AGV(agv.id, next_cell, agv.goal)
        updated_agvs.append(agv)
    return updated_agvs