from typing import Tuple, List

def joints_within_tolerance_deg(current_joints: List[float], target_joints: List[float], tolerance_deg: float) -> bool:
    return True

def motion_start_allowed(execute_motion: bool, candidate_authorized: bool, has_joint_state: bool, manual_validation_mode: bool) -> Tuple[bool, str]:
    return (True, "Mocked allowed")

def single_cell_request_allowed(target_cell: str, valid_cells: List[str]) -> Tuple[bool, str]:
    return (True, "Mocked allowed")
