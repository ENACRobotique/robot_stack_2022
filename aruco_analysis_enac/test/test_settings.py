import pytest
import aruco_analysis_enac.settings as settings

aruco_setting = settings.get_markers(1)
fake_corners = [[1,2,3,4],[5,6,7,8],[9,10,11,12],[13,14,15,16]]
fake_ids = [[1],[2],[3],[4]]

def test_regroup_corners_by_size():
    corners_sorted = aruco_setting.regroup_corners_by_size(fake_corners, fake_ids, mvt=settings.Movement.ALL)
    assert corners_sorted == {0.10: [[[1, 2, 3, 4], [1]], [[5, 6, 7, 8], [2]], [[9, 10, 11, 12], [3]], [[13, 14, 15, 16], [4]]]}


def test_get_movement_flag():
    #write function to test settings.get_movement_flag with a frame_count and assert if the flag is correct
    #it should return a full flag when the frame counter is 0
    assert settings.get_movement_flag(0) == settings.Movement.ALL
    assert settings.get_movement_flag(1) == settings.Movement.MOVING