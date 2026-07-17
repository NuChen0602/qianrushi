from robot_lidar_bridge.frontier_explorer import frontier_clusters


def test_frontier_clusters_detects_free_unknown_boundary():
    width = height = 7
    data = [100] * (width * height)
    for y in range(1, 6):
        for x in range(1, 6):
            data[y * width + x] = -1
    for y in range(2, 5):
        for x in range(2, 5):
            data[y * width + x] = 0
    groups = frontier_clusters(data, width, height, min_cells=4)
    assert len(groups) == 1
    assert len(groups[0]) == 8
