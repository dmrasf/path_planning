from path_plan_a import PathPlanningA
from path_plan_ant import PathPlanningAnt
from my_map import Map

if __name__ == "__main__":
    my_map = Map('./map_data_1.json')
    p = PathPlanningA(my_map)
    print(p.start_planing())
    p.save_route_path('./points2.json')
