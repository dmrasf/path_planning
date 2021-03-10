from path_plan_a import PathPlanningA
from path_plan_ant import PathPlanningAnt
from my_map import Map

if __name__ == "__main__":
    my_map = Map('./map_data_1.json')

    p_ant = PathPlanningAnt(my_map)
    p_ant.start_planing()

    p_a = PathPlanningA(my_map)
    path_a = p_a.start_planing()
    print(my_map.get_visual_points())

    # my_map.show_map()
