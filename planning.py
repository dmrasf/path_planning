from path_plan_a import PathPlanningA
from path_plan_ant import PathPlanningAnt
from my_map import Map

if __name__ == "__main__":
    my_map = Map('./map_data_1.json')

    p_ant = PathPlanningAnt(my_map)
    r_ant = p_ant.start_planing()
    my_map.show_map('Ants', points=r_ant)

    p_a = PathPlanningA(my_map)
    r_a = p_a.start_planing()
    my_map.show_map('A*', points=r_a)
