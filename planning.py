from path_plan_a import PathPlanningA
from path_plan_ant import PathPlanningAnt
from my_map import Map

if __name__ == "__main__":
    my_map = Map('./map/map_data_1.json')

    p_ant = PathPlanningAnt(my_map)
    p_ant.set_params(ants_num=10, a=1, b=0.3, p=0.8, ant_phermomone=50,
                     init_path_phermomone_value=1, iteration_num=10)
    r_ant = p_ant.start_planing()
    my_map.show_map('Ants', points=r_ant)

    p_a = PathPlanningA(my_map)
    r_a = p_a.start_planing()
    my_map.show_map('A*', points=r_a)
