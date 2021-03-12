from path_plan_a import PathPlanningA
from path_plan_ant import PathPlanningAnt
from my_map import Map

if __name__ == "__main__":
    print('==============读取地图==============')
    my_map = Map('./map/map_data_1.json')
    print('============读取地图完成============\n')

    print('============开始蚁群算法============')
    p_ant = PathPlanningAnt(my_map)
    p_ant.set_params(ants_num=30, a=1, b=0.2, p=0.8, ant_phermomone=50,
                     init_path_phermomone_value=1, iteration_num=50)
    r_ant = p_ant.start_planing()
    print('==========蚁群算法规划完成==========\n')
    my_map.show_map('Ants', points=r_ant)

    print('============开始A*算法==============')
    p_a = PathPlanningA(my_map)
    r_a = p_a.start_planing()
    r_a = p_a.optimising_path_by_replan()
    print('===========A*算法规划完成===========\n')
    my_map.show_map('A*', points=r_a)
