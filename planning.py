from path_plan_a import PathPlanningA
from path_plan_ant import PathPlanningAnt
from my_map import Map

if __name__ == "__main__":
    print('==============读取地图==============')
    my_map = Map('./map/map_data_2.json', 'json')
    print('============读取地图完成============\n')

    print('============开始蚁群算法============')
    plan_ant = PathPlanningAnt(my_map)
    plan_ant.set_params(ants_num=40, a=1, b=0.15, p=0.8, ant_phermomone=50,
                        init_path_phermomone_value=1, iteration_num=30)
    points_ant = plan_ant.start_planing(is_optimising=False)
    path_ant = my_map.calculate_path_distance(points_ant)
    print('路径总长度', path_ant)
    my_map.show_map('Ants', points=points_ant, is_show_all_points=True)
    print('==========蚁群算法规划完成==========\n')

    print('============开始A*算法==============')
    plan_a = PathPlanningA(my_map)
    points_a = plan_a.start_planing(is_optimising=True)
    path_a = my_map.calculate_path_distance(points_a)
    print('路径总长度', path_a)
    my_map.show_map('A*', points=points_a, is_show_all_points=True)
    print('===========A*算法规划完成===========\n')
