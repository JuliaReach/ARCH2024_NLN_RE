""" check_collision

Extracting reachable sets (whole car occupancy) from csv files and checking
for collision with the corresponding scenario. Also includes visualization options.
"""

# General imports
import os
import gc
import argparse
import matplotlib.pyplot as plt
import numpy as np
from csv import reader
import glob
try:
    from celluloid import Camera
    CELLULOID_AVAILABILITY = True
except:
    CELLULOID_AVAILABILITY = False

# commonroad specific imports
# from commonroad_dc.collision.visualization.draw_dispatch import draw_object
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad_dc.collision.collision_detection.pycrcc_collision_dispatch \
        import create_collision_checker, create_collision_object
from commonroad_dc.boundary import boundary
from commonroad.prediction.prediction import SetBasedPrediction, Occupancy
from commonroad.geometry.shape import Polygon, ShapeGroup
from commonroad.common.file_reader import CommonRoadFileReader


__author__ = "Philipp Gassert, Niklas Kochdumper"
__copyright__ = "TUM Cyber-Physical Systems Group"
__version__ = "2021.0"
__maintainer__ = "Niklas Kochdumper"
__email__ = "niklas.kochdumper@tum.de"
__status__ = "Released"


def load_scenario(scenario_path, verbose=False):
    """Load scenario and return CommonRoad scenario object and step_size.

    :param scenario_path: path to scenario to be loaded
    :param verbose: boolean flag determining verbosity
    :return: commonroad scenario object and step_size of that scenario
    """

    if '.xml' not in scenario_path:
        scenario_path = scenario_path + '.xml'
    
    scenario_name = scenario_path.split('/')[-1].split('.')[-2]

    if verbose:
        print('Loading scenario ' + scenario_name)

    # load the exemplary CommonRoad scenario using the CommonRoad file reader
    scenario, _ = CommonRoadFileReader(scenario_path).open()
    step_size = float(scenario.dt)

    return scenario, step_size

def read_occupancy_csv(path_occupancy_csv, step_size=None, verbose=False):
    """ Create occupancy based prediction objects from csv for later collision checking.
    
    :param path_occupancy_csv: path to solution to be loaded
    :param step_size: underlying timespan of a timestep in the respective scenario
    :param verbose: boolean flag determining verbosity
    :return: returns commonroad OccupancyPrediction objects containing only coinciding occupancies
             and cumulative occupancies (all occupancies before each time step)
    """

    if '.csv' not in path_occupancy_csv:
        path_occupancy_csv = path_occupancy_csv + '.csv'

    if step_size is None:
        path, file = os.path.split(os.getcwd())
        scenario_path = os.path.join(path,'data','scenarios', path_occupancy_csv.split('/')[-2] + '.xml')
        _, step_size = load_scenario(scenario_path, verbose=verbose)
    
    occupancies_cumulative = list()
    occupancies_coinciding = list()
    shape_list = list()
    scenario_time_step = 0

    # open file containing vertices of occupancies
    with open(path_occupancy_csv, 'r') as read_obj:
        csv_reader = reader(read_obj, delimiter=',')
        flag = 1
        for row in csv_reader:
            if flag == 1:
                time_buf = [-1,-1]
                time_buf[0] = float(row[0])
                points_buffer = row[1:]
            elif flag == -1:
                time_buf[1] = float(row[0])
                vertices = np.array([points_buffer, row[1:]], dtype=float).transpose()
                step_size_prediction = np.around(time_buf[1] - time_buf[0], 5)

                assert (step_size >= step_size_prediction),\
                    'Step size of solution must currently be smaller or equal to global step size.'
                assert step_size_prediction >= 0, 'invalid time interval'
                
                current_polygon = Polygon(vertices)           
                
                if (scenario_time_step == 0) and (time_buf[0] == 0):
                    occupancies_cumulative.append(
                            Occupancy(int(scenario_time_step/step_size), ShapeGroup(shape_list)))
                
                if time_buf[1] < scenario_time_step:
                    shape_list.append(current_polygon)
                    
                elif time_buf[1] == scenario_time_step:
                    shape_list.append(current_polygon)
                    occupancies_cumulative.append(
                        Occupancy(int(scenario_time_step/step_size), ShapeGroup(shape_list)))
                    shape_list = []
                    
                elif time_buf[0] <= scenario_time_step:
                    shape_list.append(current_polygon)
                    occupancies_cumulative.append(
                        Occupancy(int(scenario_time_step/step_size), ShapeGroup(shape_list)))
                    shape_list = []
                    shape_list.append(current_polygon)
                    
                else:
                    raise Exception('Unexpected behaviour: Time intervals appear not to be in chronological order.')
                
                if (time_buf[1] >= scenario_time_step) and (time_buf[0] <= scenario_time_step):
                    occupancies_coinciding.append(Occupancy(int(scenario_time_step/step_size), current_polygon))
                    scenario_time_step = scenario_time_step + step_size
                
            flag *= -1
            
    occupancy_coinciding_prediction = SetBasedPrediction(0, occupancies_coinciding)
    occupancy_cumulative_prediction = SetBasedPrediction(0, occupancies_cumulative)

    return occupancy_coinciding_prediction, occupancy_cumulative_prediction

def make_animation(scenario, cc, co, road_boundary_sg_triangles, output_path=None, verbose=False):
    """ Make 'video' saved as a gif from solution to scenario given.
    
    :param scenario: commonroad scenario object from underlying scenario
    :param cc: commonroad collision_checker object containing scenario objects
    :param co: commonoad collision_object object containing the prediction occupancies
    :param road_boundary_sg_triangles: commonroad road_boundary_obstacle containing the road boundary
    :param output_path: path to store the output gif to
    :param verbose: boolean flag determining verbosity
    """

    if output_path is None:
        output_path = str(scenario.scenario_id) + '.gif'
    elif '.gif' not in output_path:
        output_path = output_path + '.gif'
    
    fig = plt.figure(figsize=(20,20))
    camera = Camera(fig)

    for i in np.arange(co.time_start_idx(), co.time_end_idx()+1):
        #print(i)
        if True:
            show_progress_movie(i, co.time_end_idx()+1)
        rnd = MPRenderer()
        #draw_object(scenario.lanelet_network)
        scenario.lanelet_network.draw(rnd)
        #draw_object(road_boundary_sg_triangles)
        road_boundary_sg_triangles.draw(rnd)
        if cc.time_slice(i).collide(co.obstacle_at_time(i)):
            # draw_object(co.obstacle_at_time(i), draw_params={'collision': {'facecolor': 'red'}})
            rnd.draw_params.shape.facecolor = "red"
            co.obstacle_at_time(i).draw(rnd)
        elif road_boundary_sg_triangles.collide(co.obstacle_at_time(i)):
            #draw_object(co.obstacle_at_time(i), draw_params={'collision': {'facecolor': 'yellow'}})
            rnd.draw_params.shape.facecolor = "yellow"
            co.obstacle_at_time(i).draw(rnd)
        else:
            #draw_object(co.obstacle_at_time(i), draw_params={'collision': {'facecolor': 'green'}})
            rnd.draw_params.shape.facecolor = "green"
            co.obstacle_at_time(i).draw(rnd)
        #draw_object(cc.time_slice(i), draw_params={'collision': {'facecolor': 'blue'}})
        rnd.draw_params.shape.facecolor = "blue"
        cc.time_slice(i).draw(rnd)
        rnd.render()
        plt.autoscale()
        plt.axis('equal')
        camera.snap()

    if True:   
        print('                                                               ', end='\r')

    animation = camera.animate()
    animation.save(output_path, fps=2)
    if verbose:
        print('Animation saved under ' + output_path)

    plt.close(fig)

def show_progress_movie(current, total, barLength = 20):
    """ Show progress writing movie gif.
    
    :param current: current frame
    :param total: total number of frames
    :param barLength: length of progress bar
    """

    percent = float(current) * 100 / total
    arrow   = '-' * int(percent/100 * barLength)
    spaces  = ' ' * (barLength - len(arrow))

    print('Progress writing gif animation: [%s%s] %d %%' % (arrow, spaces, percent), end='\r')

def plot_scene(scenario, cc, co, road_boundary_sg_triangles):
    """ Plot scenario and solution in single figure.
    
    :param scenario: commonroad scenario object from underlying scenario
    :param cc: commonroad collision_checker object containing scenario objects
    :param co: commonoad collision_object object containing the prediction occupancies
    :param road_boundary_sg_triangles: commonroad road_boundary_obstacle containing the road boundary
    """

    plt.figure(figsize=(25, 10))
    rnd = MPRenderer()
    #draw_object(scenario.lanelet_network)
    scenario.lanelet_network.draw(rnd)
    #draw_object(road_boundary_sg_triangles)
    road_boundary_sg_triangles.draw(rnd)
    for i in np.arange(co.time_start_idx(), co.time_end_idx()+1):
        if cc.time_slice(i).collide(co.obstacle_at_time(i)):
            #draw_object(co.obstacle_at_time(i), draw_params={'collision': {'facecolor': 'red','edgecolor': 'none'}})
            rnd.draw_params.shape.facecolor = "red"
            rnd.draw_params.shape.edgecolor = "none"
            co.obstacle_at_time(i).draw(rnd)
        elif road_boundary_sg_triangles.collide(co.obstacle_at_time(i)):
            #draw_object(co.obstacle_at_time(i), draw_params={'collision': {'facecolor': 'yellow','edgecolor': 'none'}})
            rnd.draw_params.shape.facecolor = "yellow"
            rnd.draw_params.shape.edgecolor = "none"
            co.obstacle_at_time(i).draw(rnd)
        else:
            #draw_object(co.obstacle_at_time(i), draw_params={'collision': {'facecolor': 'green','edgecolor': 'none'}})
            rnd.draw_params.shape.facecolor = "green"
            rnd.draw_params.shape.edgecolor = "none"
            co.obstacle_at_time(i).draw(rnd)
    #draw_object(cc, draw_params={'collision': {'facecolor': 'blue'}})
    rnd.draw_params.shape.facecolor = "blue"
    cc.draw(rnd)
    rnd.render()
    plt.autoscale()
    plt.axis('equal')
    plt.show()

def occupancy_collision_checker(path_occupancy_csv, verbose=False, animation=False, plot=False):
    """ Check for collision between scenario and prediction.
    
    :param path_occupancy_csv: path to solution to be loaded
    :param verbose: flag determining verbosity
    :param animation: if flag is set, gif 'animation' is created and saved to path argument if given
    :param plot: if flag is set, scenario and solution are plotted in single frame
    :return: returns booleans c_obstalces (true if colliding with an obstacle) and c_boundary
             (true if colliding with road boundary)
    """

    path, file = os.path.split(os.getcwd())
    scenario_path = '/code/BEL_Putte-4_2_T-1.xml'
    # os.path.join(path, 'code', path_occupancy_csv.split('/')[-1].replace('_occupancies.csv', '.xml'))
    
    scenario, step_size = load_scenario(scenario_path, verbose=verbose)
    occupancy_coinciding_prediction, occupancy_cumulative_prediction = read_occupancy_csv(
            path_occupancy_csv, step_size=step_size)
        
    # create collision checker using the scenario
    cc = create_collision_checker(scenario)
    # create a collision objects using the trajectory prediction of the ego vehicle
    co = create_collision_object(occupancy_coinciding_prediction)
    # create road boundary obstacle
    road_boundary_obstacle, road_boundary_sg_triangles = boundary.create_road_boundary_obstacle(
            scenario, method='triangulation')

    # test the trajectory of the ego vehicle for collisions with obstacles or the road boundary
    c_boundary = road_boundary_sg_triangles.collide(co)
    c_obstacles = cc.collide(co)

    if verbose:

        if c_obstacles or c_boundary:
            print('A collision occured')
        else:
            print('No collision detected')
    
        print(' ')

    if animation:
        animationDir_path = os.path.join(path,'results')
        if not os.path.isdir(animationDir_path):
            os.mkdir(animationDir_path)
        name = path_occupancy_csv.split('/')[-1].replace('occupancies.csv', 'animation.gif')
        animation_path = os.path.join(animationDir_path,name)
        make_animation(scenario, cc, co, road_boundary_sg_triangles, verbose=verbose, output_path=animation_path)

    if plot:
        plot_scene(scenario, cc, co, road_boundary_sg_triangles)

    return c_obstacles, c_boundary
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
            prog='Commonroad CollisionChecker',
            description='Check reachable set solution (whole car occupancy) from csv file for collisions.')

    parser.add_argument(
            'path_occupancy_csv', nargs='?',
            default=None,
            type=str, help='Path to csv file containing solution occupancy sets.')

    parser.add_argument(
            '-s', '--suppress', nargs='?', const=False, default=True, dest='output',
            help='Suppress output (plot and animation).')
    
    args = parser.parse_args()

    path, file = os.path.split(os.getcwd())

    if args.path_occupancy_csv is None:
        solution_list = sorted(glob.glob(os.path.join(path,'results','*_occupancies.csv')))
        num_solutions = len(solution_list)
        counter = 0
        collisions = 0
        print('Collision Checker ------------------------------------------------')

        for solution in solution_list:

            c_o, c_b = occupancy_collision_checker(solution)
            gc.collect()
            counter += 1

            if c_o or c_b:
                collisions += 1

                if collisions == 1:
                    print(' ')
                    print('Collisions occured in:')

                temp = solution.split('solutions')[-1]
                temp = temp.replace('_occupancies.csv','')
                print('    ' + temp.replace('/',''))

        assert counter == num_solutions     
        print(' ')           
        print('Number of collisions: {} of {} scenarios     '.format(collisions, num_solutions))

    else:
        if not CELLULOID_AVAILABILITY:
            print('Celluloid package not available; animation cannot be created and option is deactivated.')
            args.animation_path = False

        path_occupancy_csv = args.path_occupancy_csv

        if not '.csv' in path_occupancy_csv:
            path_occupancy_csv = os.path.join(path,'results', path_occupancy_csv + '_occupancies.csv')

        occupancy_collision_checker(path_occupancy_csv, animation=args.output, plot=args.output, verbose=True)