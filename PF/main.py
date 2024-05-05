from MapClasses import *
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

def test_classes():
    robot_trans_model = lambda pos, command: pos + command
    becaon_trans_model = lambda pos, command: pos
    map = Map()
    map.add_object(Robot(np.array([0,0]), robot_trans_model), "robot")
    map.add_object(Beacon(np.array([1,1]), becaon_trans_model), "becaon")
    print(map.objects['robot'])
    print(map.objects['becaon'])
    command = {"robot": np.array([1,2]), "becaon": np.array([0,0])}
    map.transition(command)
    print(map.objects['robot'])
    print(map.objects['becaon'])
    #map.plot()
    #get meas fron Robot to becaon:
    meas = map.measure_distances("robot")
    print(meas)
    
    #using animation show the robot moving eachtime to the right by 1 for 10 steps:
    # fig, ax = plt.subplots()
    # ax.set_xlim(-10, 10)
    # ax.set_ylim(-10, 10)
    # def animate(i):
    #     command = {"robot": np.array([1,0]), "becaon": np.array([0,0])}
    #     map.transition(command)
    #     ax.clear()
    #     for obj in map.objects.values():
    #         ax.plot(obj.pos[0], obj.pos[1], 'ro')
    # ani = FuncAnimation(fig, animate, frames=10, repeat=False, interval=1000)
    # plt.show()

if __name__ == "__main__":
    test_classes()