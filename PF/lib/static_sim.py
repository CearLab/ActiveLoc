import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from ParticleFilter import single_step_particle_filter
import pandas as pd

__all__ = ['particle_filter_simulation', 'plot_and_save_stats', 'load_stats']

def particle_filter_simulation(n_steps, n_particles, beacons, x, u_agent, z, agent_transition_model, measurement_likelihood, resample_method, exp_folder_name):
    dict_stats = {'mean': list(np.zeros((n_steps, 2))), 'cov': list(np.zeros((n_steps, 4)))}
    particles = (np.random.rand(n_particles, 2)-0.5)*4
    fig, ax = plt.subplots()
    sc_path, = ax.plot([], [], c='g',marker='o')
    sc = ax.scatter([], [], color='red')
    sc_beacons = ax.scatter(beacons[:, 0], beacons[:, 1], c='b')
    #set the lim for the maximum position of the agent and the beacons
    x_max = np.max(np.abs(x))
    beacons_max = np.max(np.abs(beacons))
    lim = x_max if x_max > beacons_max else beacons_max
    lim += 1
    def init():
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        sc_path.set_data([], [])

    def animate(i):
        nonlocal particles
        sc_path.set_data(x[:i, 0],x[:i, 1])
        sc.set_offsets(particles)
        # set the title of the plot to the current step
        ax.set_title(f"Step {i}")
        print(f"\r{i}/{n_steps}", end='', flush=True)
        particles = single_step_particle_filter(particles, u_agent[i], z[i], agent_transition_model, measurement_likelihood,resample_method)
        dict_stats['mean'][i] = np.mean(particles, axis=0)
        dict_stats['cov'][i] = np.cov(particles, rowvar=False).flatten()
        return sc, sc_path

    ani = animation.FuncAnimation(fig, animate, frames=n_steps, init_func=init, interval=500)
    ani.save(f'{exp_folder_name}/movie.gif', writer='imagemagick')

    return dict_stats

def plot_and_save_stats(x, beacons, dict_stats, exp_folder_name,stats_fig=None):
    #plot ground truth and estimated path:
    # use the inputed plot object if it is provided
    print(exp_folder_name)
    plt.figure()
    plt.plot(x[:, 0], x[:, 1], 'bo-')
    plt.plot(beacons[:, 0], beacons[:, 1], 'ro')
    plt.plot(np.array(dict_stats['mean'])[:, 0], np.array(dict_stats['mean'])[:, 1], 'g-')
    plt.savefig(f'{exp_folder_name}/path.png')
    plt.show()

    #plot std norm
    fig =plt.figure()
    ax = fig.add_subplot(211)
    ## calculate the covariance matrix determinant
    matrix_cov = np.array(dict_stats['cov']).reshape(-1, 2, 2)
    cov_det = np.linalg.det(matrix_cov)
    eigen_values, eigen_vectors = np.linalg.eig(matrix_cov)
    # convert negative values to zero
    cov_det[cov_det < 0] = 0
    std_norm = np.sqrt(cov_det)
    std_norm = np.sqrt(eigen_values[:,0] + eigen_values[:,1])
    ax.plot(std_norm)
    ax.yaxis.label.set_text('sqrt cov det')

    # plot positon error
    ax = fig.add_subplot(212)
    ax.plot(np.linalg.norm(x - np.array(dict_stats['mean']), axis=1))
    ax.yaxis.label.set_text('point error')
    plt.savefig(f'{exp_folder_name}/stats.png')
    plt.show()

    #save mean
    np.savetxt(f'{exp_folder_name}/stats_mean.csv', np.array(dict_stats['mean']), delimiter=',',header='x,y')

    #save std
    np.savetxt(f'{exp_folder_name}/stats_cov.csv', np.array(dict_stats['cov']), delimiter=',',header='xx,xy,yx,yy')
    
def load_stats(exp_folder_name):
    #load stats to pandas data frame
    stats_mean = pd.read_csv(f'{exp_folder_name}/stats_mean.csv')
    stats_cov = pd.read_csv(f'{exp_folder_name}/stats_cov.csv')
    # set header to pandas
    stats_mean.columns = ['x', 'y']
    stats_cov.columns = ['xx', 'xy', 'yx', 'yy']
    return stats_mean, stats_cov