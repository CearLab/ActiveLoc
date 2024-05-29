import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Ellipse
import os
import imageio
__all__ = ['draw_ellipse', 'images_to_gif']
def draw_ellipse(ax, n_std=3.0, edgecolor = 'b', facecolor='none', data = None , mean = None, cov = None,  **kwargs):
    """
    Draw an ellipse with a given mean and covariance
    """
    
    if data is not None:
        mean = np.mean(data, axis = 0)
        cov = np.cov(data.T)
    elif mean is None or cov is None:
        raise ValueError("Either data or mean and cov should be provided")
    # Compute eigenvalues and associated eigenvectors
    vals, vecs = np.linalg.eigh(cov)

    # Compute "tilt" of ellipse using first eigenvector
    x, y = vecs[:, 0]
    theta = np.degrees(np.arctan2(y, x))

    # Eigenvalues give length of ellipse along each eigenvector
    width, height = 2 * n_std * np.sqrt(vals)

    # Draw the Ellipse
    ellipse = Ellipse(xy=mean, width=width, height=height, angle=theta, facecolor=facecolor, edgecolor=edgecolor, **kwargs)

    ax.add_patch(ellipse)
    ## add point at the mean
    ax.plot(mean[0], mean[1], c = edgecolor, marker = 'x', markersize = 3)
    ax.set_aspect('equal')
    
    
def images_to_gif(folder_path, output_path):
    # Get all file names in the folder
    filenames = os.listdir(folder_path)

    # Filter out non-image files
    image_filenames = [file for file in filenames if file.endswith(('.png', '.jpg', '.jpeg'))]

    # Sort the image filenames
    image_filenames.sort(key=lambda filename: int(filename.split('_')[1].split('.')[0]))

    # Create a list to hold the images
    images = []

    # Read each image file and append it to the images list
    for filename in image_filenames:
        image_path = os.path.join(folder_path, filename)
        images.append(imageio.imread(image_path))

    # Write the images to a GIF file
    imageio.mimsave(output_path, images, format='GIF', fps=5)
    print(f'gif saved to {output_path}')