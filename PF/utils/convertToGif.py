import os
import imageio
__all__ = ['images_to_gif']
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