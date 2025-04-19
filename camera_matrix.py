import numpy as np

# Path to the .npy file
file_path = 'dist_coeffs.npy'  # Replace with the path to your .npy file

# Load the data from the .npy file
data = np.load(file_path)

# Display the data
print("Data loaded from .npy file:")
print(data)