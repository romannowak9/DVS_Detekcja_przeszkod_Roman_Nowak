from numba import cuda

# Check if a GPU is detected
print("Is GPU available?", cuda.is_available())

# Get details of the GPU
if cuda.is_available():
    print("GPU Name:", cuda.get_current_device().name)