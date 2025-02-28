from build123d import *
from ocp_vscode import show  # Optional: For previewing the model

# Define torus parameters
minor_radius = 0.11  # Inner tube radius
major_radius = .4064 -minor_radius # Outer radius


# Create torus
torus = Torus(major_radius, minor_radius)

# Export the torus mesh
export_stl(torus, "torus.stl")    # STL format (for Gazebo)
