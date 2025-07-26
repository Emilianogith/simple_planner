from PIL import Image
import os
import yaml

input_path = '../maps/maze.png'
pgm_path = '../maps/maze.pgm'
yaml_path = '../maps/maze.yaml'


img = Image.open(input_path).convert('L')  
# save the pgm file
img.save(pgm_path)


map_metadata = {
    'image': os.path.basename(pgm_path),
    'resolution': 0.05,  
    'origin': [0.0, 0.0, 0.0], 
    'negate': 0, 
    'occupied_thresh': 0.65,
    'free_thresh': 0.196
}

# save the YAML file
with open(yaml_path, 'w') as f:
    yaml.dump(map_metadata, f, default_flow_style=False)

print(f"Generated {pgm_path} and {yaml_path}")