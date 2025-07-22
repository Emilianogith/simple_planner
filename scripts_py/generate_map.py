from PIL import Image, ImageDraw

# Create a 100x100 white image (free space)
img = Image.new('L', (200, 200), color=255)

draw = ImageDraw.Draw(img)
top_left = (40, 40)
bottom_right = (60, 60)
draw.rectangle([top_left, bottom_right], fill=0)


img.save('../maps/map_with_obstacle.pgm')