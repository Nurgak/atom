#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Duckietown map generator

Duckietown map generator takes the Duckietown road tiles and generates an
image that can be printed or imported in a simulation for testing.

The tiles have been taken from the Duckietown project and cropped/resized.
"""

import sys
from PIL import Image

__author__ = "Karl Kangur"
__license__ = "MIT"
__version__ = "1.0"
__email__ = "karl.kangur@gmail.com"

# Assembled map size in hirozontal direction (aspect ratio will be kept)
map_resolution_x = 6000

# Map definition
map = ["br sh bl gr gr br sh 3t sh bl",
   "sv wa tr sh 3t tl gr sv gr sv",
   "3l sh bl gr sv gr gr sv gr sv",
   "sv gr sv gr sv gr br 3b sh 3r",
   "sv gr sv gr sv gr sv gr gr sv",
   "tr 3t 3b sh 4i sh 4i sh wa sv",
   "wa sv gr gr sv gr sv gr gr sv",
   "wa sv gr br tl gr sv gr gr sv",
   "wa sv gr sv gr gr sv gr br tl",
   "wa tr sh 3b sh sh 3b sh tl gr"]

# Tile definition
tile = {
    'sv': 'tile_straight_vertical.png',
    'sh': 'tile_straight_horizontal.png',
    'tl': 'tile_curve_top_left.png',
    'tr': 'tile_curve_top_right.png',
    'bl': 'tile_curve_bottom_left.png',
    'br': 'tile_curve_bottom_right.png',
    '3t': 'tile_intersection_3_top.png',
    '3r': 'tile_intersection_3_right.png',
    '3l': 'tile_intersection_3_left.png',
    '3b': 'tile_intersection_3_bottom.png',
    '4i': 'tile_intersection_4.png',
    'gr': 'tile_grass.png',
    'wa': 'tile_water.png'
    }

# Count map rows and columns
map_rows = len(map)
map_cols = len(map[0].split(" "))

# Scale the y according to map size
scale_xy = float(map_rows) / float(map_cols)
# Create output image, y resolution depends on map ratio
road = Image.new('RGB', (map_resolution_x, int(map_resolution_x * scale_xy)))
costmap = Image.new('RGB', (map_resolution_x, int(map_resolution_x * scale_xy)))

for y, row in zip(range(map_rows), map):
    tiles = row.split(" ")
    for x, type in zip(range(map_cols), tiles):
        # Get the tile type
        tile_type = tile[type]
        # Fetch the tile image
        tile_image = Image.open("tiles/" + tile_type)
        costmap_image = Image.open("costmap/" + tile_type)
        # Get size according to requested resolution
        size = (map_resolution_x / len(tiles))
        # Resize image
        tile_image = tile_image.resize((size, size), Image.ANTIALIAS)
        costmap_image = costmap_image.resize((size, size), Image.ANTIALIAS)
        # Blit image to road image
        road.paste(tile_image, (size * x, size * y))
        costmap.paste(costmap_image, (size * x, size * y))

        # Report progress every 10%
        progress = 100 * (x + y * map_cols) / (map_rows * map_cols)
        if progress % 10 == 0:
            print("Progress: %d" % progress)

# Output the generated map to "road.png"
road.save('duckietown_road.png')
costmap.save('duckietown_costmap.png')
