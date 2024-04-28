# Represents an item that has been analyzed, including its original characteristics and additional processing results.

# Original position and characteristics of the item.
int16 x         # X position of the item
int16 y         # Y position of the item
uint16 diameter # diameter of the item
string colour   # colour of the item
uint8 value     # value of the item

# The computed position of the item relative to a map.
float32 x_coord # X coordinate relative to map
float32 y_coord # Y coordinate relative to map

# Calculated distance from the item to the robot.
float32 distance # distance to the item in meters

