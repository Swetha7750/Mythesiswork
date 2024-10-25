import math

# Constants
RADIUS_OF_EARTH = 6371000  # in meters

# Origin latitude and longitude in degrees
origin_latitude = 50.764905 # degrees
origin_longitude = 6.078757  # degrees

# x, y coordinates in meters
x = 511.4997  # meters
y = -1111.8373  # meters

# Convert origin latitude to radians for calculation
origin_latitude_rad = math.radians(origin_latitude)

# Latitude calculation
latitude = origin_latitude + (y / RADIUS_OF_EARTH) * (180 / math.pi)

# Longitude calculation (accounting for Earth's curvature at given latitude)
longitude = origin_longitude + (x / (RADIUS_OF_EARTH * math.cos(origin_latitude_rad))) * (180 / math.pi)

print("Converted Latitude:", latitude)
print("Converted Longitude:", longitude)
