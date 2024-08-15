import math
from datetime import datetime
import random

# Constants
RADIUS_OF_EARTH = 6371000.0  # Radius of Earth in meters



# Function to convert x, y to latitude and longitude
def convert_xy_to_lat_long(x, y, longitude_0, latitude_0):
    longitude = longitude_0 + (x / RADIUS_OF_EARTH)
    latitude = latitude_0 + (y / (math.cos(longitude_0) * RADIUS_OF_EARTH))
    return latitude, longitude

# Helper function to convert radians to NMEA format (DDMM.MMMM)
def radians_to_nmea(radians):
    degrees = radians * (180.0 / math.pi)
    deg = int(degrees)
    minutes = (degrees - deg) * 60.0
    return f"{deg:02d}{minutes:07.4f}"

# Function to generate a simple checksum for NMEA sentences
def calculate_checksum(sentence):
    checksum = 0
    for char in sentence:
        checksum ^= ord(char)
    return checksum

# Function to generate NMEA GGA sentence
def generate_gga(latitude, longitude, formatted_time):
    
    lat_str = radians_to_nmea(latitude)
    lon_str = radians_to_nmea(longitude)
    lat_dir = 'N' if latitude >= 0 else 'S'
    lon_dir = 'E' if longitude >= 0 else 'W'
    orthometric_height = 18.893
    
    
    random_hdop = random.uniform(0.7, 5)
    no_of_satellites = [5,10,15,20]
    randome_satellite = random.choice(no_of_satellites)

    gga_sentence = (f"$GPGGA,{formatted_time},{lat_str},{lat_dir},{lon_str},{lon_dir},1,{randome_satellite},{random_hdop:.1f},545.4,M,46.9,M,,")
    
    

    checksum = calculate_checksum(gga_sentence)
    print(f"${gga_sentence}*{checksum:02X}")

# Function to generate NMEA RMC sentence
def generate_rmc(latitude, longitude, speed_knots, track_angle, formatted_time):
    lat_str = radians_to_nmea(latitude)
    lon_str = radians_to_nmea(longitude)
    lat_dir = 'N' if latitude >= 0 else 'S'
    lon_dir = 'E' if longitude >= 0 else 'W'

    rmc_sentence = (f"GPRMC,{formatted_time},A,{lat_str},{lat_dir},{lon_str},{lon_dir},"
                    f"{speed_knots:.1f},{track_angle:.1f},230394,003.1,W")
    checksum = calculate_checksum(rmc_sentence)
    print(f"${rmc_sentence}*{checksum:02X}")

# Function to generate NMEA VTG sentence
def generate_vtg(track_angle, speed_knots):
    vtg_sentence = (f"GPVTG,{track_angle:.1f},T,,M,{speed_knots:.1f},N,{speed_knots * 1.852:.1f},K")
    checksum = calculate_checksum(vtg_sentence)
    print(f"${vtg_sentence}*{checksum:02X}")

def main():
    # Input x, y, and reference latitude_0 and longitude_0
    x = 1000.0  # Example x value in meters
    y = 500.0   # Example y value in meters
    latitude_0 = math.radians(30)  # Example reference latitude in radians (30 degrees)
    longitude_0 = math.radians(60) # Example reference longitude in radians (60 degrees)
    
    # Get the current time
    current_time = datetime.now()

    # Format the time
    formatted_time = current_time.strftime("%H%M%S")

    # Display the formatted time
    print("Current Time:", formatted_time)


    # Convert x, y to latitude and longitude
    latitude, longitude = convert_xy_to_lat_long(x, y, longitude_0, latitude_0)

    # Example speed and track angle
    speed_knots = 5.5  # Speed in knots
    track_angle = 54.7 # Track angle in degrees
    # Generate and print NMEA sentences
    generate_gga(latitude, longitude, formatted_time)
    generate_rmc(latitude, longitude, speed_knots, track_angle, formatted_time)
    generate_vtg(track_angle, speed_knots)

if __name__ == "__main__":
    main()
