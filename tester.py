"""
    Streaming 6Dof from QTM
"""

import asyncio #asynchronous programming 
import xml.etree.ElementTree as ET # parsing XML data which will be used in our code as ET
import pkg_resources #to locate the resources with the package
import struct
import qtm #this file faciliates the communication to QTM
import math
import random
from datetime import datetime
import numpy as np

# Constants
RADIUS_OF_EARTH = 6371000.0  # Radius of Earth in meters
orthometric_height = 18.893


import serial 
ser = serial.Serial('/dev/ttyUSB0', 115200) #function for establishing serial connt

QTM_FILE = pkg_resources.resource_filename("qtm", "data/demo.qtm") 

#QTM_FILE = "C:\\Users\\LenttyUSB0ovo\\OneDrive\\Desktop\\QTM_Python_SDK\\qualisys_python_sdk-2.1.0\\qtm_rt\\data\\mzdrone0039.qtm"

#projection = pyproj.Proj(proj='utm', zone=10, ellps='WGS84')

def create_body_index(xml_string):
    """ Extract a name to index dictionary from 6dof settings xml """
    xml = ET.fromstring(xml_string)

    body_to_index = {}
    for index, body in enumerate(xml.findall("*/Body/Name")):
        body_to_index[body.text.strip()] = index

    return body_to_index


async def main(): # for connecting to QTM 
    

    # Connect to qtm
    connection = await qtm.connect("192.168.252.1")

    # Connection failed?
    if connection is None:
        print("Failed to connect") 
        return

    # Take control of qtm, context manager will automatically release control after scope end
    async with qtm.TakeControl(connection, "password"):

        realtime = True

        if realtime:
            # Start new realtime
            await connection.new()
        else:
            # Load qtm file
            await connection.load(QTM_FILE)

            # start rtfromfile
            await connection.start(rtfromfile=True)

    # Get 6dof settings from qtm
    xml_string = await connection.get_parameters(parameters=["6d"])
    body_index = create_body_index(xml_string)

    wanted_body = "calib_triangle"

    def on_packet(packet):
        info, bodies = packet.get_6d()
        print(
            "Framenumber: {} - Body count: {}".format(
                packet.framenumber, info.body_count
            )
        )
        # ser.write(b'helloooooooooooooooooo')
        if wanted_body is not None and wanted_body in body_index:
            # Extract one specific body
            wanted_index = body_index[wanted_body]
            position, rotation = bodies[wanted_index]
            print("{} - Pos: {} - Rot: {}".format(wanted_body, position, rotation))
            ##data_packet = struct.pack('<3f', position[0], position[1], position[2]) #convert a floating point value into 
            #NMEA_sentences = struct.pack()
            #print(data_packet)
            #ser.write(data_packet)
            #print(position)
           
            print("x= ","{:.4f}".format(position[0]))
            print("Y= ", "{:.4f}".format(position[1]))
            print("Z= ", "{:.4f}".format(position[2]))

            
            rotation_matrix = np.array(rotation).reshape(3, 3)

        # Print the rotation matrix in formatted way
            print("Rotation Matrix:") 
            trans_rotation_matrix = rotation_matrix.T  
            
            for row in trans_rotation_matrix:
                print("  [" + " ".join("{:.4f}".format(value) for value in row) + "]")
            


            

            print("r00= ", "{:.4f}".format(rotation_matrix[0][0]))
            print("r21= ", "{:.4f}".format(rotation_matrix[2][1]))
            print("r22= ", "{:.4f}".format(rotation_matrix[2][2]))
           

            #print("rotation= ","{:.4f}".format(rotation)) 
            print("___________")
                

                # Calculate roll
            # Assuming rotation_matrix is defined and is a 3x3 NumPy array
            roll = math.atan2(trans_rotation_matrix[2][1], trans_rotation_matrix[2][2])  # Correct usage of atan2

            # Convert roll from radians to degrees if needed
            roll_degrees = math.degrees(roll)

            
            # Calculate pitch

            pitch =math.atan2(-trans_rotation_matrix[2][0], math.sqrt(trans_rotation_matrix[2][1]**2 + trans_rotation_matrix[2][2]**2))
            pitch_degrees =math.degrees(pitch)
            

            # Calculate yaw
          
            yaw = math.atan2(trans_rotation_matrix[1][0], trans_rotation_matrix[0][0])
            yaw_degrees =math.degrees(yaw)
            
            #in radians
            
            print("Roll (radians): {:.4f}".format(roll))
            print("pitch (radians): {:.4f}".format(pitch))
            print("Yaw (radians): {:.4f}".format(yaw))

            #in degrees
            print("Roll (degrees): {:.4f}".format(roll_degrees))
            print("Pitch (degrees): {:.4f}".format(pitch_degrees))
            print("Yaw (degrees): {:.4f}".format(yaw_degrees))
          

            latitude_0 = math.radians(30)
            longitude_0 =math.radians(60)
            

            def convert_xy_to_lat_long(x,y,longitude_0, latitude_0):
                longitude = longitude_0+(x/RADIUS_OF_EARTH)
                latitude  = latitude_0+(y/math.cos(longitude_0)*RADIUS_OF_EARTH)
                print("latitude:{:.4f}".format(latitude))
                return latitude, latitude
                
                       
                    
            
            def radians_to_nmea(radians):
                degrees = radians *(180.0/math.pi)
                deg = int(degrees)
                minutes = (degrees -deg) *60.0
                return f"{deg:02d}{minutes:07.4f}"
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
            
                random_hdop = random.uniform(0.7, 5)
                no_of_satellites = [5,10,15,20]
                randome_satellite = random.choice(no_of_satellites)
                gga_sentence = (f"GPGGA,{formatted_time},{lat_str},{lat_dir},{lon_str},{lon_dir},1,{randome_satellite},{random_hdop:.1f},545.4,M,46.9,M,, "    )
                checksum = calculate_checksum(gga_sentence)
                print(f"${gga_sentence}*{checksum:02X}")
                return gga_sentence

            def generate_rmc(latitude, longitude, speed_knots, track_angle, formatted_time):
                lat_str = radians_to_nmea(latitude)
                lon_str = radians_to_nmea(longitude)
                lat_dir = 'N' if latitude >= 0 else 'S'
                lon_dir = 'E' if longitude >= 0 else 'W'

                rmc_sentence = (f"GPRMC,{formatted_time},A,{lat_str},{lat_dir},{lon_str},{lon_dir},  "
                    f"{speed_knots:.1f},{track_angle:.1f},230394,003.1,W ")
                checksum = calculate_checksum(rmc_sentence)
                print(f"${rmc_sentence}*{checksum:02X}")
                return rmc_sentence

            # Function to generate NMEA VTG sentence
            def generate_vtg(track_angle, speed_knots):
                vtg_sentence = (f"GPVTG,{track_angle:.4f},T,,M,{speed_knots:.4f},N,{speed_knots * 1.852:.1f},K "   )
                checksum = calculate_checksum(vtg_sentence)
                print(f"${vtg_sentence}*{checksum:02X}")
                return vtg_sentence
   
            def generate_rpy(roll_degrees,pitch_degrees,yaw_degrees):
                rpy_sentence =(f"GPRPY,{roll_degrees:.3f},{pitch_degrees:.4f}, {yaw_degrees:.4f}"  )
                checksum = calculate_checksum(rpy_sentence)
                print(f"${rpy_sentence}*{checksum:02X}")
                return rpy_sentence
            
            x = position[0]
            y = position[1]
            z = position[2]
            current_time = datetime.now()
            formatted_time = current_time.strftime("%H%M%S")
            print("current time:", formatted_time)

            latitude_0 = math.radians(30)
            longitude_0 =math.radians(60)
            speed_knots = 5.5 #speed in knots
            track_angle = 54.7 # track angle in degrees
            latitude, longitude = convert_xy_to_lat_long(x,y, longitude_0, latitude_0)
            #generate_gga(latitude, longitude,formatted_time)
            #generate_rmc(latitude, longitude, speed_knots, track_angle, formatted_time)
            #generate_vtg(track_angle, speed_knots)


            #GGA sentence 
            gga_sentence = generate_gga(latitude, longitude,formatted_time)
            gga_bytes = gga_sentence.encode('ascii') #converts the human readable string into ascii
            gga_packed_data =struct.pack(f'{len(gga_bytes)}s', gga_bytes) #packs the gga_bytes data into binary format
            

            #rmc sentence 
            rmc_sentence = generate_rmc(latitude, longitude, speed_knots, track_angle, formatted_time)
            rmc_bytes = rmc_sentence.encode('ascii')
            rmc_packed_data = struct.pack(f'{len(rmc_bytes)}s', rmc_bytes)
            


            #vtg sentence
            vtg_sentence = generate_vtg(track_angle, speed_knots)
            vtg_bytes = vtg_sentence.encode('ascii')
            vtg_packed_data = struct.pack(f'{len(vtg_bytes)}s', vtg_bytes)

            rpy_sentence = generate_rpy(roll_degrees, pitch_degrees, yaw_degrees)
            rpy_bytes = vtg_sentence.encode('ascii')
            rpy_packed_data = struct.pack(f'{len(rpy_bytes)}s', rpy_bytes)
            



            ser.write(gga_packed_data) #sends the packed data over serial communication
            ser.write(vtg_packed_data)
            ser.write(rmc_packed_data)
            #command_to_be_sent = gga_packed_data + rmc_packed_data + vtg_packed_data
            #ser.write(command_to_be_sent)

            #data_packet = struct.pack('<3f', position[0], position[1], position[2]) #convert a floating point value into 
            #ser.write(data_packet)
            


              



    


        else:
            # Print all bodiesab
            
            
             
            for position, rotation in bodies:
                #print("{} - Pos: {} - Rot: {}".format(wanted_body, position, rotation))
                print("x= ","{:.4f}".format(position[0]))
                print("Y= ", "{:.4f}".format(position[1]))
                print("Z= ", "{:.4f}".format(position[2]))
                print("___________")
            #for index, body in enumerate(xml.findall("*/Body/Name")):
                #number = 000
                data_packet = struct.pack('<3f', position[0], position[1], position[2]) #convert a floating point value into 
                ser.write(data_packet)
                #holder = 'hello the code is working'
                #ser.write(holder)
                #print(data_packet)
                #print(unpack_data_packet)
                
        
               
    # Start streaming frames
    await connection.stream_frames(components=["6d"], on_packet=on_packet)
  

    # Wait asynchronously 5 seconds
    await asyncio.sleep(15)

    # Stop streaming
    await connection.stream_frames_stop()
    # await connection.close()
    
    ser.close()
    


if __name__ == "__main__":
    # Run our asynchronous function until complete
    asyncio.get_event_loop().run_until_complete(main())
