"""
    Streaming 6Dof from QTM
"""
import pyproj
import asyncio #asynchronous programming 
import xml.etree.ElementTree as ET # parsing XML data which will be used in our code as ET
import pkg_resources #to locate the resources with the package
import struct
import qtm #this file faciliates the communication to QTM

import serial 
ser = serial.Serial('/dev/ttyUSB0', 115200) #function for establishing serial connt

QTM_FILE = pkg_resources.resource_filename("qtm", "data/demo.qtm") 

#QTM_FILE = "C:\\Users\\LenttyUSB0ovo\\OneDrive\\Desktop\\QTM_Python_SDK\\qualisys_python_sdk-2.1.0\\qtm_rt\\data\\mzdrone0039.qtm"

projection = pyproj.Proj(proj='utm', zone=10, ellps='WGS84')

def create_body_index(xml_string):
    """ Extract a name to index dictionary from 6dof settings xml """
    xml = ET.fromstring(xml_string)

    body_to_index = {}
    for index, body in enumerate(xml.findall("*/Body/Name")):
        body_to_index[body.text.strip()] = index

    return body_to_index


async def main(): # for connecting to QTM 
    

    # Connect to qtm
    connection = await qtm.connect("149.201.162.82")

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
            #data_packet = struct.pack('<3f', position[0], position[1], position[2]) #convert a floating point value into 
            #ser.write(data_packet)
            #print(position)




        else:
            # Print all bodiesab
            for position, rotation in bodies:
                print("{} - Pos: {} - Rot: {}".format(wanted_body, position, rotation))
            #for index, body in enumerate(xml.findall("*/Body/Name")):
                #number = 000
                data_packet = struct.pack('<3f', position[0], position[1], position[2]) #convert a floating point value into 
                ser.write(data_packet)
                print(data_packet)
                #print(unpack_data_packet)

               
    # Start streaming frames
    await connection.stream_frames(components=["6d"], on_packet=on_packet)
  

    # Wait asynchronously 5 seconds
    await asyncio.sleep(20)

    # Stop streaming
    await connection.stream_frames_stop()
    # await connection.close()
    
    ser.close()

if __name__ == "__main__":
    # Run our asynchronous function until complete
    asyncio.get_event_loop().run_until_complete(main())
