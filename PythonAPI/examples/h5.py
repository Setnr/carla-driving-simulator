import tables
from tables import *
import numpy as np
import h5py
import json 
import math
import threading
import carla

lock = threading.Lock()

RADAR_DEFAULT_MOUNTING = {
    1: {"x": 3.663, "y": -0.873, "yaw": -1.48418552},
    2: {"x": 3.86, "y": -0.70, "yaw": -0.436185662},
    3: {"x": 3.86, "y": 0.70, "yaw": 0.436},
    4: {"x": 3.663, "y": 0.873, "yaw": 1.484},
}

class Odometry(IsDescription):
    timestamp       = Int64Col(pos=0)    # float  (single-precision)
    x_seq           = Float32Col(pos=1)    # float  (single-precision)
    y_seq           = Float32Col(pos=2)    # float  (single-precision)
    yaw_seq         = Float32Col(pos=3)    # float  (single-precision)
    vx              = Float32Col(pos=4)    # float  (single-precision)
    yaw_rate        = Float32Col(pos=5)    # float  (single-precision)

class RadarDetection(IsDescription):
    timestamp       = Int64Col(pos=0)    # float  (single-precision)
    sensor_id       = Int32Col(pos=1)
    range_sc        = Float32Col(pos=2)    # float  (single-precision)
    azimuth_sc      = Float32Col(pos=3)    # float  (single-precision)
    rcs             = Float32Col(pos=4)    # float  (single-precision)
    vr              = Float32Col(pos=5)    # float  (single-precision)
    vr_compensated  = Float32Col(pos=6)    # float  (single-precision)
    x_cc            = Float32Col(pos=7)    # float  (single-precision)
    y_cc            = Float32Col(pos=8)    # float  (single-precision)
    x_seq           = Float32Col(pos=9)    # float  (single-precision)
    y_seq           = Float32Col(pos=10)    # float  (single-precision)
    uuid            = tables.StringCol(25,pos=11)    # float  (single-precision) String otherwise you can watch the Information tab (load crashes)
    track_id        = tables.StringCol(25,pos=12)    # float  (single-precision) String benötigt um eine Leere Zelle zu generieren (track_id gibt an was für einem Objekt der Punkt zugeordnet wird)
    label_id        = Float32Col(pos=13)    # float  (single-precision)

class RadarScenes:
    def __init__(self,hdffilename):

        self.h5file = open_file(hdffilename + "radar_data.h5", mode="w", title="Test hdf5 file")
        self.tbl_odom = self.h5file.create_table(where="/", name='odometry', description=Odometry, title="RadaData")
        self.tbl_radar = self.h5file.create_table(where="/", name='radar_data', description=RadarDetection, title="RadaData")


        self.TimeStamp = 0
        self.OdoMetryCounter = 0
        self.jsonname = hdffilename +"scenes.json"
        self.jsonfile = open(self.jsonname, "w")

        self.last_odomentry_index = 0
        self.last_radardata_index = 0
        self.meta_header = dict()
        self.meta_header["sequence_name"] = "sequence_1"
        self.meta_header["category"] = "train"
        self.meta_header["first_timestamp"] = 0
        self.meta_header["last_timestamp"] = 0
        self.meta_header["scenes"] = dict() 
        self.uuid = 0

    def close(self):
        self.last_odomentry_index = len(self.tbl_odom.cols.timestamp)-1
        self.last_radardata_index = len(self.tbl_radar.cols.timestamp)-1
        self.meta_header["last_timestamp"] = self.TimeStamp
        self.LastDetection["next_timestamp"] = None
        self.FixIndices()
        json.dump(self.meta_header, self.jsonfile, indent=2)
        self.h5file.close()
        self.jsonfile.close()

    def SaveRadarData(self,detect,  radar,  oddometry):
        # Calculate X and Y without roll and yaw of object, because Viewer loads Oddometry and calculates Points
        # based on X/Y and roll/yaw of the car
        lock.acquire()
        try:
            azi = math.degrees(detect.azimuth)
            alt = math.degrees(detect.altitude)
            fw_vec = carla.Vector3D(x=detect.depth)
            rotation = radar.get_transform().rotation
            carla.Transform(carla.Location(), carla.Rotation(pitch=alt, 
                        yaw=azi + rotation.yaw, roll=0)).transform(fw_vec) #rotation.yaw  was radar.Yaw
            
            entry = self.tbl_radar.row
            entry['timestamp'] = self.TimeStamp
            entry['sensor_id'] = radar.ID
            entry['range_sc'] = detect.depth
            #azimuth * -1 -> Drehen der Y-Achse da diese unterschiedlich ist in Carla und Radar Scenes
            #GetRadarMounting -> Scenes added fixe Yaw die muss rausgerechnet werden
            #math.radians(180) -> Rotiere um die Richtige Richtung der Anzeige zu haben
            #math.radians(radar.Yaw * -1) Rotiere basierende auf Radar rotation (*-1 wegen Y-Achse xD)
            entry['azimuth_sc'] =  (detect.azimuth * -1) + self.GetRadarMounting(radar.ID) + math.radians(180) + math.radians(radar.Yaw * -1)# + 1.48418552 #+ math.radians(oddometry.rotation.yaw * -1) 
            #print(f"UUID: {str(self.uuid)} Azi-Radians={detect.azimuth} Azi-Degre={azi} yaw={rotation.yaw}")
            entry['rcs'] = 0
            entry['vr'] = 0
            entry['vr_compensated'] = detect.velocity *-1
            entry['x_cc'] = fw_vec.y # Diese beiden Spalten rotieren noch, wenn das fahrzeug abbiegt
            entry['y_cc'] = fw_vec.x      #
            entry['x_seq'] = fw_vec.x 
            entry['y_seq'] = (fw_vec.y) * -1
            entry['uuid'] = str(self.uuid)
            self.uuid = self.uuid +  1
            entry['track_id'] = ""
            entry['label_id'] = RadarScenes.ConvertLabelFromCarla(detect.label)
            entry.append()
            self.tbl_radar.flush()

            detection = dict()
            self.LastDetection = detection
            detection["sensor_id"] = radar.ID
            detection["prev_timestamp"] = self.TimeStamp - 1
            detection["next_timestamp"] = self.TimeStamp + 1 
            detection["prev_timestamp_same_sensor"] = self.TimeStamp - 1
            detection["next_timestamp_same_sensor"] = self.TimeStamp + 1
            detection["odometry_timestamp"] = self.TimeStamp
            detection["odometry_index"] = self.OdoMetryCounter 
            detection["image_name"] = f"{self.TimeStamp}.jpg" 
            detection["radar_indices"] = list()
            if hasattr(radar,"LastDetection") and detection != self.LastDetection:
                radar.LastDetection["next_timestamp"] = self.TimeStamp
                radar.LastDetection["next_timestamp_same_sensor"] = self.TimeStamp
            radar.LastDetection = detection

            entry = self.tbl_odom.row
            entry['timestamp'] =self.TimeStamp
            entry['x_seq'] = oddometry.location.x
            entry['y_seq'] = oddometry.location.y
            entry['yaw_seq'] = oddometry.rotation.yaw
            entry['vx'] = 0
            entry['yaw_rate'] = 0
            entry.append()
            self.tbl_odom.flush()
            self.OdoMetryCounter += 1
            radar_indices = [1, 2] # no idea what that is, But i know ò.ó
            detection["radar_indices"] = radar_indices
            self.meta_header["scenes"][self.TimeStamp] = detection
        finally:
            lock.release()
        

    def GetTimeStamp(self):
        return self.TimeStamp

    def NewTimeStamp(self):
        self.TimeStamp += 1

    def FixIndices(self):
        start = 0
        for scene in self.meta_header["scenes"]:
            reduced = [x for x in self.tbl_radar.iterrows() if x["timestamp"] == scene]
            self.meta_header["scenes"][scene]["radar_indices"]=[start, start + len(reduced)]
            start +=len(reduced)

    def GetRadarMounting(self, ID):
        if ID > 4: 
            return 0
        else:
            return RADAR_DEFAULT_MOUNTING[ID]["yaw"] * -1

    @staticmethod
    def ConvertLabelFromCarla(label):
        labelList = [10]
        if label == 0: 
            return 11
        for i in range(32):
            NewLabel = 11
            if int(label) >> i & 1: 
                i = i + 1 #Offset wegen der 0 damit Label 0 = 0 und label 1 = 1 muss beim bitshift 1 versetzt werden daher in carla selbst (tag - 1 in Radar.cpp)
                if i == 14:
                    NewLabel = 0
                elif i == 15:
                    NewLabel = 2
                elif i == 16:
                    NewLabel = 3
                elif i == 17:
                    NewLabel = 4
                elif i == 19:
                    NewLabel = 5
                elif i == 18:
                    NewLabel = 6
                elif i == 12:
                    NewLabel = 8
                elif i > 0 and i < 12:
                    NewLabel = 10
                else:
                    NewLabel = 11
                labelList.append(NewLabel)
        return labelList[-1]

class Label(Enum):
    """
    The Labels enum contains all semantic labels available in the data set.
    """
    CAR = 0
    LARGE_VEHICLE = 1
    TRUCK = 2
    BUS = 3
    TRAIN = 4
    BICYCLE = 5
    MOTORIZED_TWO_WHEELER = 6
    PEDESTRIAN = 7
    PEDESTRIAN_GROUP = 8
    ANIMAL = 9
    OTHER = 10
    STATIC = 11

    @staticmethod
    def label_id_to_name(label_id: int) -> str:
        """
        Convert an integer label_id to the string representation of the corresponding enum
        :param label_id: Label ID of a class for which a string is desired.
        :return: The class name as a string
        """
        return Label(label_id).name