%% Loading Point Cloud Data

lidar_data = load(lidarFileName);
lidar_PC = lidar_data.lidarPointClouds;

head(lidar_PC)

%% Loading GPS Data

gps_data = load(gpsFileName);
gps_sequence = gps_data.gpsSequence;

head(gps_sequence)

%% Loading IMU Data

imu_data = load(imuFileName);
imu_orientation = imu_data.imuOrientations;

% converting the orientation into quaternion type
imu_orientation = convertvars(imu_orientation,'Orientation','quaternion');

% display the imu data
head(imu_orientation)
 