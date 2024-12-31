#RSDS Lidar Client 폴더 위치
{CarMaker 프로젝트 폴더 위치}/RSDS_Lidar

#build
make clean; make

#실행코드
./rsds-client.exe -f 220614_DemoCar_SensorLidarRSI_Velodyne -p 2211
- f: 사용하는 차량파일
- p: 연결하고자 하는 GPUSensor의 port 번호
- 참고) {차량 파일}/Additional tap: UDP 송신하는 ip 주소
    - Sensor.LidarRSI.IP = 	localhost


./rsds-client.exe -f Hyundai_Solati_2015_V2X_dhk -p 2211
