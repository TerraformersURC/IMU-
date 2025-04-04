#include "serial.h"
#include "wit_c_sdk.h"
#include "REG.h"
#include <stdint.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#define SHM_KEY 12345

#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80

struct shared_data {
    long shm_fAcc[3];  
    long shm_fGyro[3]; 
    long shm_fAngle[3]; 
    int shm_sReg[3];  
};

struct shared_data *shm_data;

static int fd, s_iCurBaud = 9600;
static volatile char s_cDataUpdate = 0;


const int c_uiBaud[] = {2400 , 4800 , 9600 , 19200 , 38400 , 57600 , 115200 , 230400 , 460800 , 921600};


static void AutoScanSensor(char* dev);
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);
//0,owner, group, others 
void init_shared_memory() {
    int shmid = shmget(SHM_KEY, sizeof(struct shared_data), IPC_CREAT | 0666);
    if (shmid == -1) {
        perror("shmget failed");
        exit(1);
    }

    shm_data = (struct shared_data*)shmat(shmid, NULL, 0);
    if (shm_data == (void*)-1) {
        perror("shmat failed");
        exit(1);
    }
    else{
        printf("memory shared!\n");
    }
    memset(shm_data, 0, sizeof(struct shared_data));
}
void cleanup_shared_memory() {
    if (shmdt(shm_data) == -1) {
        perror("shmdt failed");
    }
}
int add_to_memory(){
    if (s_cDataUpdate & ACC_UPDATE)
        {
            
            if (s_cDataUpdate & ACC_UPDATE)
            {
                // Using shm_fAcc instead of fAcc
                shm_data->shm_fAcc[0] = sReg[AX] / 32768.0f * 16.0f;
                shm_data->shm_fAcc[1] = sReg[AX + 1] / 32768.0f * 16.0f;
                shm_data->shm_fAcc[2] = sReg[AX + 2] / 32768.0f * 16.0f;
                // printf("acc:%.3ld %.3ld %.3ld\r\n", shm_data->shm_fAcc[0], shm_data->shm_fAcc[1], shm_data->shm_fAcc[2]);
                s_cDataUpdate &= ~ACC_UPDATE;  // Clear the flag
            }
        
            // Update gyroscope data
            if (s_cDataUpdate & GYRO_UPDATE)
            {
                shm_data->shm_fGyro[0] = sReg[GX] / 32768.0f * 2000.0f;
                shm_data->shm_fGyro[1] = sReg[GX + 1] / 32768.0f * 2000.0f;
                shm_data->shm_fGyro[2] = sReg[GX + 2] / 32768.0f * 2000.0f;
                // printf("gyro:%.3ld %.3ld %.3ld\r\n", shm_data->shm_fGyro[0], shm_data->shm_fGyro[1], shm_data->shm_fGyro[2]);
                s_cDataUpdate &= ~GYRO_UPDATE;  // Clear the flag
            }
        
            // Update angle data
            if (s_cDataUpdate & ANGLE_UPDATE)
            {
                shm_data->shm_fAngle[0] = sReg[Roll] / 32768.0f * 180.0f;
                shm_data->shm_fAngle[1] = sReg[Roll + 1] / 32768.0f * 180.0f;
                shm_data->shm_fAngle[2] = sReg[Roll + 2] / 32768.0f * 180.0f;
                // printf("angle:%.3ld %.3ld %.3ld\r\n", shm_data->shm_fAngle[0], shm_data->shm_fAngle[1], shm_data->shm_fAngle[2]);
                s_cDataUpdate &= ~ANGLE_UPDATE;  // Clear the flag
            }
        
            // Update magnetometer data
            if (s_cDataUpdate & MAG_UPDATE)
            {
                shm_data->shm_sReg[0] = sReg[HX];
                shm_data->shm_sReg[1] = sReg[HY];
                shm_data->shm_sReg[2] = sReg[HZ];
                // printf("mag : %d %d %d\r\n", sReg[HX], sReg[HY], sReg[HZ]);
                s_cDataUpdate &= ~MAG_UPDATE;  // Clear the flag
            }
    }
}
int main(int argc,char* argv[]){
	
	if(argc < 2)
	{
		printf("please input dev name\n");
		return 0;
	}


    if((fd = serial_open(argv[1] , 9600)<0))
	 {
	     printf("open %s fail\n", argv[1]);
	     return 0;
	 }
	else printf("open %s success\n", argv[1]);
    init_shared_memory();

	float fAcc[3], fGyro[3], fAngle[3];
	int i , ret;
	char cBuff[1];
	
	WitInit(WIT_PROTOCOL_NORMAL, 0x50);
	WitRegisterCallBack(SensorDataUpdata);
	
	// printf("\r\n********************** wit-motion Normal example  ************************\r\n");
	AutoScanSensor(argv[1]);
	
	while(1){
	    while(serial_read_data(fd, cBuff, 1)){
        WitSerialDataIn(cBuff[0]);
        }

        // printf("\n");
        Delayms(500);
    
        if(s_cDataUpdate){
            add_to_memory();   
        }
    }       
        cleanup_shared_memory();
        serial_close(fd);
        return 0;

}
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{
    int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
//            case AX:
//            case AY:
            case AZ:
				s_cDataUpdate |= ACC_UPDATE;
            break;
//            case GX:
//            case GY:
            case GZ:
				s_cDataUpdate |= GYRO_UPDATE;
            break;
//            case HX:
//            case HY:
            case HZ:
				s_cDataUpdate |= MAG_UPDATE;
            break;
//            case Roll:
//            case Pitch:
            case Yaw:
				s_cDataUpdate |= ANGLE_UPDATE;
            break;
            default:
				s_cDataUpdate |= READ_UPDATE;
			break;
        }
		uiReg++;
	}
}

static void Delayms(uint16_t ucMs)
{ 
     usleep(ucMs*500);
}
 
	
static void AutoScanSensor(char* dev)
{
	int i, iRetry;
	char cBuff[1];
	
	for(i = 1; i < 10; i++)
	{
		serial_close(fd);
		s_iCurBaud = c_uiBaud[i];
		fd = serial_open(dev , c_uiBaud[i]);
		
		iRetry = 2;
		do
		{
			s_cDataUpdate = 0;
			WitReadReg(AX, 3);
			Delayms(200);
			while(serial_read_data(fd, cBuff, 1))
			{
				WitSerialDataIn(cBuff[0]);
			}
			if(s_cDataUpdate != 0)
			{
				printf("%d baud find sensor\r\n\r\n", c_uiBaud[i]);
				return ;
			}
			iRetry--;
		}while(iRetry);		
	}
	printf("can not find sensor\r\n");
	printf("please check your connection\r\n");
}
