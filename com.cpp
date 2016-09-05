#include "common.h"
#include "protocol.h"
#include "aes.h"

#define READ_BUFF_LEN 256
#define READ_INTERVAL_TIME 5
#define WRITE_INTERVAL_TIME 5
#define AES_KEY_DIR	"./AES.TXT"

unsigned char packageRecv[PACKAGE_LEN];
unsigned char packageSend[PACKAGE_LEN];
unsigned char aesData[AES_DATA_LEN];

int fd;
string readBuff;
void (*cmdFunc)(unsigned char* data,unsigned char len) = NULL;
uint8_t* key = NULL;
int keyLen = 0;
uint8_t *w; // expanded key

int OpenDev(char *Dev)
{
    int fd = open(Dev,O_RDWR|O_NOCTTY);  
    if(-1 == fd)   
    {
		ERROR("Can't Open Serial Port");
        return -1;      
    }   
    else
	{
		DEBUG("Open Serial Port Success");    
        return fd;
	}
}

//@brief Set Baud Rate
//@param fd serial port handle
//@param speed Baud Rate
int speedPara[] = {B115200,B38400,B19200,B9600,B4800,B2400,B1200,B300,
                   B115200,B38400,B19200,B9600,B4800,B2400,B1200,B300,};
int speedValue[] = {115200,38400,19200,9600,4800,2400,1200,300,
				    115200,38400,19200,9600,4800,2400,1200,300,};
void setSpeed(int fd,int speed)
{
    int i;
    int status;
    struct termios Opt;
    tcgetattr(fd,&Opt);
    for(i= 0;i<sizeof(speedValue)/sizeof(int);i++)
	{
		if(speed == speedValue[i])
		{
			tcflush(fd,TCIOFLUSH);
            cfsetispeed(&Opt,speedPara[i]);
            cfsetospeed(&Opt,speedPara[i]);
            status = tcsetattr(fd,TCSANOW,&Opt);
            if(status!= 0)
			{
                ERROR("Set Speed Error");
                return;
            }
            tcflush(fd,TCIOFLUSH);
        }
    }
	DEBUG("Speed Set Success");
}

//@brief set data bit、stop bit、check type
//@param fd			serial port handle
//@param databits	data bit	7 or 8
//@param stopbits	stop bit	1 or 2
//@param parity		check type	N,E,O,S
int setParity(int fd,int databits,int stopbits,int parity)
{ 
    struct termios options; 
    options.c_lflag &= ~(ICANON|ECHO|ECHOE|ISIG);	//Input
    options.c_oflag &= ~OPOST;						//Output
    if(tcgetattr(fd,&options)!=0)
	{ 
        ERROR("Get Serial Port Opt Error");     
        return(FALSE);  
    }
    options.c_cflag &= ~CSIZE;
	 //Set Data Bit 
    switch (databits)
    {   
    	case 7:     
        	options.c_cflag |= CS7; 
        	break;
    	case 8:     
        	options.c_cflag |= CS8;
        	break;   
    	default:
			ERROR("Data Bit Error");
			return (FALSE);  
    }
    switch (parity) 
    {   
        case 'n':
        case 'N':    
            options.c_cflag &= ~PARENB;	//Clear Parity Enable
            options.c_iflag &= ~INPCK;	//Enable Parity Checking 
            break;  
        case 'o':   
        case 'O':     
            options.c_cflag |= (PARODD | PARENB);	//Odd  
            options.c_iflag |= INPCK;				//Disnable Parity Checking 
            break;  
        case 'e':  
        case 'E':   
            options.c_cflag |= PARENB;	//Enable Parity    
            options.c_cflag &= ~PARODD;	//Even     
            options.c_iflag |= INPCK;	//Disnable Parity Checking
            break;
        case 's': 
        case 'S':
			//As No Parity   
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;break;  
        default:
			ERROR("Parity Bit Error");    
            return (FALSE);  
    }  
    //Set Stop Bit
    switch (stopbits)
    {   
        case 1:    
            options.c_cflag &= ~CSTOPB;  
            break;  
        case 2:    
            options.c_cflag |= CSTOPB;  
           break;
        default:
			ERROR("Stop Bit Error");     
			return (FALSE); 
    } 
    //Set Input Parity Option  
    if (parity != 'n')   
        options.c_iflag |= INPCK; 
    tcflush(fd,TCIFLUSH);
    options.c_cc[VTIME] = 150;	//Timeout 15 Seconds   
    options.c_cc[VMIN] = 0;		//Update The Options And Do It Now
    if (tcsetattr(fd,TCSANOW,&options) != 0)   
    { 
        ERROR("Set Serial Port Opt Error");   
        return (FALSE);  
    }
	DEBUG("Set Serial Port Opt Success");
    return (TRUE);  
}

void SendPackage();
int Pack(unsigned char* dataSrc,unsigned char dataLen,unsigned short cmdType);

void EchoData(unsigned char* data,unsigned char len)
{
	DEBUG("data = %s",data);
}

void HwaetBeat(unsigned char* data,unsigned char len)
{
	//unsigned char data[HEARTBEAT_DATA_LEN] = "";//please input heart beat data
	//Pack(data,HEARTBEAT_DATA_LEN,1000);
	SendPackage();
}

//@brief According To CMD Selection Function
void FunctionSwitch(unsigned short cmd)
{
	switch(cmd)
	{
		//1000
		case SERVER_HEARTBEAT:
			cmdFunc = HwaetBeat;
			break;			
		//1、SELF_CHECK,	1100-1199
		//1100
		case SELF_STATE:break;
		//1101
		case SELF_TEMPERATURE:break;
		//1102
		case SELF_POWER:break;
		//1103
		case SELF_POSE:break;
		//2、SENSOR,	1200-1299
		//1200
		case SENSOR_SELF_TEMPERATURE:break;
		//1201
		case SENSOR_SELF_BLOODPRESSURE:break;
		//1202
		case SENSOR_SELF_BLOODSUGAR:break;
		//1203
		case SENSOR_SELF_HEIGHT:break;
		//1204
		case SENSOR_SELF_WEIGHT:break;
		//1205
		case SENSOR_ENV_TEMPERATURE:break;
		//1206
		case SENSOR_ENV_GAS:break;
		//1207
		case SENSOR_ENV_PM25:break;
		//1208
		case SENSOR_ENV_PRESSURE:break;
		//1209
		case SENSOR_ENV_OBSTACLE:break;	
		//3、ACTION,1300-1699
		//3.1、ACTION_HEAD
		//1300
		case ACTION_HEAD_RISE:break;
		//1301
		case ACTION_HEAD_SWIVEL:break;	
		//3.2、ACTION_WAIST
		//3.3、ACTION_KNEE
		//3.4、ACTION_ARM
		//3.5、ACTION_MOVE
		//1302
		case ACTION_MOVE_STOP:break;
		//1303
		case ACTION_MOVE_ALONG:break;	
		//3.6、ACTION_MODE
		//1307
		case ACTION_MODE_FREE:break;
		//1308
		case ACTION_MODE_CONTROLLED:break;
		//4、SOUND_CMD,1700-1799
		//1700
		case SOUND_SEARCH_SOUND:break;
		//1701
		case SOUND_WAKE_UP:break;		
		//5、LED，1800-1899
		//1800
		case LED_LIGHTING:break;
		//1801
		case LED_EXPRESSION:break;		
		//6、WARRING，9900-9999
		//9900
		case WARRING_POWER:break;
		//9901
		case WARRING_FALL:break;
		//9902
		case WARRING_OBSTACLE:break;
		//unkonw type of cmd
		default :cmdFunc = NULL;
	}
}

void GetDataAes(int flag);
void GetKeyExpansion();

int Pack(unsigned char* dataSrc,unsigned char dataLen,unsigned short cmdType)
{
	if(dataSrc == NULL)
	{
		return -1;
	}
	memset(packageSend,0,PACKAGE_LEN);
	
	unsigned char	* head	=	&packageSend[PACKAGE_HEAD_POS];
	unsigned short	* cmd	=	(unsigned short	*)&packageSend[PACKAGE_CMD_POS];
	unsigned char	* len	=	&packageSend[PACKAGE_DATALEN_POS];
	unsigned short	* crc	=	(unsigned short	*)&packageSend[PACKAGE_CRCCODE_POS];
	unsigned char	* data	=	&packageSend[PACKAGE_DATA_POS];

	*head	=	0xf1;
	*cmd	=	cmdType;
	*len	=	dataLen;
	memcpy(data,dataSrc,dataLen);
	//crc
	unsigned char crcData[AES_DATA_LEN];
	memcpy(crcData,packageSend+1,AES_DATA_LEN);
	*crc	=	CRC16Bytes(crcData,AES_DATA_LEN);
	//aes
	GetKeyExpansion();
	GetDataAes(0);

	return 0;
}

void Unpack()
{
	unsigned char	* head	=	&packageRecv[PACKAGE_HEAD_POS];
	unsigned short	* cmd	=	(unsigned short	*)&packageRecv[PACKAGE_CMD_POS];
	unsigned char	* len	=	&packageRecv[PACKAGE_DATALEN_POS];
	unsigned short	* crc	=	(unsigned short	*)&packageRecv[PACKAGE_CRCCODE_POS];
	//aes
	GetKeyExpansion();
	GetDataAes(1);
	//crc
	unsigned char crcData[AES_DATA_LEN];
	memcpy(crcData,packageRecv+1,AES_DATA_LEN);	
	unsigned short crcCheck = CRC16Bytes(crcData,AES_DATA_LEN);
	if(crcCheck != *crc)
	{
		ERROR("CRC Check Error");
		return ;
	}
	//get data
	unsigned char data[PACKAGE_DATA_LEN];
	memset(data,0,PACKAGE_DATA_LEN);
	memcpy(data,&packageRecv[PACKAGE_DATA_POS],*len);
	//fun switch
	FunctionSwitch(*cmd);	
	if(cmdFunc != NULL)
	{
		(*cmdFunc)(data,*len);
	}
	else
	{
		ERROR("Unkonw Type Of Cmd");
	}
}

void FindPackageHead()
{
	int bufLen = readBuff.length();
	size_t i = readBuff.find(PACKAGE_HEAD);
	while(i != string::npos)
	{
		if((bufLen-i)>PACKAGE_LEN)
		{
			//unpack
			memcpy(packageRecv,readBuff.c_str(),PACKAGE_LEN);
			Unpack();
			i++;
		}
		else
			break ;
		i = readBuff.find(PACKAGE_HEAD,i);
	}
	if(i == string::npos)
	{
		//set empty
		string empty;
		readBuff = empty;
	}	
	else
	{
		//substr
		string subReadBuf = readBuff.substr(i,bufLen-i);
		readBuff = subReadBuf;	
	}
}

int ReadNonBlock(int fd,char* recvBuf,int recvLen)
{
	if((-1 == fd)||(NULL == recvBuf)||(recvLen <= 0))
	{
		return -1;
	}
	fd_set fds;
	struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec =1000;
	int returnValue = 0;
	FD_ZERO(&fds);
	FD_SET(fd,&fds);
	int maxfd =(fd)+1;
	int res = select(maxfd,&fds,NULL,NULL,&timeout);
    switch(res)
    {
		case -1:
			returnValue = -1;break;
        case 0:
        	returnValue = 0;break;
        default:
        	if(FD_ISSET(fd,&fds))
        	{
				returnValue = read(fd,recvBuf,recvLen);
        	}
    }
    FD_CLR(fd,&fds);
	return returnValue;
}

void* readThread(void* para)
{
	sleep(1);
	DEBUG("Read Thread Start Success");
	char recvBuffer[READ_BUFF_LEN];
	while(1)
	{
		memset(recvBuffer,0,READ_BUFF_LEN);
		int res = ReadNonBlock(fd,recvBuffer,READ_BUFF_LEN);
		if(res > 0)
		{
			DEBUG("recvBuffer=%s len=%d\n",recvBuffer,res);
			//make sure that the string does not contain characters '\0'
			for(int i=0;i<res;i++)
			{
				if(recvBuffer[i]=='\0')
					recvBuffer[i] = 0xff;
			}
			readBuff += recvBuffer;//merge
			FindPackageHead();		
    		continue;
		}
		else if(res < 0)
		{
			ERROR("Read Failed");
		}
		else
		{
			DEBUG("Data Len Is Zero");
		}
		sleep(READ_INTERVAL_TIME);
	}
	free(recvBuffer);
}

void SendPackage()
{
	if(-1 == fd)
	{
		return ;
	}
	int res = write(fd,packageSend,READ_BUFF_LEN);
	if(res == READ_BUFF_LEN)
	{
		DEBUG("Send Package Success");
	}
	else
	{
		ERROR("Send Package Error");
	}
}

void* writeThread(void* para)
{
	DEBUG("Write Thread Start Success");
	while(1)
	{
		sleep(WRITE_INTERVAL_TIME);
	}
}

void GetKeyExpansion()
{
	string aesFilePath = AES_KEY_DIR;
	string aesKeyExpansion;
	if(access(aesFilePath.c_str(),0) == 0)
	{
		ifstream in(aesFilePath.c_str());		
		getline(in,aesKeyExpansion);
		int aesKeyLen = aesKeyExpansion.length();
		if(aesKeyLen == 16 || aesKeyLen == 24 || aesKeyLen == 32)
		{
			DEBUG("Use Key File");			
			key = (uint8_t*)malloc(sizeof(aesKeyLen*sizeof(uint8_t)));
			memcpy(key,aesKeyExpansion.c_str(),keyLen);
			switch (keyLen)
			{
				default:
				case 16: Nk = 4; Nr = 10; break;
				case 24: Nk = 6; Nr = 12; break;
				case 32: Nk = 8; Nr = 14; break;
			}
			return ;
		}
	}
	//Aes File Doesn't Exist Or Aes Key Len Error
	//Use Default Key
	DEBUG("Use Default Key");
	uint8_t key1[] = {
		0x00, 0x01, 0x02, 0x03,
		0x04, 0x05, 0x06, 0x07,
		0x08, 0x09, 0x0a, 0x0b,
		0x0c, 0x0d, 0x0e, 0x0f,
		0x10, 0x11, 0x12, 0x13,
		0x14, 0x15, 0x16, 0x17,
		0x18, 0x19, 0x1a, 0x1b,
		0x1c, 0x1d, 0x1e, 0x1f
	};

	key = (uint8_t*)malloc(sizeof(key1));
	for(int i=0;i<sizeof(key1);i++)
	{
		key[i] = key1[i];
	}
	keyLen = 32;
	Nk = 8; Nr = 14;
}

//@para flag 0 Encryption,1 Decrypt
void GetDataAes(int flag)
{
	w = (uint8_t*)malloc(Nb*(Nr+1)*4);
	key_expansion(key, w);

	if(1 != flag && 0 != flag)
	{
		ERROR("Unkonw Type Of Choose");
	}
	DEBUG("Get Data Aes Flag = %d",flag);
	if(0 == flag)
	{	
		memcpy(aesData,packageSend+1,AES_DATA_LEN);
	}
	else if(1 == flag)
	{
		memcpy(aesData,packageRecv+1,AES_DATA_LEN);
	}

	int n = AES_DATA_LEN/16;
	if(0 != AES_DATA_LEN%16)
	{
		n+=1;
	}

	for(int i=0;i<n;i++)
	{
		uint8_t in[16] = {};
		uint8_t out[16] = {};
		int cpyLen;
		if(i==n-1)
		{
			cpyLen = AES_DATA_LEN%16;
		}
		else
		{
			cpyLen = 16;
		}
		memcpy(in,aesData+i*16,cpyLen);
		if(0 == flag)
		{
			cipher(in,out,w);
		}		
		else if(1 == flag)
		{
			inv_cipher(in,out,w);
		}
		memcpy(aesData+i*16,out,cpyLen);
	}
	if(0 == flag)
	{	
		memcpy(packageSend+1,aesData,AES_DATA_LEN);
	}
	else if(1 == flag)
	{
		memcpy(packageRecv+1,aesData,AES_DATA_LEN);
	}
}


//Function Tests 
void AesTest()
{		
	unsigned char testData[] = "abcdefghijklmn";
	Pack(testData,16,1000);
	memcpy(packageRecv,packageSend,PACKAGE_LEN);
	Unpack();
}

void RecieveTest()
{
	while(1)
	{
		unsigned char dataSend[] = "abcdefghijklmn";
		Pack(dataSend,16,1000);
		SendPackage();
		sleep(5);
	}
}

int main()
{
	char *dev  = "/dev/ttyUSB0";
	fd = OpenDev(dev);
	setSpeed(fd,115200);
    	if(setParity(fd,8,1,'N') == FALSE)  
	{
		ERROR("Set Parity Error");
	}

	pthread_t readThreadId,writeThreadId;
	int result=pthread_create(&readThreadId,NULL,readThread,NULL);
	if(result != 0)
	{
		ERROR("Create Read Thread Failed");
	}
	while(1);

	return 0;
}
