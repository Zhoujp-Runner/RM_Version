#include "./include/serialport.h"

SerialPort::SerialPort()
{
    /**
     *@brief   初始化数据
    *@param  fd       类型  int  打开的串口文件句柄
    *@param  speed    类型  int  波特率

    *@param  databits 类型  int  数据位   取值 为 7 或者8

    *@param  stopbits 类型  int  停止位   取值为 1 或者2
    *@param  parity   类型  int  效验类型 取值为N,E,O,S
    *@param  portchar 类型  char* 串口路径
    */
	fd = open(UART_DEVICE, O_RDWR | O_NOCTTY | O_NDELAY);
    if(fd == -1)
    {
        fd = open(UART_DEVICE1, O_RDWR | O_NOCTTY | O_NDELAY);
    }

    speed = BAUDRATE;
    databits = 8;
    stopbits = 1;
	parity = 'N';
}

SerialPort::SerialPort(char *portpath, char *portpath1)
{
	fd = open(portpath, O_RDWR | O_NOCTTY | O_NDELAY);
    if(fd == -1)
    {
        fd = open(portpath1, O_RDWR | O_NOCTTY | O_NDELAY);
    }

    speed = BAUDRATE;
    databits = 8;
    stopbits = 1;
	parity = 'N';
}

vector<Device> SerialPort::ListSerialport()
{
    for(auto &ptr : SerialPath)
    {
        string tty_path;
        string dev_path;
        tty_path = "sys/class/tty/" + ptr;
        for(int i = 0; i < 10; i++)
        {   //搜索串口号尾数0～9
            dev_path = tty_path + to_string(i);
            
            //判断该串口是否可获得
            if(access(dev_path.c_str(),F_OK) != -1)
            { 
                Device dev;
                dev.Id = ptr;

                //将该串口号进行保存
                devices.push_back(dev);
            }
        }
    }
    return devices;
}

Device SerialPort::GetPortId(vector<Device> dev)
{
    for(auto &ports : dev)
    {
        if(ports.Id == UART_DEVICE || ports.Id == UART_DEVICE1)
        return ports;
    }
    return Device();
}

bool SerialPort::initSerialPort()
{
    /**
     *@brief   初始化串口,打开串口
    */
    Device ttyId = GetPortId(ListSerialport());
    if(ttyId.Id == "")
    {
        return false;
    }

    const string ttyIdPath = "/dev/" + ttyId.Id;  

    close(last_fd);
    fd = open(ttyIdPath.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1)
	{
        perror(ttyIdPath.c_str());
        return false;
    }

	std::cout << "Opening..." << std::endl;

    speed = BAUDRATE;
    databits = 8;
    stopbits = 1;
	parity = 'N';
    
    set_Brate();
	if (set_Bit() == FALSE)
	{
        printf("Set Parity Error\n");
		exit(0);
    }
    printf("Open successed\n");

    last_fd = fd;

    return true;
}

// bool SerialPort::getSendData()
// {
    
// }

bool SerialPort::Check_Port_State()
{
    system("sudo chmod 777 /dev/ttyACM0");
    fd = open(UART_DEVICE, O_RDWR | O_NOCTTY | O_NDELAY);
    if(fd == -1)
    {
        system("sudo chmod 777 /dev/ttyACM1");
        fd = open(UART_DEVICE1, O_RDWR | O_NOCTTY | O_NDELAY);
    }

    if(fd == -1)
    {
        return false;
    }
    return true;
}

bool SerialPort::get_Mode(int &mode, int &sentry_mode, int &base_mode)
{
    /**
     *@brief   获取模式命令
    */
    int bytes;
    char *name = ttyname(fd);
    if (name = NULL)printf("tty is null\n");
    //if (name != NULL)printf("device:%s\n",name);
    int result = ioctl(fd, FIONREAD, &bytes);
    if (result == -1) return false;

    if (bytes == 0)
    {
    //    cout << "缓冲区为空" << endl;
        return true;
    }

    bytes = read(fd, rdata, 22);

    if (rdata[0] == 0xA5 && Verify_CRC8_Check_Sum(rdata, 3))
    {
        //判断针头和CRC校验是否正确
        mode  = (int)rdata[1]; //通过此数据控制线程的开启	0关闭自瞄 1开启自瞄 2小能量机关 3大能量机关
        // printf("接收到的指令:%d\r\n", mode);
	//----------No use---------//    
        sentry_mode  = (int)rdata[15];
        // printf("Is in sentry mode ? :%d\r\n", sentry_mode);
        base_mode  = (int)rdata[16];
        // printf("Is in base mode ? :%d\r\n", base_mode);

    }
    return true;
}


/**
 *@brief   设置波特率
 */
void SerialPort::set_Brate()
{
    int speed_arr[] = {B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300,
					   B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300,
					  };
    int name_arr[] = {115200, 38400, 19200, 9600, 4800, 2400, 1200,  300,
					  115200, 38400, 19200, 9600, 4800, 2400, 1200,  300,
					 };
    int   i;
    int   status;
    struct termios   Opt;
    tcgetattr(fd, &Opt);

	for (i = 0;  i < sizeof(speed_arr) / sizeof(int);  i++)
	{
		if (speed == name_arr[i])
		{
			tcflush(fd, TCIOFLUSH);//清空缓冲区的内容
			cfsetispeed(&Opt, speed_arr[i]);//设置接受和发送的波特率
			cfsetospeed(&Opt, speed_arr[i]);
            status = tcsetattr(fd, TCSANOW, &Opt); //使设置立即生效

			if (status != 0)
			{
                perror("tcsetattr fd1");
                return;
            }

			tcflush(fd, TCIOFLUSH);

        }
    }
}

/**
 *@brief   设置串口数据位，停止位和效验位
 */
int SerialPort::set_Bit()
{
    struct termios termios_p;

	if (tcgetattr(fd, &termios_p)  !=  0)
	{
        perror("SetupSerial 1");
		return (FALSE);
    }

    termios_p.c_cflag |= (CLOCAL | CREAD);  //接受数据
	termios_p.c_cflag &= ~CSIZE;//设置数据位数

    switch (databits)
    {
	case 7:
		termios_p.c_cflag |= CS7;
		break;

	case 8:
		termios_p.c_cflag |= CS8;
		break;

	default:
		fprintf(stderr, "Unsupported data size\n");
		return (FALSE);
    }

	//设置奇偶校验位double
    switch (parity)
    {
	case 'n':
	case 'N':
		termios_p.c_cflag &= ~PARENB;   /* Clear parity enable */
		termios_p.c_iflag &= ~INPCK;     /* Enable parity checking */
		break;

	case 'o':
	case 'O':
		termios_p.c_cflag |= (PARODD | PARENB); /* 设置为奇效验*/
		termios_p.c_iflag |= INPCK;             /* Disnable parity checking */
		break;

	case 'e':
	case 'E':
		termios_p.c_cflag |= PARENB;     /* Enable parity */
		termios_p.c_cflag &= ~PARODD;   /* 转换为偶效验*/
		termios_p.c_iflag |= INPCK;       /* Disnable parity checking */
		break;

	case 'S':
	case 's':  /*as no parity*/
		termios_p.c_cflag &= ~PARENB;
		termios_p.c_cflag &= ~CSTOPB;
		break;

	default:
		fprintf(stderr, "Unsupported parity\n");
		return (FALSE);

    }

    /* 设置停止位*/
    switch (stopbits)
    {
	case 1:
		termios_p.c_cflag &= ~CSTOPB;
		break;

	case 2:
		termios_p.c_cflag |= CSTOPB;
		break;

	default:
		fprintf(stderr, "Unsupported stop bits\n");
		return (FALSE);

    }

    /* Set input parity option */
    if (parity != 'n')
        termios_p.c_iflag |= INPCK;

	tcflush(fd, TCIFLUSH); //清除输入缓存区
    termios_p.c_cc[VTIME] = 150; /* 设置超时15 seconds*/
    termios_p.c_cc[VMIN] = 0;  //最小接收字符
    termios_p.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /*Input原始输入*/
	termios_p.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
    termios_p.c_iflag &= ~(ICRNL | IGNCR);
    termios_p.c_oflag &= ~OPOST;   /*Output禁用输出处理*/

	if (tcsetattr(fd, TCSANOW, &termios_p) != 0) /* Update the options and do it NOW */
    {
        perror("SetupSerial 3");
        return (FALSE);
    }

    return (TRUE);
}


////
//int SerialPort::set_disp_mode(int option)
//{
//   int err;
//   struct termios term;
//   if(tcgetattr(fd,&term)==-1){
//     perror("Cannot get the attribution of the terminal");
//     return 1;
//   }
//   if(option)
//        term.c_lflag|=ECHOFLAGS;   //(ECHO | ECHOE | ECHOK | ECHONL)
//   else
//        term.c_lflag &=~ECHOFLAGS;
//   err=tcsetattr(fd,TCSAFLUSH,&term);
//   if(err==-1 && err==EINTR){
//        perror("Cannot set the attribution of the terminal");
//        return 1;
//   }
//   return 0;
//}


////////////////////////////////////////////////////////////
/**
 *@brief   转换数据并发送

 *@param  p        类型  int  x坐标
 *@param  yaw      类型  int  y坐标
 *@param  dis      类型  int  dis  距离
 *@param  unsign   类型  int  空白（可添加数据）
 */
void SerialPort::TransformDataFirst(int Xpos, int Ypos, int dis)
{

    Tdata[0] = 0xA5;
    Tdata[1] = CmdID0;   //command

	Append_CRC8_Check_Sum(Tdata, 3); //CRC8 校验

	for (int i = 0; i < 4; i++)
	{
		Tdata[3 + i] = Xpos % 10;
		Xpos = (Xpos - Xpos % 10) / 10;
    }

	for (int i = 0; i < 4; i++)
	{
		Tdata[7 + i] = Ypos % 10;
		Ypos = (Ypos - Ypos % 10) / 10;
    }

	for (int i = 0; i < 4; i++)
	{
		Tdata[11 + i] = dis % 10;
		dis = (dis - dis % 10) / 10;
    }

	Tdata[15] = 0;//通过此端口向下位机发送是否识别到敌方装甲板或能量机关
    Tdata[16] = 0;
    Tdata[17] = 0;
    Tdata[18] = 0;
    Tdata[19] = 0;

	Append_CRC16_Check_Sum(Tdata, 22); //CRC16校验
}

/**
 *@brief   转换数据并发送
 *@param   data   类型  VisionData(union)  包含pitch,yaw,distance
 *@param   flag   类型  char   用于判断是否瞄准目标，0代表没有，1代表已经瞄准
 */
bool SerialPort::TransformData(const VisionData &data)
{

    Tdata[0] = 0xA5;

    //识别红色
    Tdata[1] = CmdID1;
	Append_CRC8_Check_Sum(Tdata, 3);

    Tdata[3] = data.pitch_angle.c[0];
    Tdata[4] = data.pitch_angle.c[1];
    Tdata[5] = data.pitch_angle.c[2];
    Tdata[6] = data.pitch_angle.c[3];

    Tdata[7] = data.yaw_angle.c[0];
    Tdata[8] = data.yaw_angle.c[1];
    Tdata[9] = data.yaw_angle.c[2];
    Tdata[10] = data.yaw_angle.c[3];

    Tdata[11] = data.dis.c[0];
    Tdata[12] = data.dis.c[1];
    Tdata[13] = data.dis.c[2];
    Tdata[14] = data.dis.c[3];

    Tdata[15] = data.ismiddle;
	Tdata[16] = data.isFindTarget;

    Tdata[17] = data.isfindDafu;
    Tdata[18] = 0x00;
    Tdata[19] = data.nearFace;
    Tdata[20] = 0;
    Tdata[21] = 0;
	Append_CRC16_Check_Sum(Tdata, 22);
    return true;
}

//发送数据函数
void SerialPort::send()
{
	ssize_t value = write(fd, Tdata, 22);
}

//关闭通讯协议接口
void SerialPort::closePort()
{
    close(fd);
}
