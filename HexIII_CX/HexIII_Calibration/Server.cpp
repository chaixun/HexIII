#include "Server.h"

static int LastCMD=-1;

Aris::Core::CONN ControlSystem;

Aris::Core::CONN VisionSystem;

// CONN call back functions
int On_CS_ConnectionReceived(Aris::Core::CONN *pConn,const char* addr,int port)
{
    Aris::Core::MSG msg;

    msg.SetMsgID(CS_Connected);
    msg.SetLength(sizeof(port));
    msg.Copy(&port,sizeof(port));
    msg.CopyMore(addr,strlen(addr));
    PostMsg(msg);
    return 0;
}

int On_CS_DataReceived(Aris::Core::CONN *pConn,Aris::Core::MSG &data)
{
    int cmd=data.GetMsgID()+100;
    Aris::Core::MSG CMD=CS_CMD_Received;
    CMD.SetLength(sizeof(int));
    bool IsCMDExecutable=true;
    CMD.Copy(&cmd,sizeof(int));
    cout<<"received CMD is"<<cmd<<endl;
    PostMsg(CMD);
    LastCMD=data.GetMsgID();
    return 0;
}

int On_CS_ConnectionLost(Aris::Core::CONN *pConn)
{
    PostMsg(Aris::Core::MSG(CS_Lost));
    return 0;
}

//MSG call back functions
int On_CS_Connected(Aris::Core::MSG &msg)
{
    cout<<"Received Connection from Control System:"<<endl;
    cout<<"   Remote IP is: "<<msg.GetDataAddress()+sizeof(int)<<endl;
    cout<<"   Port is     : "<<*((int*)msg.GetDataAddress()) << endl << endl;

    Aris::Core::MSG data(0,0);
    ControlSystem.SendData(data);
    return 0;
}

int On_CS_CMD_Received(Aris::Core::MSG &msg)
{
    int Command_Gait;
    msg.Paste(&Command_Gait,sizeof(int));
    Aris::Core::MSG Gait=Command_Gait;
    cout<<"Gait Needed is: "<<Gait.GetMsgID()<<endl;
    PostMsg(Gait);
    Aris::Core::MSG data(0,0);
    ControlSystem.SendData(data);
    return 0;
}

int On_CS_Lost(Aris::Core::MSG &msg)
{
    cout << "Control system connection lost" << endl;
    ControlSystem.StartServer("5690");
    return 0;
}


int On_VS_ConnectionReceived(Aris::Core::CONN *pConn, const char* addr,int port)
{
    Aris::Core::MSG msg;
    msg.SetMsgID(VS_Connected);
    msg.SetLength(sizeof(port));
    msg.Copy(&port,sizeof(port));
    msg.CopyMore(addr,strlen(addr));
    PostMsg(msg);
    return 0;
}

int On_VS_DataReceived(Aris::Core::CONN *pConn, Aris::Core::MSG &data)
{

    cout<<"Calibration Singnal Recieved"<<endl;
    int num;
    memcpy(&num,data.GetDataAddress(),data.GetLength());
    //stop num >= 54
    if (num >= 62 )
    {
        cout<<"Calibration finished"<<endl;
        CGait::IsAutoCalibrateRegistered = false;
    }
    else
    {
        ifstream inf;
        stringstream filenum;
        string filename;
        filenum<<(num-1);
        filename = "./calibration_txt/"+filenum.str() + "F.txt";
        inf.open(filename);
        int Line_Num;
        double temp;
        for(int i=0;i<GAIT_AUTOCALIB_LEN/2;i++)
        {
            inf>>Line_Num;
            for(int j=0;j<GAIT_WIDTH;j++)
            {
                inf>>temp;
                CGait::GaitAutoCalib[i][j]=-(int)temp;
            }
        }

        inf.close();
        filenum.str("");
        filename.clear();

        filenum<<num;
        filename = "./calibration_txt/"+filenum.str()+".txt";
        inf.open(filename);
        for(int i=GAIT_AUTOCALIB_LEN/2;i<GAIT_AUTOCALIB_LEN;i++)
        {
            inf>>Line_Num;
            for(int j=0;j<GAIT_WIDTH;j++)
            {
                inf>>temp;
                CGait::GaitAutoCalib[i][j]=-(int)temp;
            }
        }
        inf.close();

        cout<<"Calibration "<<num<<" begin"<<endl;
        cout<<CGait::GaitAutoCalib[3999][0]<<endl;
        EGAIT cmd=GAIT_AUTO_CALIBRATION;
        Aris::Core::MSG data;
        data.SetLength(sizeof(cmd));
        data.Copy(&cmd,sizeof(cmd));
        cout<<"Send Message to CS"<<endl;
        cs.NRT_SendData(data);
    }
    return 0;
}

int On_VS_ConnectionLost(Aris::Core::CONN *pConn)
{
    PostMsg(Aris::Core::MSG(VS_Lost));
    return 0;
}

int On_VS_Connected(Aris::Core::MSG &msg)
{
    cout<<"Received Connection from Vision System:"<<endl;
    cout<<"   Remote IP is: "<<msg.GetDataAddress()+sizeof(int)<<endl;
    cout<<"   Port is     : "<<*((int*)msg.GetDataAddress()) << endl << endl;

    return 0;

}

int On_VS_Capture(Aris::Core::MSG &msg)
{
    MSG data(0,0);
    VisionSystem.SendData(data);
    return 0;
}

int On_VS_Lost(Aris::Core::MSG &msg)
{
    cout << "Visual system connection lost" << endl;
    VisionSystem.StartServer("5691");
    return 0;
}


