#include "Server.h"

static int LastCMD=-1;

Aris::Core::CONN ControlSystem;
Aris::Core::CONN VisionSystem;

double Gait_Calculated_From_Map[GAIT_ADAPTIVEWALK_LEN][GAIT_WIDTH];

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
    CMD.Copy(&cmd,sizeof(int));
    cout<<"received CMD is "<<cmd<<endl;
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
    cout<<"receive data from visual system"<<endl;

    if(data.GetLength() == 4)
    {
        int controlcmd;
        memcpy(&controlcmd,data.GetDataAddress(),data.GetLength());

        switch (controlcmd)
        {
        case 1:
        {
            EGAIT cmd=GAIT_MOVE;
            Aris::Core::MSG data;
            data.SetLength(sizeof(cmd));
            data.Copy(&cmd,sizeof(cmd));
            cout<<"Send Message to CS"<<endl;
            cs.NRT_SendData(data);
        }
            break;
        case 2:
        {
            EGAIT cmd=GAIT_MOVE_BACK;
            Aris::Core::MSG data;
            data.SetLength(sizeof(cmd));
            data.Copy(&cmd,sizeof(cmd));
            cout<<"Send Message to CS"<<endl;
            cs.NRT_SendData(data);
        }
            break;
        case 3:
        {
            EGAIT cmd=GAIT_TURN_LEFT;
            Aris::Core::MSG data;
            data.SetLength(sizeof(cmd));
            data.Copy(&cmd,sizeof(cmd));
            cout<<"Send Message to CS"<<endl;
            cs.NRT_SendData(data);
        }
            break;
        case 4:
        {
            EGAIT cmd=GAIT_TURN_RIGHT;
            Aris::Core::MSG data;
            data.SetLength(sizeof(cmd));
            data.Copy(&cmd,sizeof(cmd));
            cout<<"Send Message to CS"<<endl;
            cs.NRT_SendData(data);
        }
            break;
        default:
            break;
        }

    }
    else
    {

        double *map = new double [data.GetLength()/sizeof(double)];
        memcpy(map, data.GetDataAddress(), data.GetLength()  );

        static double currentH[6],nextH[6];

        memcpy(nextH,map,sizeof(nextH));

        if(currentH[0] == 0&&currentH[1] == 0&&currentH[2] == 0&&currentH[3] == 0&&currentH[4] == 0&&currentH[5] == 0&&
                nextH[0] == 0&&nextH[1] == 0&&nextH[2] == 0&&nextH[3] == 0&&nextH[4] == 0&&nextH[5] == 0)
        {
            delete[] map;
            CGait::IsWalkAdaptiveRegistered = false;
            EGAIT cmd=GAIT_END_DISCOVER;
            Aris::Core::MSG controldata;
            controldata.SetLength(sizeof(cmd));
            controldata.Copy(&cmd,sizeof(cmd));
            cs.NRT_SendData(controldata);
        }
        else
        {

            HexIII.MoveWithKinect(currentH,nextH,*Gait_Calculated_From_Map);

            memcpy(currentH,nextH,sizeof(nextH));

            delete[] map;

            for(int j=0;j<GAIT_ADAPTIVEWALK_LEN;j++)
            {
                for(int i=0;i<GAIT_WIDTH;i++)
                {
                    CGait::GaitAdaptiveWalk[j][i]=-(int)(Gait_Calculated_From_Map[j][i]);
                }
            }

            EGAIT cmd=GAIT_WALK_ADAPTIVE;
            Aris::Core::MSG controldata;
            controldata.SetLength(sizeof(cmd));
            controldata.Copy(&cmd,sizeof(cmd));
            cs.NRT_SendData(controldata);
        }
    }
    return 0;
}

int On_VS_ConnectionLost(Aris::Core::CONN *pConn)
{
    CGait::IsWalkAdaptiveRegistered = false;
    CGait::IsWalkAvoidRegistered = false;
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
    if(CGait::IsWalkAdaptiveRegistered == true)
    {
        //Vision_Msg
        int vis_msg =60;// Walk_Adaptive;
        Aris::Core::MSG data = 60;
        //data.SetLength(sizeof(vis_msg));
        data.Copy(&vis_msg, sizeof(vis_msg));
        VisionSystem.SendData(data);
    }
    else if(CGait::IsWalkAvoidRegistered == true)
    {
        Vision_Msg vis_msg = Walk_Avoid;
        Aris::Core::MSG data;
        data.SetLength(sizeof(vis_msg));
        data.Copy(&vis_msg, sizeof(vis_msg));
        VisionSystem.SendData(data);
    }

    return 0;

}

int On_VS_Lost(Aris::Core::MSG &msg)
{
    cout << "Visual system connection lost" << endl;
    VisionSystem.StartServer("5691");
    return 0;
}


