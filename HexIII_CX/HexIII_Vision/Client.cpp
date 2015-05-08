#include "Client.h"

CONN ControlSysClient;

//CONN callback functions
int OnConnDataReceived(Aris::Core::CONN *pConn, Aris::Core::MSG &data)
{
    Aris::Core::PostMsg(Aris::Core::MSG(ControlCommandNeeded));
    return 0;
}

int OnConnectLost(Aris::Core::CONN *pConn)
{
    PostMsg(Aris::Core::MSG(SystemLost));
    return 0;
}

//MSG callback functions
int OnControlCommandNeeded(Aris::Core::MSG &msg)
{
    int cmd;
    cout<<"Commands:"<<endl;
    cout<<"1.Eanble"<<endl<<"2.Disable"<<endl<<"3.Home"<<endl<<"4.Home to standstill"<<endl<<"5.Forward"<<endl<<"6.Back"
       <<endl<<"7.Walk Adaptive"<<endl<<"8.Turn left"<<endl<<"9.Turn right"<<endl<<"10.Begin Discover"<<endl<<"11.End Discover"
       <<endl<<"12.Walk Avoid"<<endl;
    cout<<"Please enter your command: ";
    cin>>cmd;
    while(cmd!=1&&cmd!=2&&cmd!=3&&cmd!=4&&cmd!=5&&cmd!=6&&cmd!=7&&cmd!=8&&cmd!=9&&cmd!=10&&cmd!=11&&cmd!=12)
    {
        cout<<"Not valid command ID,enter again : ";
        cin>>cmd;
    }
    cmd=cmd-1;
    MSG data;
    data.SetMsgID(cmd);
    ControlSysClient.SendData(data);
    return 0;
}

int OnSystemLost(Aris::Core::MSG &msg)
{
    cout<<"Control system lost"<<endl;
    return 0;
}
