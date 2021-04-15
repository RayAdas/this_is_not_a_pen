#include "pmbcbspkeeper.h"

UartKeeper_Infantry::UartKeeper_Infantry(SerialPort* uart1)
{
    this->Uart1 = uart1;
    *(this->aim_mode_buffer_.Back) = manualMode;
    this->aim_mode_buffer_.writeOver();
    *(this->aim_mode_buffer_.Back) = manualMode;
    this->aim_mode_buffer_.writeOver();

    *(this->enemy_color_buffer_.Back) = teamColor_red;
    this->enemy_color_buffer_.writeOver();
    *(this->enemy_color_buffer_.Back) = teamColor_red;
    this->enemy_color_buffer_.writeOver();
    sent_length_ = 13;
    this->S_DATA[0] = 0xAA;
    this->S_DATA[1] = 0xAA;
    this->S_DATA[12] = 0XBB;
}

void UartKeeper_Infantry::boot(void)
{
    keeper_thread_ = std::thread(&UartKeeper_Infantry::KeeperCycle,this);
}

void UartKeeper_Infantry::KeeperCycle()
{
    while(true)
    {
        if(Uart1->get_data())
        {
            switch(Uart1->receice_data_[2])
            {
            case 0x01://普通自瞄申请
                *(aim_mode_buffer_.Back) = robotMode;
                aim_mode_buffer_.writeOver();
                //cout<<"AimMod:robotMod"<<endl;
                break;
            case 0x02://大符自瞄申请
                *(aim_mode_buffer_.Back) = buffMode;
                aim_mode_buffer_.writeOver();
                //cout<<"AimMod:buffMod"<<endl;
                break;
            case 0x03://禁止自瞄申请
                *(aim_mode_buffer_.Back) = manualMode;
                aim_mode_buffer_.writeOver();
                //cout<<"AimMod:manualMode"<<endl;
                break;
            case 0x04:
                axis_data_buffer_.Back->ProjectileVel = *((short*)&(Uart1->receice_data_[3]));
                axis_data_buffer_.Back->RA_yaw = *((float*)&(Uart1->receice_data_[5]));
                axis_data_buffer_.Back->RA_pitch = *((float*)&(Uart1->receice_data_[9]));
                axis_data_buffer_.Back->AAV_yaw = *((float*)&(Uart1->receice_data_[13]));
                axis_data_buffer_.Back->AAV_pitch = *((float*)&(Uart1->receice_data_[17]));
                axis_data_buffer_.writeOver();
                break;
            case 0x07://角色反馈
                *(enemy_color_buffer_.Back) = Uart1->receice_data_[3] == 0x00 ? teamColor_red : teamColor_blue;
                enemy_color_buffer_.writeOver();
                break;
            default:return;
            }
        }
    }
}

void UartKeeper_Infantry::read(axisData* &AxisData,TeamColor* &EnemyColor,aimMod* &AimMod)
{
    AxisData = axis_data_buffer_.read();
    EnemyColor = enemy_color_buffer_.read();
    AimMod = aim_mode_buffer_.read();
}

void UartKeeper_Infantry::write()
{
    this->Uart1->send_data(this->S_DATA,this->sent_length_);
}
void UartKeeper_Infantry::set(const void* data,data_label label)
{
    switch(label)
    {
    case YawAngle:memcpy((void*)&(S_DATA[3]),data,4);break;
    case PitchAngle:memcpy((void*)&(S_DATA[7]),data,4);break;
    case AOrR:memcpy((void*)&(S_DATA[2]),data,1);break;
    case FirePermit:memcpy((void*)&(S_DATA[11]),data,1);break;
    }
}
//#############################以下为哨兵部分######################
UartKeeper_Guard::UartKeeper_Guard(SerialPort* uart1)
{
    this->Uart1 = uart1;

    *(this->aim_mode_buffer_.Back) = robotMode;
    this->aim_mode_buffer_.writeOver();
    *(this->aim_mode_buffer_.Back) = robotMode;
    this->aim_mode_buffer_.writeOver();

    *(this->enemy_color_buffer_.Back) = teamColor_red;
    this->enemy_color_buffer_.writeOver();
    *(this->enemy_color_buffer_.Back) = teamColor_red;
    this->enemy_color_buffer_.writeOver();
    sent_length_ = 12;
    this->S_DATA[0] = 0xBB;
    this->S_DATA[1] = 0x01;
    this->S_DATA[11] = 0XFF;
}
void UartKeeper_Guard::boot(void)
{
    keeper_thread_ = std::thread(&UartKeeper_Guard::KeeperCycle,this);
}
void UartKeeper_Guard::read(axisData* &AxisData,TeamColor* &EnemyColor,aimMod* &AimMod)
{
    AxisData = nullptr;
    EnemyColor = enemy_color_buffer_.read();
    AimMod = aim_mode_buffer_.read();
}
void UartKeeper_Guard::write()
{
    this->Uart1->send_data(this->S_DATA,this->sent_length_);
}
void UartKeeper_Guard::set(const void* data,data_label label)
{
    switch(label)
    {
    case YawAngle:memcpy((void*)&(S_DATA[2]),data,4);break;
    case PitchAngle:memcpy((void*)&(S_DATA[6]),data,4);break;
    case YunTaiMode:memcpy((void*)&(S_DATA[1]),data,1);break;
    case FirePermit:memcpy((void*)&(S_DATA[10]),data,1);break;
    }}
void UartKeeper_Guard::KeeperCycle()
{
    while(true)
    {
        if(Uart1->get_data())
        {
            switch(Uart1->receice_data_[2])
            {
            /*
            case 0x01://普通自瞄申请
                *(AimModBuffer.Back) = robotMode;
                AimModBuffer.writeOver();
                //cout<<"AimMod:robotMod"<<endl;
                break;
            case 0x02://大符自瞄申请
                *(AimModBuffer.Back) = buffMode;
                AimModBuffer.writeOver();
                //cout<<"AimMod:buffMod"<<endl;
                break;
            case 0x03://禁止自瞄申请
                *(AimModBuffer.Back) = manualMode;
                AimModBuffer.writeOver();
                //cout<<"AimMod:manualMode"<<endl;
                break;
            case 0x04:
                AxisDataBuffer.Back->ProjectileVel = *((short*)&(Uart1->receiceData[3]));
                AxisDataBuffer.Back->RA_yaw = *((float*)&(Uart1->receiceData[5]));
                AxisDataBuffer.Back->RA_pitch = *((float*)&(Uart1->receiceData[9]));
                AxisDataBuffer.Back->AAV_yaw = *((float*)&(Uart1->receiceData[13]));
                AxisDataBuffer.Back->AAV_pitch = *((float*)&(Uart1->receiceData[17]));
                AxisDataBuffer.writeOver();
                break;
            */
            case 0x07://角色反馈
                *(enemy_color_buffer_.Back) = Uart1->receice_data_[3] == 0x00 ? teamColor_red : teamColor_blue;
                enemy_color_buffer_.writeOver();
                break;
            default:return;
            }
        }
    }
}
