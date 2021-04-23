#include "pmbcbspkeeper.h"

UartKeeper_Infantry::UartKeeper_Infantry(SerialPort* uart1)
{
    this->Uart1 = uart1;
    /*
    *(this->aim_mode_buffer_.Back) = manualMode;
    this->aim_mode_buffer_.writeOver();
    *(this->aim_mode_buffer_.Back) = manualMode;
    this->aim_mode_buffer_.writeOver();
    */

    *(this->aim_mode_buffer_.Back) = buffMode;
    this->aim_mode_buffer_.writeOver();
    *(this->aim_mode_buffer_.Back) = buffMode;
    this->aim_mode_buffer_.writeOver();

    /*
    *(this->aim_mode_buffer_.Back) = robotMode;
    this->aim_mode_buffer_.writeOver();
    *(this->aim_mode_buffer_.Back) = robotMode;
    this->aim_mode_buffer_.writeOver();
    */
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
        ssize_t readLength = Uart1->get_data(&(receive_data_buffer_[4096]),MAX_RECIEVE_LENGTH);
        if(readLength <= 0)//啥也没读到
        {continue;}

        int p = MAX_RECIEVE_LENGTH - left_receice_data_buffer_end;
        int p_end_plus_one = MAX_RECIEVE_LENGTH + readLength;

        //注意，下文中所有的到头了，其中的i += 的数值均可减一，不影响流程执行,因为下一个循环开始会i++
        for(int i = p;i<p_end_plus_one;i++)
        {//遍历找AA
            if(receive_data_buffer_[i] == 0xAA)
            {
                if(i + 1 < p_end_plus_one)
                {//没到头
                    if(receive_data_buffer_[i + 1] == 0xAA)
                    {//连着两个AA
                        if(i + 2 < p_end_plus_one)
                        {
                            switch(receive_data_buffer_[i + 2])
                            {
                            case 0x01://普通自瞄申请
                                if(i + 3 < p_end_plus_one)
                                {
                                    if(receive_data_buffer_[i + 3] == 0xBB)
                                    {
                                        *(aim_mode_buffer_.Back) = robotMode;
                                        aim_mode_buffer_.writeOver();
                                        cout<<"AimMod:robotMod"<<endl;
                                        i += 3;
                                    }
                                    else
                                    {continue;}
                                }
                                else
                                {//到头了
                                    memcpy(&(receive_data_buffer_[4093]),&(receive_data_buffer_[i]),3);
                                    left_receice_data_buffer_end = 3;
                                    i += 3;
                                }
                                break;
                            case 0x02://大符自瞄申请
                                if(i + 3 < p_end_plus_one)
                                {
                                    if(receive_data_buffer_[i + 3] == 0xBB)
                                    {
                                        *(aim_mode_buffer_.Back) = buffMode;
                                        aim_mode_buffer_.writeOver();
                                        cout<<"AimMod:buffMod"<<endl;
                                        i += 3;
                                    }
                                    else
                                    {continue;}
                                }
                                else
                                {
                                    memcpy(&(receive_data_buffer_[4093]),&(receive_data_buffer_[i]),3);
                                    left_receice_data_buffer_end = 3;
                                    i += 3;
                                }
                                break;
                            case 0x03://禁止自瞄申请
                                if(i + 3 < p_end_plus_one)
                                {
                                    if(receive_data_buffer_[i + 3] == 0xBB)
                                    {
                                        *(aim_mode_buffer_.Back) = manualMode;
                                        aim_mode_buffer_.writeOver();
                                        cout<<"AimMod:manualMode"<<endl;
                                        i += 3;
                                    }
                                    else
                                    {continue;}
                                }
                                else
                                {
                                    memcpy(&(receive_data_buffer_[4093]),&(receive_data_buffer_[i]),3);
                                    left_receice_data_buffer_end = 3;
                                    i += 3;
                                }
                                break;
                            case 0x04:
                                if(i + 21 < p_end_plus_one)
                                {
                                    if(receive_data_buffer_[i + 21] == 0xBB)
                                    {
                                        axis_data_buffer_.Back->ProjectileVel = *((short*)&(receive_data_buffer_[i + 3]));
                                        axis_data_buffer_.Back->RA_yaw = *((float*)&(receive_data_buffer_[i + 5]));
                                        axis_data_buffer_.Back->RA_pitch = *((float*)&(receive_data_buffer_[i + 9]));
                                        axis_data_buffer_.Back->AAV_yaw = *((float*)&(receive_data_buffer_[i + 13]));
                                        axis_data_buffer_.Back->AAV_pitch = *((float*)&(receive_data_buffer_[i + 17]));
                                        axis_data_buffer_.writeOver();
                                        i += 21;
                                    }
                                    else
                                    {continue;}
                                }
                                else
                                {
                                    memcpy(&(receive_data_buffer_[4096 - (p_end_plus_one - i)]),&(receive_data_buffer_[i]),(p_end_plus_one - i));
                                    left_receice_data_buffer_end = p_end_plus_one - i;
                                    i += (p_end_plus_one - i);
                                }
                                break;
                            case 0x07://角色反馈
                                if(i + 4 < p_end_plus_one)
                                {
                                    if(receive_data_buffer_[i + 4] == 0xBB)
                                    {
                                        *(enemy_color_buffer_.Back) = receive_data_buffer_[i + 3] == 0x00 ? teamColor_red : teamColor_blue;
                                        enemy_color_buffer_.writeOver();
                                        i += 4;
                                    }
                                    else
                                    {continue;}
                                }
                                else
                                {
                                    memcpy(&(receive_data_buffer_[4096 - (p_end_plus_one - i)]),&(receive_data_buffer_[i]),(p_end_plus_one - i));
                                    left_receice_data_buffer_end = p_end_plus_one - i;
                                    i += (p_end_plus_one - i);
                                }
                                break;
                            default:return;
                            }
                        }
                        else
                        {//到头了
                            memcpy(&(receive_data_buffer_[4094]),&(receive_data_buffer_[i]),2);
                            left_receice_data_buffer_end = 2;
                            i += 2;
                        }
                    }
                    else
                    {continue;}
                }
                else
                {//到头了
                    memcpy(&(receive_data_buffer_[4095]),&(receive_data_buffer_[i]),1);
                    left_receice_data_buffer_end = 1;
                    i += 1;
                }
            }
            else{left_receice_data_buffer_end = 0;}
        }
    }
}

void UartKeeper_Infantry::read(AxisData* &axisData,TeamColor* &EnemyColor,aimMod* &AimMod)
{
    axisData = axis_data_buffer_.read();
    EnemyColor = enemy_color_buffer_.read();
    AimMod = aim_mode_buffer_.read();
}

void UartKeeper_Infantry::write()
{
    this->Uart1->send_data(this->S_DATA,this->sent_length_);
}
void UartKeeper_Infantry::set(const void* data,DataLabel label)
{
    switch(label)
    {
    case YawAngle:memcpy((void*)&(S_DATA[3]),data,4);break;
    case PitchAngle:memcpy((void*)&(S_DATA[7]),data,4);break;
    case AOrR:memcpy((void*)&(S_DATA[2]),data,1);break;
    case FirePermit:memcpy((void*)&(S_DATA[11]),data,1);break;
    }
}
/*=======以下为哨兵部分=======*/
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
void UartKeeper_Guard::read(AxisData* &axisData,TeamColor* &EnemyColor,aimMod* &AimMod)
{
    axisData = nullptr;
    EnemyColor = enemy_color_buffer_.read();
    AimMod = aim_mode_buffer_.read();
}
void UartKeeper_Guard::write()
{
    this->Uart1->send_data(this->S_DATA,this->sent_length_);
}
void UartKeeper_Guard::set(const void* data,DataLabel label)
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
        ssize_t readLength = Uart1->get_data(&(receive_data_buffer_[4096]),MAX_RECIEVE_LENGTH);
        if(readLength == 0)//啥也没读到
        {continue;}

        int p = MAX_RECIEVE_LENGTH - left_receice_data_buffer_end;
        int p_end_plus_one = MAX_RECIEVE_LENGTH + readLength;

        //注意，下文中所有的到头了，其中的i += 的数值均可减一，不影响流程执行,因为下一个循环开始会i++
        for(int i = p;i<p_end_plus_one;i++)
        {//遍历找AA
            if(receive_data_buffer_[i] == 0xAA)
            {
                if(i + 1 < p_end_plus_one)
                {//没到头
                    if(receive_data_buffer_[i + 1] == 0xAA)
                    {//连着两个AA
                        if(i + 2 < p_end_plus_one)
                        {
                            switch(receive_data_buffer_[i + 2])
                            {
                            case 0x01://普通自瞄申请
                                if(i + 3 < p_end_plus_one)
                                {
                                    if(receive_data_buffer_[i + 3] == 0xFF)
                                    {
                                        *(aim_mode_buffer_.Back) = robotMode;
                                        aim_mode_buffer_.writeOver();
                                        //cout<<"AimMod:robotMod"<<endl;
                                        i += 3;
                                    }
                                    else
                                    {continue;}
                                }
                                else
                                {//到头了
                                    memcpy(&(receive_data_buffer_[4093]),&(receive_data_buffer_[i]),3);
                                    left_receice_data_buffer_end = 3;
                                    i += 3;
                                }
                                break;
                            case 0x02://大符自瞄申请
                                if(i + 3 < p_end_plus_one)
                                {
                                    if(receive_data_buffer_[i + 3] == 0xFF)
                                    {
                                        *(aim_mode_buffer_.Back) = buffMode;
                                        aim_mode_buffer_.writeOver();
                                        //cout<<"AimMod:buffMod"<<endl;
                                        i += 3;
                                    }
                                    else
                                    {continue;}
                                }
                                else
                                {
                                    memcpy(&(receive_data_buffer_[4093]),&(receive_data_buffer_[i]),3);
                                    left_receice_data_buffer_end = 3;
                                    i += 3;
                                }
                                break;
                            case 0x03://禁止自瞄申请
                                if(i + 3 < p_end_plus_one)
                                {
                                    if(receive_data_buffer_[i + 3] == 0xFF)
                                    {
                                        *(aim_mode_buffer_.Back) = manualMode;
                                        aim_mode_buffer_.writeOver();
                                        //cout<<"AimMod:manualMode"<<endl;
                                        i += 3;
                                    }
                                    else
                                    {continue;}
                                }
                                else
                                {
                                    memcpy(&(receive_data_buffer_[4093]),&(receive_data_buffer_[i]),3);
                                    left_receice_data_buffer_end = 3;
                                    i += 3;
                                }
                                break;
                            case 0x04:
                                if(i + 21 < p_end_plus_one)
                                {
                                    if(receive_data_buffer_[i + 21] == 0xFF)
                                    {
                                        axis_data_buffer_.Back->ProjectileVel = *((short*)&(receive_data_buffer_[i + 3]));
                                        axis_data_buffer_.Back->RA_yaw = *((float*)&(receive_data_buffer_[i + 5]));
                                        axis_data_buffer_.Back->RA_pitch = *((float*)&(receive_data_buffer_[i + 9]));
                                        axis_data_buffer_.Back->AAV_yaw = *((float*)&(receive_data_buffer_[i + 13]));
                                        axis_data_buffer_.Back->AAV_pitch = *((float*)&(receive_data_buffer_[i + 17]));
                                        axis_data_buffer_.writeOver();
                                        i += 21;
                                    }
                                    else
                                    {continue;}
                                }
                                else
                                {
                                    memcpy(&(receive_data_buffer_[4096 - (p_end_plus_one - i)]),&(receive_data_buffer_[i]),(p_end_plus_one - i));
                                    left_receice_data_buffer_end = p_end_plus_one - i;
                                    i += (p_end_plus_one - i);
                                }
                                break;
                            case 0x07://角色反馈
                                if(i + 4 < p_end_plus_one)
                                {
                                    if(receive_data_buffer_[i + 4] == 0xFF)
                                    {
                                        *(enemy_color_buffer_.Back) = receive_data_buffer_[i + 3] == 0x00 ? teamColor_red : teamColor_blue;
                                        enemy_color_buffer_.writeOver();
                                        i += 4;
                                    }
                                    else
                                    {continue;}
                                }
                                else
                                {
                                    memcpy(&(receive_data_buffer_[4096 - (p_end_plus_one - i)]),&(receive_data_buffer_[i]),(p_end_plus_one - i));
                                    left_receice_data_buffer_end = p_end_plus_one - i;
                                    i += (p_end_plus_one - i);
                                }
                                break;
                            default:return;
                            }
                        }
                        else
                        {//到头了
                            memcpy(&(receive_data_buffer_[4094]),&(receive_data_buffer_[i]),2);
                            left_receice_data_buffer_end = 2;
                            i += 2;
                        }
                    }
                    else
                    {continue;}
                }
                else
                {//到头了
                    memcpy(&(receive_data_buffer_[4095]),&(receive_data_buffer_[i]),1);
                    left_receice_data_buffer_end = 1;
                    i += 1;
                }
            }
            else{left_receice_data_buffer_end = 0;}
        }
    }
}
