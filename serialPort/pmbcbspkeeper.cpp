#include "pmbcbspkeeper.h"

PMBCBSPKeeper::PMBCBSPKeeper(serialPort* uart1)
{
    this->Uart1 = uart1;
    *(this->AimModBuffer.Back) = manualMode;
    this->AimModBuffer.writeOver();
    *(this->AimModBuffer.Back) = manualMode;
    this->AimModBuffer.writeOver();

    *(this->EnemyColorBuffer.Back) = teamColor_red;
    this->EnemyColorBuffer.writeOver();
    *(this->EnemyColorBuffer.Back) = teamColor_red;
    this->EnemyColorBuffer.writeOver();}

void PMBCBSPKeeper::boot(void)
{
    PMBCBSPThread = std::thread(&PMBCBSPKeeper::PMBCBSPCycle,this);
}

void PMBCBSPKeeper::PMBCBSPCycle()
{
    while(true)
    {
        if(Uart1->get_data())
        {
            switch(Uart1->receiceData[2])
            {
            case 0x01://普通自瞄申请
                *(AimModBuffer.Back) = robotMod;
                AimModBuffer.writeOver();
                //cout<<"AimMod:robotMod"<<endl;
                break;
            case 0x02://大符自瞄申请
                *(AimModBuffer.Back) = buffMod;
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
            case 0x07://角色反馈
                *(EnemyColorBuffer.Back) = Uart1->receiceData[3] == 0x00 ? teamColor_red : teamColor_blue;
                EnemyColorBuffer.writeOver();
                break;
            default:return;
            }
        }
    }
}

void PMBCBSPKeeper::read(axisData* &AxisData,teamColor* &EnemyColor,aimMod* &AimMod)
{
    AxisData = AxisDataBuffer.read();
    EnemyColor = EnemyColorBuffer.read();
    AimMod = AimModBuffer.read();
}
