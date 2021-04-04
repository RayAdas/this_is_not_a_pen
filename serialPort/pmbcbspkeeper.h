#ifndef PMBCBSPKEEPER_H
#define PMBCBSPKEEPER_H
#include "serialPort/serialPort.h"
#include "tool/triplebuffering.h"
#include <thread>

#ifndef TEAMCOLOR
#define TEAMCOLOR
enum teamColor{teamColor_red = 0,teamColor_blue = 1};
#endif

struct axisData
{
    short ProjectileVel = 0;//#########################是不是16位存疑
    float RA_yaw = 0;//RA = RelativeAngular
    float RA_pitch = 0;//RA = RelativeAngular
    float AAV_yaw = 0;//AAV = AbsoluteAngularVelocity
    float AAV_pitch = 0;//AAV = AbsoluteAngularVelocity
};

enum aimMod{manualMode,buffMod,robotMod};

class PMBCBSPKeeper
{
public:
    PMBCBSPKeeper(serialPort* uart1);
    void read(axisData* &AxisData,teamColor* &EnemyColor,aimMod* &AimMod);
    void boot(void);
private:
    void PMBCBSPCycle();
private:
    tripleBuffering<axisData> AxisDataBuffer;
    tripleBuffering<teamColor> EnemyColorBuffer;
    tripleBuffering<aimMod> AimModBuffer;

    serialPort* Uart1 = nullptr;
    std::thread PMBCBSPThread;
};

#endif // PMBCBSPKEEPER_H
