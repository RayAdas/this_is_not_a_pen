#ifndef PMBCBSPKEEPER_H
#define PMBCBSPKEEPER_H
#include "serialPort/serialPort.h"
#include "tool/triplebuffering.h"
#include <thread>
#include "preferences.h"

enum data_label{YawAngle,PitchAngle,FirePermit,AOrR};//A0rR用于指示相对角和绝对角

struct axisData
{
    short ProjectileVel = 0;//#########################是不是16位存疑
    float RA_yaw = 0;//RA = RelativeAngular
    float RA_pitch = 0;//RA = RelativeAngular
    float AAV_yaw = 0;//AAV = AbsoluteAngularVelocity
    float AAV_pitch = 0;//AAV = AbsoluteAngularVelocity
};

enum aimMod{manualMode,buffMod,robotMod};

class UartKeeper
{
public:
    virtual void boot(void) = 0;
    virtual void read(axisData* &AxisData,teamColor* &EnemyColor,aimMod* &AimMod) = 0;
    virtual void write() = 0;
    virtual void set(const void* data,data_label label) = 0;
protected:
    virtual void KeeperCycle() = 0;
protected:
    int sent_length;
    std::thread KeeperThread;
    serialPort* Uart1 = nullptr;
    tripleBuffering<axisData> AxisDataBuffer;
    tripleBuffering<teamColor> EnemyColorBuffer;
    tripleBuffering<aimMod> AimModBuffer;
};

class UartKeeper_Infantry:public UartKeeper
{
public:
    UartKeeper_Infantry(serialPort* uart1);
    void boot(void) override;
    void read(axisData* &AxisData,teamColor* &EnemyColor,aimMod* &AimMod) override;
    void write() override;
    void set(const void* data,data_label label) override;
private:
    void KeeperCycle() override;
private:
    unsigned char S_DATA[13];
};

class UartKeeper_Guard:public UartKeeper
{
public:
    UartKeeper_Guard(serialPort* uart1);
    void boot(void) override;
    void read(axisData* &AxisData,teamColor* &EnemyColor,aimMod* &AimMod) override;
    void write() override;
    void set(const void* data,data_label label) override;
private:
    void KeeperCycle() override;
private:
    unsigned char S_DATA[13];
};
#endif // PMBCBSPKEEPER_H
