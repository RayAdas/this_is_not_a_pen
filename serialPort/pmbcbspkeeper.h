#ifndef PMBCBSPKEEPER_H
#define PMBCBSPKEEPER_H
#include "serialPort/serialPort.h"
#include "tool/triplebuffering.h"
#include <thread>
#include "preferences.h"

enum data_label{YawAngle,PitchAngle,FirePermit,AOrR,YunTaiMode};//A0rR用于指示相对角和绝对角//YunTaiMode云台模式

struct axisData
{
    short ProjectileVel = 0;//#########################是不是16位存疑
    float RA_yaw = 0;//RA = RelativeAngular
    float RA_pitch = 0;//RA = RelativeAngular
    float AAV_yaw = 0;//AAV = AbsoluteAngularVelocity
    float AAV_pitch = 0;//AAV = AbsoluteAngularVelocity
};

enum aimMod{manualMode,buffMode,robotMode};

class UartKeeper
{
public:
    virtual void boot(void) = 0;
    virtual void read(axisData* &AxisData,TeamColor* &EnemyColor,aimMod* &AimMod) = 0;
    virtual void write() = 0;
    virtual void set(const void* data,data_label label) = 0;
protected:
    virtual void KeeperCycle() = 0;
protected:
    int sent_length_;
    std::thread keeper_thread_;
    SerialPort* Uart1 = nullptr;
    TripleBuffering<axisData> axis_data_buffer_;
    TripleBuffering<TeamColor> enemy_color_buffer_;
    TripleBuffering<aimMod> aim_mode_buffer_;
};

class UartKeeper_Infantry:public UartKeeper
{
public:
    UartKeeper_Infantry(SerialPort* uart1);
    void boot(void) override;
    void read(axisData* &AxisData,TeamColor* &EnemyColor,aimMod* &AimMod) override;
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
    UartKeeper_Guard(SerialPort* uart1);
    void boot(void) override;
    void read(axisData* &AxisData,TeamColor* &EnemyColor,aimMod* &AimMod) override;
    void write() override;
    void set(const void* data,data_label label) override;
private:
    void KeeperCycle() override;
private:
    unsigned char S_DATA[13];
};
#endif // PMBCBSPKEEPER_H
