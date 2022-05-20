#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/sysinfo.h>
#include <systemd/sd-journal.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <vector>

#include <boost/asio/io_service.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <phosphor-logging/log.hpp>
#include <sdbusplus/asio/object_server.hpp>
/*
 * A Service monitoring and interacting with CPLD
 */

#define PROJECT_NAME "pld-managerd"

using namespace phosphor::logging;

namespace cpld
{

#define I2CBUSID   0x7
#define ADDR       0x20

/* CPLD registers definition */
#define SYSTEM_GENERAL_REG_0012                 0x0012
#define DIMM_ALARM_FLAG_BIT         (1<<5)              // MEM_STATUS_REG_0311
#define CPU_ALARM_FLAG_BIT          (1<<4)              // CPU_STATUS_REG_02A1/02A2/02A8/_02A9
#define OPERATE_ALARM_FLAG_BIT      (1<<3)              // BUTTON_LED_STATUS_REG_0120

#define SYSTEM_GENERAL_REG_0013                 0x0013

#define SINGLE_BOARD_GENERAL_POWER_REG_0030     0x0030
#define SYSTEM_PWR_STS_BIT          (1<<0)

#define POWER_ABNORMAL_REG_0090                 0x0090
#define IRQ_CPU0_VRHOT_N_BIT        (1<<1)
#define IRQ_CPU1_VRHOT_N_BIT        (1<<0)

#define BUTTON_LED_STATUS_REG_0120              0x0120
#define SBTN_PWRON_EVT_BIT          (1<<4)
#define LBTN_PWRDOWN_EVT_BIT        (1<<3)
#define SBTN_SYSRST_EVT_BIT         (1<<2)
#define UID_BTN_EVT_BIT             (1<<1)

#define CPU_STATUS_REG_02A1                     0x02A1
#define CPU0_THERMTRIP_BIT          (1<<7)
#define CPU_IERR_ERROR_BIT          (1<<6)
#define CPU_MSMI_BIT                (1<<5)
#define CPU_ERR2_BIT                (1<<2)
#define CPU_ERR1_BIT                (1<<1)
#define CPU_ERR0_BIT                (1<<0)

#define CPU_STATUS_REG_02A2                     0x02A2
#define CPU0_FIVR_FAULT_BIT         (1<<0)

#define CPU_STATUS_REG_02A8                     0x02A8
#define CPU1_THERMTRIP_BIT          (1<<7)

#define CPU_STATUS_REG_02A9                     0x02A9
#define FM_CPU1_FIVR_FAULT_LVT3_CPLD_BIT    (1<<0)

#define MEM_STATUS_REG_0311                     0x0311
#define CPU1_MEMTRIP_BIT            (1<<1)
#define CPU0_MEMTRIP_BIT            (1<<0)


static int readReg(uint busNo, uint8_t slaveAddr, uint16_t reg, uint8_t* data)
{
    uint8_t wbuf[2];
    struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data msgset[1];

    char busName[16];
    std::snprintf(busName, sizeof(busName) - 1, "/dev/i2c-%d", busNo);
    int busFd = open(busName, O_RDWR);
    if (busFd < 0)
    {
        std::cerr << "Failed to open " + std::string(busName) << std::endl;
        return -1;
    }

    wbuf[0] = reg & 0xff; //TODO: check if CPLD accepts LSB first.
    wbuf[1] = reg >> 8;

    msgs[0].addr = slaveAddr >> 1;
    msgs[0].flags = 0;
    msgs[0].len = 2;
    msgs[0].buf = wbuf;

    msgs[1].addr = slaveAddr >> 1;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = 1;
    msgs[1].buf = data;

    msgset[0].msgs = msgs;
    msgset[0].nmsgs = 2;

    if (ioctl(busFd, I2C_RDWR, &msgset) < 0) {
        std::cerr
            << "Failed to read " + std::string(busName) + " addr 0x"
            << std::hex<< slaveAddr << " reg 0x" << std::hex<< reg
            << std::endl;
        (void)close(busFd);
        return -1;
    }

    (void)close(busFd);
    return 0;
}

[[maybe_unused]] static int writeReg(uint busNo, uint8_t slaveAddr, uint16_t reg, uint8_t data)
{
    uint8_t wbuf[3];
    struct i2c_msg msgs[1];
    struct i2c_rdwr_ioctl_data msgset[1];

    char busName[16];
    std::snprintf(busName, sizeof(busName) - 1, "/dev/i2c-%d", busNo);
    int busFd = open(busName, O_RDWR);
    if (busFd < 0)
    {
        std::cerr << "Failed to open " + std::string(busName) << std::endl;
        return -1;
    }

    wbuf[0] = reg & 0xff; //TODO: check if CPLD accepts LSB first.
    wbuf[1] = reg >> 8;
    wbuf[2] = data;

    msgs[0].addr = slaveAddr >> 1;
    msgs[0].flags = 0;
    msgs[0].len = 3;
    msgs[0].buf = wbuf;

    msgset[0].msgs = msgs;
    msgset[0].nmsgs = 1;

    if (ioctl(busFd, I2C_RDWR, &msgset) < 0) {
        std::cerr
            << "Failed to write " + std::string(busName) + " addr 0x"
            << std::hex<< slaveAddr << " reg 0x" << std::hex<< reg
            << " with 0x" << std::hex << data << std::endl;
        (void)close(busFd);
        return -1;
    }

    (void)close(busFd);
    return 0;
}

}

static boost::asio::io_service io;
std::shared_ptr<sdbusplus::asio::connection> conn;
static std::string hostDbusName = "xyz.openbmc_project.Cpld";

static std::shared_ptr<sdbusplus::asio::dbus_interface> platformStatesIfc;
static std::shared_ptr<sdbusplus::asio::dbus_interface> platformEventsIfc;
static std::shared_ptr<sdbusplus::asio::dbus_interface> platformControlsIfc;

// This map contains all timer values that are to be read from json config
boost::container::flat_map<std::string, int> TimerMap = {
    {"PollingIntervalMs", 1000}};

static boost::asio::steady_timer pollingTimer(io);

// property names
static const std::string powerOkPN = "PowerOk";
static const std::string cpuCatErrPN = "CpuCatErr";
static const std::string cpu0FivrFaultPN = "Cpu0FivrFault";
static const std::string cpu1FivrFaultPN = "Cpu1FivrFault";
static const std::string cpu0MemThermalTripPN = "Cpu0MemThermalTrip";
static const std::string cpu1MemThermalTripPN = "Cpu1MemThermalTrip";
static const std::string cpu0VrHotPN = "Cpu0VrHot";
static const std::string cpu1VrHotPN = "Cpu1VrHot";

// signal names
static const std::string uidButtonSN = "UidButton";
static const std::string shortBtnPowerOnSN = "ShortBtnPowerOn";
static const std::string longBtnPowerDownSN = "LongBtnPowerDown";
static const std::string shortBtnResetSN = "ShortBtnReset";
static const std::string cpuMSMISN = "CpuMSMI";
static const std::string cpuErr0SN = "CpuErr0";
static const std::string cpuErr1SN = "CpuErr1";
static const std::string cpuErr2SN = "CpuErr2";
static const std::string cpu0ThermalTripSN = "Cpu0ThermalTrip";
static const std::string cpu1ThermalTripSN = "Cpu1ThermalTrip";

enum class PropertyType
{
    propety,
    signal,
    method
};

struct PropertyMetaData
{
    PropertyType type;
    bool state;
    uint8_t regOffset;
};

boost::container::flat_map<std::string, PropertyMetaData> exposedProp = {

    {powerOkPN, {PropertyType::propety, false, SYSTEM_PWR_STS_BIT}},
    {cpuCatErrPN, {PropertyType::propety, false, CPU_IERR_ERROR_BIT}},
    {cpu0FivrFaultPN, {PropertyType::propety, false, CPU0_FIVR_FAULT_BIT}},
    {cpu1FivrFaultPN, {PropertyType::propety, false, FM_CPU1_FIVR_FAULT_LVT3_CPLD_BIT}},
    {cpu0MemThermalTripPN, {PropertyType::propety, false, CPU0_MEMTRIP_BIT}},
    {cpu1MemThermalTripPN, {PropertyType::propety, false, CPU1_MEMTRIP_BIT}},
    {cpu0VrHotPN, {PropertyType::propety, false, IRQ_CPU0_VRHOT_N_BIT}},
    {cpu1VrHotPN, {PropertyType::propety, false, IRQ_CPU1_VRHOT_N_BIT}},

    {uidButtonSN, {PropertyType::signal, false, UID_BTN_EVT_BIT}},
    {shortBtnPowerOnSN, {PropertyType::signal, false, SBTN_PWRON_EVT_BIT}},
    {longBtnPowerDownSN, {PropertyType::signal, false, LBTN_PWRDOWN_EVT_BIT}},
    {shortBtnResetSN, {PropertyType::signal, false, SBTN_SYSRST_EVT_BIT}},
    {cpuMSMISN, {PropertyType::signal, false, CPU_MSMI_BIT}},
    {cpuErr0SN, {PropertyType::signal, false, CPU_ERR0_BIT}},
    {cpuErr1SN, {PropertyType::signal, false, CPU_ERR1_BIT}},
    {cpuErr2SN, {PropertyType::signal, false, CPU_ERR2_BIT}},
    {cpu0ThermalTripSN, {PropertyType::signal, false, CPU0_THERMTRIP_BIT}},
    {cpu1ThermalTripSN, {PropertyType::signal, false, CPU1_THERMTRIP_BIT}}
};

static void updateStateProperty(const std::string name, uint8_t regData)
{
    if (exposedProp[name].type != PropertyType::propety)
        std::cerr << "Incorrect use of updateStateProperty() on " << name << std::endl;

    uint8_t regOffset = exposedProp[name].regOffset;
    bool state = regData & regOffset? true : false;
    if (exposedProp[name].state != state)
    {
        exposedProp[name].state = state;
        platformStatesIfc->set_property(name, state);
        std::string strState = state? "True" : "False";
        std::string logMsg = name + " changes to " + strState;
        log<level::INFO>(logMsg.c_str());
    }
}

static void emitSignal(const std::string name, uint8_t regData)
{
    if (exposedProp[name].type != PropertyType::signal)
        std::cerr << "Incorrect use of updateStateProperty() on " << name << std::endl;

    uint8_t regOffset = exposedProp[name].regOffset;
    bool asserted = regData & regOffset ? true : false;
    if (asserted)
    {
        sdbusplus::message::message msg =
            platformEventsIfc->new_signal(name.c_str());
        msg.append("asserted");
        msg.signal_send();
        std::string logMsg = "Signal " + name + " asserted";
        log<level::INFO>(logMsg.c_str());
    }
}

static void poll(void)
{
    uint8_t regData;

    // PowerOk
    regData = 0;
    if (!cpld::readReg(I2CBUSID, ADDR, SINGLE_BOARD_GENERAL_POWER_REG_0030, &regData))
        updateStateProperty(powerOkPN, regData);
    else
        log<level::ERR>("Failed to read Power OK state.");

    // Cpu0VrHot/Cpu1VrHot
    regData = 0;
    if (!cpld::readReg(I2CBUSID, ADDR, POWER_ABNORMAL_REG_0090, &regData))
    {
        updateStateProperty(cpu0VrHotPN, regData);
        updateStateProperty(cpu1VrHotPN, regData);
    }
    else
        log<level::ERR>("Failed to init CpuCatErr.");

    /* Chekcs SYSTEM_GENERAL_REG_0012 group */
    uint8_t genReg12Data = 0;
    if (cpld::readReg(I2CBUSID, ADDR, SYSTEM_GENERAL_REG_0012, &genReg12Data))
    {
        log<level::ERR>("Failed to read SYSTEM_GENERAL_REG_0012.");
    }

    if (genReg12Data
        // For a state like cpuCatErr has already asserted, we would need to
        // check if it is desserted now and update latest state to dbus
        // accordingly.
        || exposedProp[cpuCatErrPN].state
        || exposedProp[cpu0FivrFaultPN].state
        || exposedProp[cpu1FivrFaultPN].state
        || exposedProp[cpu0MemThermalTripPN].state
        || exposedProp[cpu1MemThermalTripPN].state)
    {
        /* Checks OPERATE_ALARM_FLAG_BIT + BUTTON_LED_STATUS_REG_0120 sub group */
        if (genReg12Data & OPERATE_ALARM_FLAG_BIT)
        {
            regData = 0;
            if (!cpld::readReg(I2CBUSID, ADDR, BUTTON_LED_STATUS_REG_0120, &regData))
            {
                emitSignal(uidButtonSN, regData);
                emitSignal(shortBtnPowerOnSN, regData);
                emitSignal(longBtnPowerDownSN, regData);
                emitSignal(shortBtnResetSN, regData);
                // clear events
                if (cpld::writeReg(I2CBUSID, ADDR, BUTTON_LED_STATUS_REG_0120, 0))
                    log<level::ERR>("Failed to clear BUTTON_LED_STATUS_REG_0120.");
            }
            else
                log<level::ERR>("Failed to read BUTTON_LED_STATUS_REG_0120.");
        }

        /* Checks CPU_ALARM_FLAG_BIT + CPU_STATUS_REG_02A1/02A2/02A8/02A9 sub group */
        if (genReg12Data & CPU_ALARM_FLAG_BIT
            // For a state like cpuCatErr has already asserted, we would need to
            // check if it is desserted now and update latest state to dbus
            // accordingly.
            || exposedProp[cpuCatErrPN].state
            || exposedProp[cpu0FivrFaultPN].state
            || exposedProp[cpu1FivrFaultPN].state)
        {
            regData = 0;
            if (!cpld::readReg(I2CBUSID, ADDR, CPU_STATUS_REG_02A1, &regData))
            {
                updateStateProperty(cpuCatErrPN, regData);
                emitSignal(cpu0ThermalTripSN, regData);
                emitSignal(cpuMSMISN, regData);
                emitSignal(cpuErr2SN, regData);
                emitSignal(cpuErr1SN, regData);
                emitSignal(cpuErr0SN, regData);
                // clear events
                if (cpld::writeReg(I2CBUSID, ADDR, CPU_STATUS_REG_02A1, 0))
                    log<level::ERR>("Failed to clear CPU_STATUS_REG_02A1.");
            }
            else
                log<level::ERR>("Failed to read CPU_STATUS_REG_02A1.");

            regData = 0;
            if (!cpld::readReg(I2CBUSID, ADDR, CPU_STATUS_REG_02A2, &regData))
            {
                updateStateProperty(cpu0FivrFaultPN, regData);
            }
            else
                log<level::ERR>("Failed to read CPU_STATUS_REG_02A2.");

            regData = 0;
            if (!cpld::readReg(I2CBUSID, ADDR, CPU_STATUS_REG_02A8, &regData))
            {
                emitSignal(cpu1ThermalTripSN, regData);
                // clear events
                if (cpld::writeReg(I2CBUSID, ADDR, CPU_STATUS_REG_02A8, 0))
                    log<level::ERR>("Failed to clear CPU_STATUS_REG_02A8.");
            }
            else
                log<level::ERR>("Failed to read CPU_STATUS_REG_02A8.");

            regData = 0;
            if (!cpld::readReg(I2CBUSID, ADDR, CPU_STATUS_REG_02A9, &regData))
            {
                updateStateProperty(cpu1FivrFaultPN, regData);
            }
            else
                log<level::ERR>("Failed to read CPU_STATUS_REG_02A9.");
        }

        /* Checks DIMM_ALARM_FLAG_BIT + MEM_STATUS_REG_0311 sub group */
        if (genReg12Data & DIMM_ALARM_FLAG_BIT
            || exposedProp[cpu0MemThermalTripPN].state
            || exposedProp[cpu1MemThermalTripPN].state)
        {
            regData = 0;
            if (!cpld::readReg(I2CBUSID, ADDR, MEM_STATUS_REG_0311, &regData))
            {
                updateStateProperty(cpu0MemThermalTripPN, regData);
                updateStateProperty(cpu1MemThermalTripPN, regData);
            }
            else
                log<level::ERR>("Failed to read MEM_STATUS_REG_0311.");
        }

    }

    // /* Chekcs SYSTEM_GENERAL_REG_0013 group */
    // uint8_t genReg13Data = 0;
    // if (cpld::readReg(I2CBUSID, ADDR, SYSTEM_GENERAL_REG_0013, &genReg13Data))
    // {
    //     log<level::ERR>("Failed to read SYSTEM_GENERAL_REG_0013.");
    // }

    // if (genReg13Data & xx)
    // {
    // }

    pollingTimer.expires_after(std::chrono::milliseconds(TimerMap["PollingIntervalMs"]));
    pollingTimer.async_wait([](const boost::system::error_code ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                std::cerr << " polling async_wait failed: " << ec.message()
                          << "\n";
            }
            return;
        }
        poll();
    });
}

int main(int argc, char **argv) {
    if(argc != 1) {
        std::cout << argv[0] <<  "takes no arguments.\n";
        return 1;
    }
    std::cout << "This is project " << PROJECT_NAME << ".\n";

    uint8_t regData = 0;
    bool state;
    uint8_t regOffset;

    std::string infoMsg =
        "Start CPLD Manager Service.";
    log<level::INFO>(infoMsg.c_str());

    conn = std::make_shared<sdbusplus::asio::connection>(io);

    conn->request_name(hostDbusName.c_str());

    sdbusplus::asio::object_server hostServer =
        sdbusplus::asio::object_server(conn);

    /* Expose platform states */
    platformStatesIfc =
        hostServer.add_interface("/xyz/openbmc_project/cpld/platform_states",
                                 "xyz.openbmc_project.Cpld.PlatformStates");

    // PowerOk
    regData = 0;
    if (cpld::readReg(I2CBUSID, ADDR, SINGLE_BOARD_GENERAL_POWER_REG_0030, &regData))
    {
        log<level::ERR>("Failed to init PowerOK.");
    }
    regOffset = exposedProp[powerOkPN].regOffset;
    state = regData & regOffset ? true : false;
    exposedProp[powerOkPN].state = state;
    platformStatesIfc->register_property(powerOkPN, state);

    // Cpu0FivrFault
    regData = 0;
    if (cpld::readReg(I2CBUSID, ADDR, CPU_STATUS_REG_02A2, &regData))
    {
        log<level::ERR>("Failed to init Cpu0FivrFault.");
    }
    regOffset = exposedProp[cpu0FivrFaultPN].regOffset;
    state = regData & regOffset ? true : false;
    exposedProp[cpu0FivrFaultPN].state = state;
    platformStatesIfc->register_property(cpu0FivrFaultPN, state);

    // Cpu1FivrFault
    regData = 0;
    if (cpld::readReg(I2CBUSID, ADDR, CPU_STATUS_REG_02A9, &regData))
    {
        log<level::ERR>("Failed to init Cpu1FivrFault.");
    }
    regOffset = exposedProp[cpu1FivrFaultPN].regOffset;
    state = regData & regOffset ? true : false;
    exposedProp[cpu1FivrFaultPN].state = state;
    platformStatesIfc->register_property(cpu1FivrFaultPN, state);

    // CpuCatErr
    regData = 0;
    if (cpld::readReg(I2CBUSID, ADDR, CPU_STATUS_REG_02A1, &regData))
    {
        log<level::ERR>("Failed to init CpuCatErr.");
    }
    regOffset = exposedProp[cpuCatErrPN].regOffset;
    state = regData & regOffset ? true : false;
    exposedProp[cpuCatErrPN].state = state;
    platformStatesIfc->register_property(cpuCatErrPN, state);

    // Cpu0MemThermalTrip/Cpu1MemThermalTrip
    regData = 0;
    if (cpld::readReg(I2CBUSID, ADDR, MEM_STATUS_REG_0311, &regData))
    {
        log<level::ERR>("Failed to init Cpu0MemThermalTrip/Cpu1MemThermalTrip.");
    }
    regOffset = exposedProp[cpu0MemThermalTripPN].regOffset;
    state = regData & regOffset ? true : false;
    exposedProp[cpu0MemThermalTripPN].state = state;
    platformStatesIfc->register_property(cpu0MemThermalTripPN, state);

    regOffset = exposedProp[cpu1MemThermalTripPN].regOffset;
    state = regData & regOffset ? true : false;
    exposedProp[cpu1MemThermalTripPN].state = state;
    platformStatesIfc->register_property(cpu1MemThermalTripPN, state);

    // Cpu0VrHot/Cpu1VrHot
    regData = 0;
    if (cpld::readReg(I2CBUSID, ADDR, POWER_ABNORMAL_REG_0090, &regData))
    {
        log<level::ERR>("Failed to init Cpu0VrHot/Cpu1VrHot.");
    }
    regOffset = exposedProp[cpu0VrHotPN].regOffset;
    state = regData & regOffset ? true : false;
    exposedProp[cpu0VrHotPN].state = state;
    platformStatesIfc->register_property(cpu0VrHotPN, state);

    regOffset = exposedProp[cpu1VrHotPN].regOffset;
    state = regData & regOffset ? true : false;
    exposedProp[cpu1VrHotPN].state = state;
    platformStatesIfc->register_property(cpu1VrHotPN, state);

    platformStatesIfc->initialize();

    /* Expose platform events */
    platformEventsIfc =
        hostServer.add_interface("/xyz/openbmc_project/cpld/platform_events",
                                 "xyz.openbmc_project.Cpld.PlatformEvents");

    platformEventsIfc->register_signal<std::string>(uidButtonSN);
    platformEventsIfc->register_signal<std::string>(shortBtnPowerOnSN);
    platformEventsIfc->register_signal<std::string>(longBtnPowerDownSN);
    platformEventsIfc->register_signal<std::string>(shortBtnResetSN);
    platformEventsIfc->register_signal<std::string>(cpuMSMISN);
    platformEventsIfc->register_signal<std::string>(cpuErr0SN);
    platformEventsIfc->register_signal<std::string>(cpuErr1SN);
    platformEventsIfc->register_signal<std::string>(cpuErr2SN);
    platformEventsIfc->register_signal<std::string>(cpu0ThermalTripSN);
    platformEventsIfc->register_signal<std::string>(cpu1ThermalTripSN);

    platformEventsIfc->initialize();

    /* Expose platform control knobs */
    platformControlsIfc =
        hostServer.add_interface("/xyz/openbmc_project/cpld/platform_controls",
                                 "xyz.openbmc_project.Cpld.PlatformControls");

    platformControlsIfc->initialize();

    io.post([&]() {
        poll();
    });

    io.run();

    return 0;
}
