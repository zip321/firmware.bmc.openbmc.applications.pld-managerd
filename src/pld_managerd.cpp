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
#include <gpiod.hpp>
#include <phosphor-logging/log.hpp>
#include <sdbusplus/asio/object_server.hpp>
/*
 * A Service monitoring and interacting with CPLD
 */

#define PROJECT_NAME "pld-managerd"

using namespace phosphor::logging;

namespace cpld
{

static constexpr bool debug = false;

#define CMU_CPLD_BUSID   0x7
#define BUSID      0xB
#define ADDR       0x20

/* CPLD registers definition */

#define CPLD_VER_B                              0x0005
#define TEST_REG_0006                           0x0006
#define CPLD_VER_A                              0x000B
#define CPLD_VER_MATCH                          0x000C  // 0: match

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

#define BUTTON_LED_CONTROL_REG_0130             0x0130
#define BMC_SBTN_POWRDOWN_CTL_BIT   (1<<3)
#define BMC_LBTN_PWRDOWN_CTL_BIT    (1<<2)
#define BMC_SBTN_POWRON_CTL_BIT     (1<<1)
#define BMC_SBTN_SYSRST_CTL_BIT     (1<<0)

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

#define PCH_CONTROL_REG_03A9                    0x03A0
#define BMC_NMI_CTL_BIT             (1<<6)


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

    wbuf[0] = (reg & 0xff00) >> 8;
    wbuf[1] = reg & 0xff;


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
            << std::hex<< slaveAddr << " reg 0x" << std::hex << reg
            << std::endl;
        (void)close(busFd);
        return -1;
    }

    if (debug)
        std::cout
            << "read " + std::string(busName) + " addr 0x"
            << std::hex << int(slaveAddr) << " reg 0x" << std::hex << reg
            << " with reading 0x" << std::hex << int(*data)
            << std::endl;

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

    wbuf[0] = (reg & 0xff00) >> 8;
    wbuf[1] = reg & 0x00ff;
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

    if (debug)
        std::cout
            << "wrtie " + std::string(busName) + " addr 0x"
            << std::hex << int(slaveAddr) << " reg 0x" << std::hex << reg
            << " with data 0x" << std::hex << int(data)
            << std::endl;

    (void)close(busFd);
    return 0;
}

}

static boost::asio::io_service io;
std::shared_ptr<sdbusplus::asio::connection> conn;
static std::string hostDbusName = "xyz.openbmc_project.Cpld";
static std::string hostDbusName1 = "xyz.openbmc_project.PldManager";

static std::shared_ptr<sdbusplus::asio::dbus_interface> platformStatesIfc;
static std::shared_ptr<sdbusplus::asio::dbus_interface> platformEventsIfc;
static std::shared_ptr<sdbusplus::asio::dbus_interface> platformControlsIfc;

// This map contains all timer values that are to be read from json config
boost::container::flat_map<std::string, int> TimerMap = {
    {"PollingIntervalMs", 1000}};

static boost::asio::steady_timer pollingTimer(io);

// property(dbus) names
static const std::string powerOkPN = "PowerOk";
static const std::string cpuCatErrPN = "CpuCatErr";
static const std::string cpu0FivrFaultPN = "Cpu0FivrFault";
static const std::string cpu1FivrFaultPN = "Cpu1FivrFault";
static const std::string cpu0MemThermalTripPN = "Cpu0MemThermalTrip";
static const std::string cpu1MemThermalTripPN = "Cpu1MemThermalTrip";
static const std::string cpu0VrHotPN = "Cpu0VrHot";
static const std::string cpu1VrHotPN = "Cpu1VrHot";

// signal(dbus) names
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
    uint8_t regOffset;
};

boost::container::flat_map<std::string, PropertyMetaData> exposedProp = {
    /**
     * @brief Basically mapped to those Read-Only-signals exposed by the CPLD.
     * 
     */
    {powerOkPN, {PropertyType::propety, SYSTEM_PWR_STS_BIT}},
    {cpuCatErrPN, {PropertyType::propety, CPU_IERR_ERROR_BIT}},
    {cpu0FivrFaultPN, {PropertyType::propety, CPU0_FIVR_FAULT_BIT}},
    {cpu1FivrFaultPN, {PropertyType::propety, FM_CPU1_FIVR_FAULT_LVT3_CPLD_BIT}},
    {cpu0MemThermalTripPN, {PropertyType::propety, CPU0_MEMTRIP_BIT}},
    {cpu1MemThermalTripPN, {PropertyType::propety, CPU1_MEMTRIP_BIT}},
    {cpu0VrHotPN, {PropertyType::propety, IRQ_CPU0_VRHOT_N_BIT}},
    {cpu1VrHotPN, {PropertyType::propety, IRQ_CPU1_VRHOT_N_BIT}},
    /**
     * @brief Basically mapped to those Write-Clear-signals exposed by the CPLD.
     * 
     */
    {uidButtonSN, {PropertyType::signal, UID_BTN_EVT_BIT}},
    {shortBtnPowerOnSN, {PropertyType::signal, SBTN_PWRON_EVT_BIT}},
    {longBtnPowerDownSN, {PropertyType::signal, LBTN_PWRDOWN_EVT_BIT}},
    {shortBtnResetSN, {PropertyType::signal, SBTN_SYSRST_EVT_BIT}},
    {cpuMSMISN, {PropertyType::signal, CPU_MSMI_BIT}},
    {cpuErr0SN, {PropertyType::signal, CPU_ERR0_BIT}},
    {cpuErr1SN, {PropertyType::signal, CPU_ERR1_BIT}},
    {cpuErr2SN, {PropertyType::signal, CPU_ERR2_BIT}},
    {cpu0ThermalTripSN, {PropertyType::signal, CPU0_THERMTRIP_BIT}},
    {cpu1ThermalTripSN, {PropertyType::signal, CPU1_THERMTRIP_BIT}}
};

boost::container::flat_map<std::string, bool> states = {
    {powerOkPN, false},
    {cpuCatErrPN, false},
    {cpu0FivrFaultPN, false},
    {cpu1FivrFaultPN, false},
    {cpu0MemThermalTripPN, false},
    {cpu1MemThermalTripPN, false},
    {cpu0VrHotPN, false},
    {cpu1VrHotPN, false}
};

static void updateStateProperty(const std::string name, uint8_t regData)
{
    if (exposedProp[name].type != PropertyType::propety)
    {
        std::cerr << "Incorrect use of updateStateProperty() on " << name << std::endl;
        return;
    }

    uint8_t regOffset = exposedProp[name].regOffset;
    bool state = regData & regOffset? true : false;
    if (states[name] != state)
    {
        states[name] = state;
        platformStatesIfc->set_property(name, state);
        std::string strState = state? "True" : "False";
        std::string logMsg = name + " changes to " + strState;
        log<level::INFO>(logMsg.c_str());
    }
}

static void emitSignal(const std::string name, uint8_t regData)
{
    if (exposedProp[name].type != PropertyType::signal)
    {
        std::cerr << "Incorrect use of emitSignal() on " << name << std::endl;
        return;
    }

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
    if (!cpld::readReg(BUSID, ADDR, SINGLE_BOARD_GENERAL_POWER_REG_0030, &regData))
        updateStateProperty(powerOkPN, regData);
    else
        log<level::ERR>("Failed to read Power OK state.");

    // Cpu0VrHot/Cpu1VrHot
    regData = 0;
    if (!cpld::readReg(BUSID, ADDR, POWER_ABNORMAL_REG_0090, &regData))
    {
        updateStateProperty(cpu0VrHotPN, regData);
        updateStateProperty(cpu1VrHotPN, regData);
    }
    else
        log<level::ERR>("Failed to init CpuCatErr.");

    /* Chekcs SYSTEM_GENERAL_REG_0012 group */
    uint8_t genReg12Data = 0;
    if (cpld::readReg(BUSID, ADDR, SYSTEM_GENERAL_REG_0012, &genReg12Data))
    {
        log<level::ERR>("Failed to read SYSTEM_GENERAL_REG_0012.");
    }

    if (genReg12Data
        // For a state like cpuCatErr has already asserted, we would need to
        // check if it is desserted now and update latest state to dbus
        // accordingly.
        || states[cpuCatErrPN]
        || states[cpu0FivrFaultPN]
        || states[cpu1FivrFaultPN]
        || states[cpu0MemThermalTripPN]
        || states[cpu1MemThermalTripPN])
    {
        /* Checks OPERATE_ALARM_FLAG_BIT + BUTTON_LED_STATUS_REG_0120 sub group */
        if (genReg12Data & OPERATE_ALARM_FLAG_BIT)
        {
            regData = 0;
            if (!cpld::readReg(BUSID, ADDR, BUTTON_LED_STATUS_REG_0120, &regData))
            {
                emitSignal(uidButtonSN, regData);
                emitSignal(shortBtnPowerOnSN, regData);
                emitSignal(longBtnPowerDownSN, regData);
                emitSignal(shortBtnResetSN, regData);
                // clear events
                if (cpld::writeReg(BUSID, ADDR, BUTTON_LED_STATUS_REG_0120, 0))
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
            || states[cpuCatErrPN]
            || states[cpu0FivrFaultPN]
            || states[cpu1FivrFaultPN])
        {
            regData = 0;
            if (!cpld::readReg(BUSID, ADDR, CPU_STATUS_REG_02A1, &regData))
            {
                updateStateProperty(cpuCatErrPN, regData);
                emitSignal(cpu0ThermalTripSN, regData);
                emitSignal(cpuMSMISN, regData);
                emitSignal(cpuErr2SN, regData);
                emitSignal(cpuErr1SN, regData);
                emitSignal(cpuErr0SN, regData);
                // clear events
                if (cpld::writeReg(BUSID, ADDR, CPU_STATUS_REG_02A1, 0))
                    log<level::ERR>("Failed to clear CPU_STATUS_REG_02A1.");
            }
            else
                log<level::ERR>("Failed to read CPU_STATUS_REG_02A1.");

            regData = 0;
            if (!cpld::readReg(BUSID, ADDR, CPU_STATUS_REG_02A2, &regData))
            {
                updateStateProperty(cpu0FivrFaultPN, regData);
            }
            else
                log<level::ERR>("Failed to read CPU_STATUS_REG_02A2.");

            regData = 0;
            if (!cpld::readReg(BUSID, ADDR, CPU_STATUS_REG_02A8, &regData))
            {
                emitSignal(cpu1ThermalTripSN, regData);
                // clear events
                if (cpld::writeReg(BUSID, ADDR, CPU_STATUS_REG_02A8, 0))
                    log<level::ERR>("Failed to clear CPU_STATUS_REG_02A8.");
            }
            else
                log<level::ERR>("Failed to read CPU_STATUS_REG_02A8.");

            regData = 0;
            if (!cpld::readReg(BUSID, ADDR, CPU_STATUS_REG_02A9, &regData))
            {
                updateStateProperty(cpu1FivrFaultPN, regData);
            }
            else
                log<level::ERR>("Failed to read CPU_STATUS_REG_02A9.");
        }

        /* Checks DIMM_ALARM_FLAG_BIT + MEM_STATUS_REG_0311 sub group */
        if (genReg12Data & DIMM_ALARM_FLAG_BIT
            || states[cpu0MemThermalTripPN]
            || states[cpu1MemThermalTripPN])
        {
            regData = 0;
            if (!cpld::readReg(BUSID, ADDR, MEM_STATUS_REG_0311, &regData))
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
    // if (cpld::readReg(BUSID, ADDR, SYSTEM_GENERAL_REG_0013, &genReg13Data))
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

static bool setGPIOOutput(const std::string& name, const int value,
                          gpiod::line& gpioLine)
{
    // Find the GPIO line
    gpioLine = gpiod::find_line(name);
    if (!gpioLine)
    {
        std::string errMsg = "Failed to find the " + name + " line";
        log<level::ERR>(errMsg.c_str());
        return false;
    }

    // Request GPIO output to specified value
    try
    {
        gpioLine.request(
            {"pld_manager", gpiod::line_request::DIRECTION_OUTPUT, {}}, value);
    }
    catch (const std::exception&)
    {
        std::string errMsg = "Failed to request " + name + " output";
        log<level::ERR>(errMsg.c_str());
        return false;
    }

    std::string logMsg = name + " set to " + std::to_string(value);
    log<level::INFO>(logMsg.c_str());
    return true;
}

static boost::asio::steady_timer heartbeat2CpldTimer(io);
static void heartbeat2CPLD(gpiod::line& gpioLine)
{
    static int value = 0;
    value = !value;
    int v = value;

    heartbeat2CpldTimer.expires_after(std::chrono::milliseconds(1000));
    heartbeat2CpldTimer.async_wait(
        [&gpioLine, v](const boost::system::error_code ec)
    {
        // Set the GPIO line back to the opposite value
        gpioLine.set_value(v);

        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                std::string errMsg =
                    "heartbeat2CPLD async_wait failed: " + ec.message();
                log<level::ERR>(errMsg.c_str());
            }
        }
        heartbeat2CPLD(gpioLine);
    });
}

static void sanityCheck(void)
{
    uint8_t regData = 0;
    cpld::writeReg(BUSID, ADDR, TEST_REG_0006, 0x77);
    cpld::readReg(BUSID, ADDR, TEST_REG_0006, &regData);
    std::cout
            << "CPLD I2C Management Interface: "
            << ((regData == uint8_t(~(0x77))) ? "ON" : "OFF")
            << std::endl;

    // MB CPLD version
    uint8_t versionA = 0xff;
    uint8_t versionB = 0xff;
    cpld::readReg(BUSID, ADDR, CPLD_VER_A, &versionA);
    cpld::readReg(BUSID, ADDR, CPLD_VER_B, &versionB);
    std::cout
            << "MB CPLD version (hex): "
            << std::hex << unsigned(versionA)
            << "." << std::hex
            << unsigned(versionB) << std::endl;

    // CMU CPLD version
    versionA = 0xff;
    versionB = 0xff;
    cpld::readReg(CMU_CPLD_BUSID, ADDR, CPLD_VER_A, &versionA);
    cpld::readReg(CMU_CPLD_BUSID, ADDR, CPLD_VER_B, &versionB);
    std::cout
            << "CMU CPLD version (hex): "
            << std::hex << unsigned(versionA)
            << "." << std::hex
            << unsigned(versionB) << std::endl;

    regData = 0xff;
    cpld::readReg(BUSID, ADDR, CPLD_VER_MATCH, &regData);
    std::cout
            << "CPLD version Match: "
            << ((regData) ? "False" : "True")
            << std::endl;
}

int main(int argc, char **argv) {
    if(argc != 1) {
        std::cout << argv[0] <<  "takes no arguments.\n";
        return 1;
    }
    std::cout << "This is project " << PROJECT_NAME << std::endl;

    uint8_t regData = 0;
    bool state;
    uint8_t regOffset;

    std::string infoMsg =
        "Start CPLD Manager Service.";
    log<level::INFO>(infoMsg.c_str());

    conn = std::make_shared<sdbusplus::asio::connection>(io);

    conn->request_name(hostDbusName.c_str());
    conn->request_name(hostDbusName1.c_str());

    sdbusplus::asio::object_server hostServer =
        sdbusplus::asio::object_server(conn);

    /* Expose platform states */
    platformStatesIfc =
        hostServer.add_interface("/xyz/openbmc_project/cpld/platform_states",
                                 "xyz.openbmc_project.Cpld.PlatformStates");

    // PowerOk
    regData = 0;
    if (cpld::readReg(BUSID, ADDR, SINGLE_BOARD_GENERAL_POWER_REG_0030, &regData))
    {
        log<level::ERR>("Failed to init PowerOK.");
    }
    regOffset = exposedProp[powerOkPN].regOffset;
    state = regData & regOffset ? true : false;
    states[powerOkPN] = state;
    platformStatesIfc->register_property(powerOkPN, state);

    // Cpu0FivrFault
    regData = 0;
    if (cpld::readReg(BUSID, ADDR, CPU_STATUS_REG_02A2, &regData))
    {
        log<level::ERR>("Failed to init Cpu0FivrFault.");
    }
    regOffset = exposedProp[cpu0FivrFaultPN].regOffset;
    state = regData & regOffset ? true : false;
    states[cpu0FivrFaultPN] = state;
    platformStatesIfc->register_property(cpu0FivrFaultPN, state);

    // Cpu1FivrFault
    regData = 0;
    if (cpld::readReg(BUSID, ADDR, CPU_STATUS_REG_02A9, &regData))
    {
        log<level::ERR>("Failed to init Cpu1FivrFault.");
    }
    regOffset = exposedProp[cpu1FivrFaultPN].regOffset;
    state = regData & regOffset ? true : false;
    states[cpu1FivrFaultPN] = state;
    platformStatesIfc->register_property(cpu1FivrFaultPN, state);

    // CpuCatErr
    regData = 0;
    if (cpld::readReg(BUSID, ADDR, CPU_STATUS_REG_02A1, &regData))
    {
        log<level::ERR>("Failed to init CpuCatErr.");
    }
    regOffset = exposedProp[cpuCatErrPN].regOffset;
    state = regData & regOffset ? true : false;
    states[cpuCatErrPN] = state;
    platformStatesIfc->register_property(cpuCatErrPN, state);

    // Cpu0MemThermalTrip/Cpu1MemThermalTrip
    regData = 0;
    if (cpld::readReg(BUSID, ADDR, MEM_STATUS_REG_0311, &regData))
    {
        log<level::ERR>("Failed to init Cpu0MemThermalTrip/Cpu1MemThermalTrip.");
    }
    regOffset = exposedProp[cpu0MemThermalTripPN].regOffset;
    state = regData & regOffset ? true : false;
    states[cpu0MemThermalTripPN] = state;
    platformStatesIfc->register_property(cpu0MemThermalTripPN, state);

    regOffset = exposedProp[cpu1MemThermalTripPN].regOffset;
    state = regData & regOffset ? true : false;
    states[cpu1MemThermalTripPN] = state;
    platformStatesIfc->register_property(cpu1MemThermalTripPN, state);

    // Cpu0VrHot/Cpu1VrHot
    regData = 0;
    if (cpld::readReg(BUSID, ADDR, POWER_ABNORMAL_REG_0090, &regData))
    {
        log<level::ERR>("Failed to init Cpu0VrHot/Cpu1VrHot.");
    }
    regOffset = exposedProp[cpu0VrHotPN].regOffset;
    state = regData & regOffset ? true : false;
    states[cpu0VrHotPN] = state;
    platformStatesIfc->register_property(cpu0VrHotPN, state);

    regOffset = exposedProp[cpu1VrHotPN].regOffset;
    state = regData & regOffset ? true : false;
    states[cpu1VrHotPN] = state;
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

    platformControlsIfc->register_method("BmcNmiCtrl", [](){
        log<level::INFO>("BmcNmiCtrl");
        if(cpld::writeReg(
            BUSID, ADDR, PCH_CONTROL_REG_03A9, BMC_NMI_CTL_BIT))
            log<level::ERR>("Failed to do BmcNmiCtrl.");
    });

    platformControlsIfc->register_method("ShortBtnPowerDownCtrl", [](){
        log<level::INFO>("ShortBtnPowerDownCtrl");
        if(cpld::writeReg(
            BUSID, ADDR,
            BUTTON_LED_CONTROL_REG_0130, BMC_SBTN_POWRDOWN_CTL_BIT))
            log<level::ERR>("Failed to do ShortBtnPowerDownCtrl.");
    });

    platformControlsIfc->register_method("LongBtnPowerDownCtrl", [](){
        log<level::INFO>("LongBtnPowerDownCtrl");
        if(cpld::writeReg(
            BUSID, ADDR,
            BUTTON_LED_CONTROL_REG_0130, BMC_LBTN_PWRDOWN_CTL_BIT))
            log<level::ERR>("Failed to do LongBtnPowerDownCtrl.");
    });

    platformControlsIfc->register_method("ShortBtnPowerOnCtrl", [](){
        log<level::INFO>("ShortBtnPowerOnCtrl");
        if(cpld::writeReg(
            BUSID, ADDR,
            BUTTON_LED_CONTROL_REG_0130, BMC_SBTN_POWRON_CTL_BIT))
            log<level::ERR>("Failed to do ShortBtnPowerOnCtrl.");
    });

    platformControlsIfc->register_method("ResetCtrl", [](){
        log<level::INFO>("ResetCtrl");
        if(cpld::writeReg(
            BUSID, ADDR,
            BUTTON_LED_CONTROL_REG_0130, BMC_SBTN_SYSRST_CTL_BIT))
            log<level::ERR>("Failed to do ResetCtrl.");
    });

    platformControlsIfc->register_method("ReadCpld", [](uint16_t regOffset){
        uint8_t data = 0;
        if(cpld::readReg(BUSID, ADDR, regOffset, &data))
            log<level::ERR>("Failed to call method ReadCpld.");
        return data;
    });

    platformControlsIfc->register_method("WriteCpld",
        [](uint16_t regOffset, uint8_t data){
            if(cpld::writeReg(BUSID, ADDR, regOffset, data))
                log<level::ERR>("Failed to call method WriteCpld.");
    });

    platformControlsIfc->initialize();

    io.post([&]() {
        poll();
    });

    // Assert BMC ready to CPLD here.
    // TODO: Shall be moved to pid fan control module
    gpiod::line gpioLine;
    setGPIOOutput("BMC_READY_OUT_N", 0, gpioLine);

    // sends BMC hearbeat to CPLD.
    // TODO: Shall be moved to pid fan control module
    if (setGPIOOutput("WDT_IN", 0, gpioLine))
    {
        io.post([&]() {
            heartbeat2CPLD(gpioLine);
        });
    }

    sanityCheck();

    io.run();

    return 0;
}
