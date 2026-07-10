// Reconnect behavior of the UART data path after a device power-cycle,
// exercised on pseudo-terminals: closing the pty master reproduces the
// tty hangup a USB re-enumeration causes, and symlinks named ttyUSB* in
// a temp directory stand in for /dev so the AUTO scan can be driven
// through loss -> re-enumeration -> recovery deterministically.

#include <gtest/gtest.h>

#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>

#include <string>

#include "../src/anello_ros_driver/comm/anello_data_port.h"
#include "../src/anello_ros_driver/comm/serial_interface.h"

namespace
{

struct Pty
{
    int master = -1;
    std::string slave_path;

    bool open_pty()
    {
        master = posix_openpt(O_RDWR | O_NOCTTY);
        if (master < 0) return false;
        if (grantpt(master) != 0 || unlockpt(master) != 0) return false;
        char name[256];
        if (ptsname_r(master, name, sizeof(name)) != 0) return false;
        slave_path = name;
        return true;
    }

    void close_master()
    {
        if (master >= 0) {
            close(master);
            master = -1;
        }
    }

    ~Pty() { close_master(); }
};

// A blocking read may need a few attempts before the written bytes cross
// the pty; bounded pump instead of a single racy read.
size_t pump_until_data(serial_interface &port, char *buf, size_t buf_len)
{
    for (int i = 0; i < 50; ++i) {
        size_t n = port.get_data(buf, buf_len, 20);
        if (n > 0) return n;
    }
    return 0;
}

class TempPortDir : public ::testing::Test
{
protected:
    std::string dir_;

    void SetUp() override
    {
        char tmpl[] = "/tmp/anello_ports_XXXXXX";
        ASSERT_NE(mkdtemp(tmpl), nullptr);
        dir_ = std::string(tmpl) + "/";
    }

    void TearDown() override
    {
        // Best-effort cleanup of the symlinks and directory.
        std::string cmd = "rm -rf " + dir_;
        if (system(cmd.c_str()) != 0) {
            ADD_FAILURE() << "cleanup failed for " << dir_;
        }
    }

    void link_port(const std::string &target, const std::string &name)
    {
        ASSERT_EQ(symlink(target.c_str(), (dir_ + name).c_str()), 0);
    }

    void unlink_port(const std::string &name)
    {
        ASSERT_EQ(unlink((dir_ + name).c_str()), 0);
    }
};

}  // namespace

TEST(SerialInterface, HangupClosesPort)
{
    Pty pty;
    ASSERT_TRUE(pty.open_pty());

    serial_interface port;
    port.init(pty.slave_path, 230400);
    ASSERT_TRUE(port.get_port_enabled());

    ASSERT_EQ(write(pty.master, "hello", 5), 5);
    char buf[64];
    EXPECT_GT(pump_until_data(port, buf, sizeof(buf)), 0u);

    // Master gone == USB device gone: blocking reads must detect the
    // hangup and close the port instead of returning empty forever.
    pty.close_master();
    for (int i = 0; i < 50 && port.get_port_enabled(); ++i) {
        port.get_data(buf, sizeof(buf), 20);
    }
    EXPECT_FALSE(port.get_port_enabled());
    EXPECT_EQ(port.get_data(buf, sizeof(buf), 10), 0u);
}

TEST_F(TempPortDir, AutoRescanFindsReenumeratedDevice)
{
    Pty first, second;
    ASSERT_TRUE(first.open_pty());
    ASSERT_TRUE(second.open_pty());
    link_port(first.slave_path, "ttyUSB0");

    interface_config_t cfg;
    cfg.type = UART;
    cfg.data_port_name = "AUTO";
    cfg.baud_rate = 230400;

    anello_data_port port(&cfg, dir_);
    port.init();

    char buf[128];
    ASSERT_EQ(write(first.master, "#APX", 4), 4);
    size_t n = 0;
    for (int i = 0; i < 50 && n == 0; ++i) {
        n = port.get_data(buf, sizeof(buf), 10);
    }
    ASSERT_GT(n, 0u);
    port.port_confirm();  // what the node does after decoding a frame
    EXPECT_EQ(port.get_portname(), dir_ + "ttyUSB0");

    // Unit power-cycles: old node dies, device returns under a NEW name.
    first.close_master();
    unlink_port("ttyUSB0");
    link_port(second.slave_path, "ttyUSB1");

    // Pump the poll loop: hangup -> stream-lost -> rescan -> ttyUSB1.
    n = 0;
    for (int i = 0; i < 500 && n == 0; ++i) {
        n = port.get_data(buf, sizeof(buf), 10);
        if (n == 0 && second.master >= 0) {
            (void)!write(second.master, "#APY", 4);
        }
    }
    EXPECT_GT(n, 0u);
    port.port_confirm();
    EXPECT_EQ(port.get_portname(), dir_ + "ttyUSB1");
}

TEST_F(TempPortDir, AutoRescanSurvivesEmptyPortDir)
{
    Pty first, second;
    ASSERT_TRUE(first.open_pty());
    ASSERT_TRUE(second.open_pty());
    link_port(first.slave_path, "ttyUSB0");

    interface_config_t cfg;
    cfg.type = UART;
    cfg.data_port_name = "AUTO";
    cfg.baud_rate = 230400;

    anello_data_port port(&cfg, dir_);
    port.init();

    char buf[128];
    ASSERT_EQ(write(first.master, "#APX", 4), 4);
    ASSERT_GT([&] {
        size_t n = 0;
        for (int i = 0; i < 50 && n == 0; ++i) n = port.get_data(buf, sizeof(buf), 10);
        return n;
    }(), 0u);
    port.port_confirm();

    // Device unplugged entirely: no candidate ports at all for a while.
    first.close_master();
    unlink_port("ttyUSB0");
    for (int i = 0; i < 300; ++i) {
        EXPECT_EQ(port.get_data(buf, sizeof(buf), 10), 0u);
    }

    // Device comes back; the scan must pick it up.
    link_port(second.slave_path, "ttyUSB0");
    size_t n = 0;
    for (int i = 0; i < 500 && n == 0; ++i) {
        n = port.get_data(buf, sizeof(buf), 10);
        if (n == 0) {
            (void)!write(second.master, "#APY", 4);
        }
    }
    EXPECT_GT(n, 0u);
}

TEST_F(TempPortDir, FixedNamePortIsReopened)
{
    Pty first, second;
    ASSERT_TRUE(first.open_pty());
    ASSERT_TRUE(second.open_pty());
    link_port(first.slave_path, "ttyUSBfix");

    interface_config_t cfg;
    cfg.type = UART;
    cfg.data_port_name = dir_ + "ttyUSBfix";  // not AUTO
    cfg.baud_rate = 230400;

    anello_data_port port(&cfg, dir_);
    port.init();

    char buf[128];
    ASSERT_EQ(write(first.master, "#APX", 4), 4);
    size_t n = 0;
    for (int i = 0; i < 50 && n == 0; ++i) {
        n = port.get_data(buf, sizeof(buf), 10);
    }
    ASSERT_GT(n, 0u);
    port.port_confirm();

    // Power-cycle: the fixed path vanishes, then udev recreates it
    // (a /dev/serial/by-id symlink re-pointing to the new tty).
    first.close_master();
    unlink_port("ttyUSBfix");
    link_port(second.slave_path, "ttyUSBfix");

    n = 0;
    for (int i = 0; i < 1000 && n == 0; ++i) {
        n = port.get_data(buf, sizeof(buf), 10);
        if (n == 0 && second.master >= 0) {
            (void)!write(second.master, "#APY", 4);
        }
    }
    EXPECT_GT(n, 0u);
    EXPECT_EQ(port.get_portname(), dir_ + "ttyUSBfix");
}
