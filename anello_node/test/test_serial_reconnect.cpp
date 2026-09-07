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
#include <thread>
#include <filesystem>
#include <poll.h>
#include <termios.h>
#include "../src/anello_ros_driver/comm/anello_config_port.h"

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

    void TearDown() override {
        std::error_code error;
        std::filesystem::remove_all(dir_,error);
        EXPECT_FALSE(error);
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
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
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
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
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
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        if (n == 0 && second.master >= 0) {
            (void)!write(second.master, "#APY", 4);
        }
    }
    EXPECT_GT(n, 0u);
    EXPECT_EQ(port.get_portname(), dir_ + "ttyUSBfix");
}

TEST_F(TempPortDir, ConfigChannelRecoversStableSymlink) {
    Pty first,second; ASSERT_TRUE(first.open_pty()); ASSERT_TRUE(second.open_pty());
    link_port(first.slave_path,"config");
    interface_config_t cfg; cfg.config_port_name=dir_+"config";
    anello_config_port port(&cfg); port.init();
    ASSERT_TRUE(port.write_data("hello",5));
    char bytes[32]; ASSERT_EQ(read(first.master,bytes,sizeof(bytes)),5);
    first.close_master(); unlink_port("config"); link_port(second.slave_path,"config");
    port.poll();
    for (int i=0;i<150 && !port.connected();++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); port.poll();
    }
    ASSERT_TRUE(port.connected()); ASSERT_TRUE(port.write_data("again",5));
    pollfd ready{second.master,POLLIN,0}; ASSERT_EQ(poll(&ready,1,100),1);
    ASSERT_EQ(read(second.master,bytes,sizeof(bytes)),5); EXPECT_EQ(std::string(bytes,5),"again");
}

TEST_F(TempPortDir, ConfigAutoAcceptsFragmentedReplyAfterCorruptLine) {
    Pty device; ASSERT_TRUE(device.open_pty());
    link_port(device.slave_path,"ttyUSB0");
    interface_config_t cfg; cfg.config_port_name="AUTO";
    anello_config_port port(&cfg,dir_); port.init();
    pollfd ready{device.master,POLLIN,0}; ASSERT_EQ(poll(&ready,1,100),1);
    char bytes[64]; const auto n=read(device.master,bytes,sizeof(bytes));
    ASSERT_GT(n,0); EXPECT_EQ(std::string(bytes,n),"#APPNG*48\r\n");
    const std::string first="#APPNG,0*00\r\n#APPNG,0*";
    ASSERT_EQ(write(device.master,first.data(),first.size()),static_cast<ssize_t>(first.size()));
    port.poll(); EXPECT_FALSE(port.connected());
    ASSERT_EQ(write(device.master,"54\r\n",4),4);
    for (int i=0;i<20 && !port.connected();++i) {
        port.poll(); std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    EXPECT_TRUE(port.connected());
}
TEST(SerialInterface, DisabledPortDoesNotClaimTransmission) {
    interface_config_t cfg; cfg.config_port_name="OFF";
    anello_config_port port(&cfg); port.init(); port.poll();
    EXPECT_FALSE(port.connected()); EXPECT_FALSE(port.write_data("test",4));
}
TEST(SerialInterface, BackpressureIsBoundedAndReported) {
    Pty pty; ASSERT_TRUE(pty.open_pty());
    serial_interface port; port.init(pty.slave_path,230400);
    std::string large(1024*1024,'x');
    const auto start=std::chrono::steady_clock::now();
    EXPECT_FALSE(port.write_data(large.data(),large.size()));
    EXPECT_LT(std::chrono::steady_clock::now()-start,std::chrono::milliseconds(500));
}
TEST(SerialInterface, PartialWritesCompleteWhenPeerDrains) {
    Pty pty; ASSERT_TRUE(pty.open_pty());
    serial_interface port; port.init(pty.slave_path,230400);
    std::string sent(128*1024,'x'), received;
    std::thread reader([&] {
        char bytes[4096];
        while (received.size()<sent.size()) {
            pollfd ready{pty.master,POLLIN,0}; if (poll(&ready,1,500)<=0) break;
            auto n=read(pty.master,bytes,sizeof(bytes)); if (n<=0) break;
            received.append(bytes,n);
        }
    });
    EXPECT_TRUE(port.write_data(sent.data(),sent.size())); reader.join();
    EXPECT_EQ(sent,received);
}
TEST(SerialInterface, DescriptorZeroIsClosed) {
    Pty pty; ASSERT_TRUE(pty.open_pty());
    const int saved=dup(0); close(0);
    serial_interface port; port.init(pty.slave_path,230400); port.close_port();
    const bool closed=fcntl(0,F_GETFD)==-1;
    if (saved>=0) { dup2(saved,0); close(saved); }
    EXPECT_TRUE(closed);
}

TEST_F(TempPortDir, SerialOwnershipRejectsAliasAndReleasesOnClose) {
    Pty pty; ASSERT_TRUE(pty.open_pty());
    link_port(pty.slave_path,"ttyUSB0"); link_port(pty.slave_path,"alias");
    serial_interface owner,other;
    owner.init(dir_+"ttyUSB0",230400);
    EXPECT_THROW(other.init(dir_+"alias",921600),std::runtime_error);
    EXPECT_TRUE(owner.write_data("still owned",11));
    char bytes[32]; pollfd ready{pty.master,POLLIN,0}; ASSERT_EQ(poll(&ready,1,100),1);
    ASSERT_EQ(read(pty.master,bytes,sizeof(bytes)),11);
    owner.close_port(); EXPECT_NO_THROW(other.init(dir_+"alias",230400));
}
TEST(SerialInterface, ClearsInheritedFlowControlAndHangupOnClose) {
    Pty pty; ASSERT_TRUE(pty.open_pty());
    int observer=open(pty.slave_path.c_str(),O_RDWR|O_NOCTTY);
    ASSERT_GE(observer,0);
    termios options{}; ASSERT_EQ(tcgetattr(observer,&options),0);
    options.c_cflag|=HUPCL; options.c_iflag|=IXON|IXOFF|IXANY;
    ASSERT_EQ(tcsetattr(observer,TCSANOW,&options),0);
    serial_interface port; port.init(pty.slave_path,230400);
    EXPECT_EQ(tcgetattr(observer,&options),0); EXPECT_EQ(options.c_cflag&HUPCL,0u);
    EXPECT_EQ(options.c_iflag&(IXON|IXOFF|IXANY),0u);
    port.close_port(); close(observer);
}
TEST(SerialInterface, OldGenerationCannotWriteToReplacementPort) {
    Pty first,second; ASSERT_TRUE(first.open_pty()); ASSERT_TRUE(second.open_pty());
    serial_interface port; port.init(first.slave_path,230400);
    const auto previous=port.generation(); port.init(second.slave_path,230400);
    EXPECT_FALSE(port.write_data("old correction",14,previous));
    pollfd ready{second.master,POLLIN,0}; EXPECT_EQ(poll(&ready,1,0),0);
}
TEST_F(TempPortDir, PortScanDoesNotReopenRapidlyOnGarbage) {
    Pty pty; ASSERT_TRUE(pty.open_pty()); link_port(pty.slave_path,"ttyUSB0");
    interface_config_t cfg; cfg.data_port_name="AUTO";
    anello_data_port port(&cfg,dir_); port.init(); const auto opened=port.generation();
    for (int i=0;i<1000;++i) port.port_parse_fail();
    EXPECT_EQ(port.generation(),opened);
    std::this_thread::sleep_for(std::chrono::milliseconds(550)); port.port_parse_fail();
    EXPECT_GT(port.generation(),opened);
}
TEST(SerialInterface, ConcurrentWritersDoNotInterleaveFrames) {
    Pty pty; ASSERT_TRUE(pty.open_pty());
    serial_interface port; port.init(pty.slave_path,230400);
    const std::string first(32768,'a'),second(32768,'b'); std::string received;
    std::thread reader([&] {
        char bytes[4096];
        while (received.size()<first.size()+second.size()) {
            pollfd ready{pty.master,POLLIN,0}; if (poll(&ready,1,500)<=0) break;
            const auto count=read(pty.master,bytes,sizeof(bytes)); if (count<=0) break;
            received.append(bytes,count);
        }
    });
    bool first_sent=false,second_sent=false;
    std::thread one([&] {first_sent=port.write_data(first.data(),first.size());});
    std::thread two([&] {second_sent=port.write_data(second.data(),second.size());});
    one.join(); two.join(); reader.join();
    EXPECT_TRUE(first_sent); EXPECT_TRUE(second_sent);
    EXPECT_TRUE(received==first+second || received==second+first);
}
