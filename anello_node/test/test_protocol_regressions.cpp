#include <gtest/gtest.h>
#include <cstring>
#include <random>
#include "../src/anello_ros_driver/messaging/protocol_decoder.h"
#include "../src/anello_ros_driver/navigation_math.h"
#include "../src/anello_ros_driver/sample_state.h"
#include "../src/anello_ros_driver/clock_translator.h"
using namespace anello;
namespace {
std::string ascii(const std::string &body) {
    return "#"+body+"*"+compute_checksum(body.data(),body.size());
}
std::vector<DecodedPacket> decode(const std::string &bytes, StreamDecoder &parser) {
    std::vector<DecodedPacket> out;
    for (unsigned char c:bytes) parser.feed(c,[&](const auto &p){out.push_back(p);});
    return out;
}
template<typename T> std::string binary(int subtype, const T &payload, int extra=0) {
    std::string bytes(sizeof(T)+8+extra,'\0');
    auto *data=reinterpret_cast<unsigned char *>(bytes.data());
    data[0]=0xd3;
    setbitu(data,14,10,sizeof(T)+2+extra); setbitu(data,24,12,4058); setbitu(data,36,4,subtype);
    std::memcpy(data+5,&payload,sizeof(T));
    setbitu(data,(bytes.size()-3)*8,24,crc24q(data,bytes.size()-3));
    return bytes;
}
const std::string imu="APIMU,1000,0,0,0,-1,1,2,3,4,0,999,20";
}
TEST(FullProtocol, ValidModernLegacyAndFragmented) {
    StreamDecoder parser;
    const auto line=ascii(imu)+"\r\n";
    EXPECT_TRUE(decode(line.substr(0,7),parser).empty());
    auto out=decode(line.substr(7),parser);
    ASSERT_EQ(out.size(),1u); EXPECT_EQ(out[0].kind,MessageKind::imu);
    EXPECT_DOUBLE_EQ(out[0].values[3],-1); EXPECT_DOUBLE_EQ(out[0].values[9],0.999);
    out=decode(ascii("APIMU,1000,0,0,-1,1,2,3,4,0,999,20")+"\r\n",parser);
    ASSERT_EQ(out.size(),1u); EXPECT_DOUBLE_EQ(out[0].values[3],-1);
}
TEST(FullProtocol, RejectsMalformedNumbersAndUnsupportedLayouts) {
    for (const auto &bad:{"nan","inf","-inf","1e309","12junk","", " 1","0x2"}) {
        auto body=imu; body.replace(body.find("1000"),4,bad);
        StreamDecoder parser; EXPECT_TRUE(decode(ascii(body)+"\r\n",parser).empty())<<bad;
    }
    for (const auto &body:{"APIMU,1000,0,0,-1,1,2,3,4,0", "APIMU,1000,0,0,0,-1,1,2,3,4,0,999,20,42",
                          "XXAPIMU,1000,0,0,0,-1,1,2,3,4,0,999,20"}) {
        StreamDecoder parser; EXPECT_TRUE(decode(ascii(body)+"\r\n",parser).empty());
    }
    StreamDecoder parser;
    EXPECT_TRUE(decode(ascii("APIMU,1000,0,nan,0,-1,1,2,3,4,0,999,20")+"\r\n",parser).empty());
}
TEST(FullProtocol, StatusAndOptionalVelocity) {
    for (const auto &status:{"255","1.5","-1"}) {
        StreamDecoder parser;
        EXPECT_TRUE(decode(ascii(std::string("APINS,1000,0,")+status+",37,-122,0,0,0,0,0,0,0,0")+"\r\n",parser).empty());
    }
    StreamDecoder parser;
    auto out=decode(ascii("APINS,1000,0,1,37,-122,0,,,,0,0,0,0")+"\r\n",parser);
    ASSERT_EQ(out.size(),1u); EXPECT_TRUE(std::isnan(out[0].values[6]));
    EXPECT_TRUE(ins_position_valid(out[0].values.data()));
    out=decode(ascii("APINS,1000,0,8,,,,,,,0,0,0,0")+"\r\n",parser);
    ASSERT_EQ(out.size(),1u); EXPECT_FALSE(ins_position_valid(out[0].values.data()));
    EXPECT_TRUE(decode(ascii("APINS,1000,0,1,91,-122,0,0,0,0,0,0,0,0")+"\r\n",parser).empty());
}
TEST(FullProtocol, ResyncAndErrorAccounting) {
    StreamDecoder parser;
    auto bad=ascii(imu); bad.back()=bad.back()=='0'?'1':'0';
    auto out=decode("junk#APbroken"+bad+"\r\n"+ascii(imu)+"\r\n",parser);
    EXPECT_EQ(out.size(),1u); EXPECT_EQ(parser.checksum_failures,1u); EXPECT_EQ(parser.parse_failures,1u);
    std::string reserved="\xd3\xfc\x01";
    out=decode(reserved+ascii(imu)+"\r\n",parser);
    EXPECT_EQ(out.size(),1u); EXPECT_EQ(parser.parse_failures,2u);
    EXPECT_EQ(decode(std::string(5000,'x')+ascii(imu)+"\r\n",parser).size(),1u);
}
TEST(FullProtocol, BinaryLengthCrcAndCovariance) {
    rtcm_apins_t ins{}; ins.Status=4; ins.Latitude=370000000; ins.Longitude=-1220000000;
    StreamDecoder parser;
    EXPECT_EQ(decode(binary(4,ins),parser).size(),1u);
    EXPECT_TRUE(decode(binary(4,ins,1),parser).empty());
    auto corrupt=binary(4,ins); corrupt.back()^=1;
    EXPECT_TRUE(decode(corrupt,parser).empty());
    EXPECT_EQ(parser.checksum_failures,1u);
    rtcm_apcov_t cov{}; cov.covLatLat=std::nanf("");
    EXPECT_TRUE(decode(binary(10,cov),parser).empty());
    cov.covLatLat=1; cov.covLonLon=1; cov.covLatLon=2;
    EXPECT_TRUE(decode(binary(10,cov),parser).empty());
    cov.covLatLon=0.25f;
    EXPECT_EQ(decode(binary(10,cov),parser).size(),1u);
}
TEST(FullProtocol, AhrsAndIm1) {
    StreamDecoder parser;
    auto out=decode(ascii("APAHRS,1000,1000000000,1,2,3,0")+"\r\n",parser);
    ASSERT_EQ(out.size(),1u); EXPECT_EQ(out[0].kind,MessageKind::ahrs);
    rtcm_apahrs_t a{1000000000,1000000000,100000,200000,300000,0};
    out=decode(binary(8,a),parser); ASSERT_EQ(out.size(),1u);
    EXPECT_DOUBLE_EQ(out[0].values[2],1);
    out=decode(ascii("APIM1,1000,0,0,0,-1,1,2,3,4,20")+"\r\n",parser);
    ASSERT_EQ(out.size(),1u); EXPECT_EQ(out[0].kind,MessageKind::im1);
}
TEST(FullProtocol, DeterministicNoiseAndBoundedRecovery) {
    std::mt19937 generator(42);
    StreamDecoder parser;
    for (int i=0;i<100000;++i) parser.feed(generator()%256,[](const auto &p){EXPECT_TRUE(validate_packet(p));});
    parser.reset();
    EXPECT_EQ(decode(ascii(imu)+"\r\n",parser).size(),1u);
}
TEST(NavigationMath, EquatorHeightAndAntimeridian) {
    LocalCartesian local; local.set_origin(0,0,10);
    auto p=local.position(0,0,15); EXPECT_NEAR(p.z(),5,1e-8); EXPECT_NEAR(p.length(),5,1e-8);
    local.set_origin(0,179.999,0); p=local.position(0,-179.999,0);
    EXPECT_NEAR(p.x(),222.63898154,1e-5); EXPECT_NEAR(p.y(),0,1e-8);
}
TEST(NavigationMath, CurvatureAndCurrentAxes) {
    LocalCartesian local; local.set_origin(37,-122,0);
    const double lon=-122+1000/89011.67172648299;
    auto p=local.position(37,lon,0);
    EXPECT_NEAR(p.x(),999.999994,0.001); EXPECT_NEAR(p.y(),0.059002,1e-5); EXPECT_NEAR(p.z(),-0.078298,1e-5);
    auto rotation=local.rotation(37,lon);
    EXPECT_NEAR(rotation.determinant(),1,1e-12);
    EXPECT_GT(std::abs(rotation[2][0]),1e-5);
}
TEST(NavigationMath, EulerCovarianceUsesAttitudeJacobian) {
    tf2::Matrix3x3 cov(1,0,0,0,2,0,0,0,3);
    auto out=euler_covariance_to_fixed(cov,0,3.14159265358979323846/2);
    EXPECT_NEAR(out[0][0],2,1e-12); EXPECT_NEAR(out[1][1],1,1e-12);
    out=euler_covariance_to_fixed(cov,0.5,0);
    EXPECT_NEAR(out[0][2],-std::cos(0.5)*std::sin(0.5),1e-12);
}
TEST(SampleState, RequiresBothAcquisitionAndArrivalFreshness) {
    SampleTime sample; const auto now=SteadyClock::now();
    EXPECT_FALSE(sample.matches(1000,0.1,now)); sample.set(1000,now);
    EXPECT_TRUE(sample.matches(1050,0.1,now)); EXPECT_FALSE(sample.matches(1200,0.1,now));
    EXPECT_FALSE(sample.matches(1000,0.1,now+std::chrono::seconds(1)));
}
TEST(SampleState, HostDeviceAndSimulationClockResets) {
    ClockDiscontinuity clock;
    EXPECT_FALSE(clock.update(5000,10000000000LL,0,false));
    EXPECT_FALSE(clock.update(5010,10010000000LL,10000000,false));
    EXPECT_TRUE(clock.update(5020,110020000000LL,20000000,false));
    EXPECT_TRUE(clock.update(5030,9000000000LL,30000000,false));
    EXPECT_TRUE(clock.update(1,9010000000LL,40000000,false));
    EXPECT_TRUE(clock.update(11,9020000000LL,50000000,true));
    EXPECT_FALSE(clock.update(21,9020000000LL,60000000,true));
    EXPECT_FALSE(clock.update(1021,9020000000LL,1060000000LL,true));
}
TEST(SampleState, IntegerHostEpochPreservesNanoseconds) {
    ClockTranslator clock;
    constexpr int64_t epoch=1788800000000000017LL;
    for (int i=0;i<100;++i) clock.update_ns(i*0.01,epoch+i*10000000LL);
    EXPECT_TRUE(clock.ready()); EXPECT_EQ(clock.translate_ns(0.99),epoch+990000000LL);
}
