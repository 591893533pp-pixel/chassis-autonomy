#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>
#include <thread>
#include <stdexcept>
#include "elmue_can/elmue_can.h"
#include "elmue_can/robstride_protocol.hpp"
#define private public
#include "elmue_can/robstride_elmue.h"
#undef private
static void require(bool v){if(!v)throw std::runtime_error("feedback snapshot test failed");}
int main(){
 elmue_can::RobstrideElmueDriver d;
 require(!d.getMotorFeedback(1).is_valid);
 uint8_t data[8]={0x80,0,0x80,0,0x80,0,1,0};
 d.parseFeedbackFrame((24u<<24)|(1u<<8)|0x1f,data);
 auto first=d.getMotorFeedback(1);require(first.is_valid);
 std::this_thread::sleep_for(std::chrono::milliseconds(110));
 auto old=d.getMotorFeedback(1);
 require(old.last_update==first.last_update);
 require(std::chrono::steady_clock::now()-old.last_update>std::chrono::milliseconds(100));
 std::atomic<bool> done{false};
 std::thread writer([&]{for(int i=0;i<100000;i++)d.parseFeedbackFrame((24u<<24)|(1u<<8)|0x1f,data);done=true;});
 bool valid=true;int count=0;
 do{auto f=d.getMotorFeedback(1);valid=valid&&f.is_valid&&f.motor_id==1&&f.velocity==first.velocity&&f.temperature==first.temperature;++count;}while(!done||count<100000);
 writer.join();require(valid);require(!d.getMotorFeedback(2).is_valid);
}
