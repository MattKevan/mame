#include "datarover_core.h"
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <thread>
using namespace std::chrono_literals;
namespace fs = std::filesystem;
template<class F> bool wait_for(F predicate, std::chrono::seconds timeout) {
 auto end=std::chrono::steady_clock::now()+timeout;
 do { if(predicate()) return true; std::this_thread::sleep_for(20ms); } while(std::chrono::steady_clock::now()<end);
 return false;
}
int main(int argc,char**argv) {
 if(argc!=3) return 2;
 fs::path base(argv[2]); fs::create_directories(base/"cfg"); fs::create_directories(base/"nvram");
 auto create=[&]{return datarover_create((base/"nvram").c_str(),(base/"cfg").c_str(),argv[1]);};
 auto fail=[](const char*why){fprintf(stderr,"FAIL %s\n",why); std::fflush(nullptr); std::_Exit(1);};
 auto*c=create(); if(!c) fail("initial boot");
 // Allow the fresh ROM to reach its stable, touch-gated welcome screen. Wait
 // in emulated time: a freshly booted simulator can run several times slower.
 if(!wait_for([&]{return datarover_emulated_seconds(c)>=25.0;},std::chrono::seconds(75))) fail("guest did not reach the welcome screen");
 datarover_set_paused(c,1);
 if(!wait_for([&]{return datarover_save_status(c)==2;},3s)) fail("checkpoint save");
 datarover_destroy(c);
 c=create(); if(!c) fail("restore boot");
 auto revision=datarover_frame_revision(c);
 datarover_pen_down(c,240,160); datarover_pen_up(c);
 // Require a response before the three-second stuck-restore recovery can
 // hide broken input by cold booting instead.
 if(!wait_for([&]{return datarover_frame_revision(c)>revision;},2s)) fail("restored guest did not respond to the welcome tap");
 datarover_restart(c);
 if(!wait_for([&]{for(auto&e:fs::directory_iterator(base/"cfg")) if(e.path().filename().string().find(".restart-")!=std::string::npos) return true; return false;},3s)) fail("restart after restore did not leave the scheduler");
 if(!wait_for([&]{return datarover_framebuffer_bytes(c)!=nullptr;},5s)) fail("restart framebuffer");
 datarover_destroy(c);
 puts("PASS checkpoint restore, guest touch response and prompt restart");
}
