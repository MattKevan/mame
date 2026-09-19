#include "datarover_core.h"
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <thread>
#include <ctime>
#include <fstream>
int main(int argc, char **argv) {
 if(argc != 3) return 2;
 std::filesystem::path base(argv[2]);
 std::filesystem::create_directories(base/"cfg");
 std::filesystem::create_directories(base/"nvram");
 auto *core=datarover_create((base/"nvram").c_str(),(base/"cfg").c_str(),argv[1]);
 if(!core) return 3;
 datarover_set_option(core,0,1); datarover_set_option(core,1,1);
 datarover_set_option(core,0,0); datarover_set_option(core,1,0);
 std::this_thread::sleep_for(std::chrono::seconds(2));
 datarover_set_paused(core,1);
 auto limit=std::chrono::steady_clock::now()+std::chrono::seconds(8);
 while(datarover_save_status(core)==1 && std::chrono::steady_clock::now()<limit) std::this_thread::sleep_for(std::chrono::milliseconds(50));
 if(datarover_save_status(core)!=2) return 4;
 auto revision=datarover_frame_revision(core);
 auto cpu=std::clock();
 std::this_thread::sleep_for(std::chrono::milliseconds(300));
 if(datarover_frame_revision(core)!=revision) return 5;
 double idle_cpu=double(std::clock()-cpu)/CLOCKS_PER_SEC;
 std::printf("Paused CPU seconds over 300ms: %.4f\n",idle_cpu);
 if(idle_cpu>0.10) return 8;
 if(!std::filesystem::exists(base/"cfg"/"session.sta")) return 6;
 datarover_set_paused(core,0); datarover_restart(core);
 std::this_thread::sleep_for(std::chrono::seconds(1));
 datarover_destroy(core);
 core=datarover_create((base/"nvram").c_str(),(base/"cfg").c_str(),argv[1]);
 if(!core || !datarover_framebuffer_bytes(core)) return 7;
 datarover_set_paused(core,1);
 std::this_thread::sleep_for(std::chrono::milliseconds(250));
 datarover_destroy(core);
 std::ofstream corrupt(base/"cfg"/"session.sta",std::ios::binary|std::ios::trunc); corrupt << "truncated"; corrupt.close();
 core=datarover_create((base/"nvram").c_str(),(base/"cfg").c_str(),argv[1]);
 if(!core || !datarover_framebuffer_bytes(core)) return 9;
 datarover_destroy(core);
 std::puts("PASS controls, pause, checkpoint, restart, resume and corrupt-save fallback");
}
