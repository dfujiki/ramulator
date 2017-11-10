#include "RamulatorWrapper.h"
#include <chrono>

#ifdef _WRAPPER_TEST
void test() {
    using namespace std::this_thread;
    using namespace std::chrono;
    
    RamulatorWrapper RW("configs/DDR3-config.cfg");
    
    // Trace trace(tracename);
    long addr = 0;
    // Request::Type type = Request::Type::READ;
    for (int i = 0; i < 3; i++)
    {
        // end = !trace.get_dramtrace_request(addr, type);
        addr = 0x12345680; 
        RW.enqueue(addr, "R");
        std::cout << "Enqueue: " << addr << " " << std::endl;
        sleep_until(system_clock::now() + seconds(1));
    }

    // You don't have to call notify end because RW is RAII.
    // Thread is terminated when the object is destructed. 
    RW.notify_end();
}

void test2() {
    RamulatorWrapper RW("configs/DDR3-config.cfg", 2);
    RW.cpu_enqueue(0,3, 20734016);
    RW.cpu_enqueue(0,1, 20846400);
    RW.cpu_enqueue(0,6, 20734208);
    RW.cpu_enqueue(0,8, 20841280, 20841280);
    RW.cpu_enqueue(0,0, 20734144);
    RW.cpu_enqueue(0,2, 20918976, 20734016);
    RW.cpu_enqueue(1,3, 20734016);
    RW.cpu_enqueue(1,1, 20846400);
    RW.cpu_enqueue(1,6, 20734208);
    RW.cpu_enqueue(1,8, 20841280, 20841280);
    RW.cpu_enqueue(1,0, 20734144);
    RW.cpu_enqueue(1,2, 20918976, 20734016);
}

int main() {test2(); return 0;}
#endif

#if 0
#include <chrono>
void sample_producer_thread(const char* tracename) {
    using namespace std::this_thread;
    using namespace std::chrono;
    Trace trace(tracename);
    bool end = false;
    long addr = 0;
    Request::Type type = Request::Type::READ;
    while (!end) {
        end = !trace.get_dramtrace_request(addr, type);
        TraceThread::enqueue(addr, type);
        std::cout << "Enqueue: " << addr << " " << std::endl;
        sleep_until(system_clock::now() + seconds(1));
    }
    TraceThread::notify_end();
}

template<typename T>
void run_dramtrace_on_thread(const Config& configs, Memory<T, Controller>& memory) {

    /* initialize DRAM trace */
    TraceThread trace;

    /* run simulation */
    bool stall = false, end = false;
    int reads = 0, writes = 0, clks = 0;
    long addr = 0;
    Request::Type type = Request::Type::READ;
    map<int, int> latencies;
    auto read_complete = [&latencies](Request& r){latencies[r.depart - r.arrive]++;};

    Request req(addr, type, read_complete);

    while (!end || memory.pending_requests()){
        if (!end && !stall){
            end = !trace.get_dramtrace_request(addr, type);
        }

        if (!end){
            req.addr = addr;
            req.type = type;
            stall = !memory.send(req);
            if (!stall){
                if (type == Request::Type::READ) reads++;
                else if (type == Request::Type::WRITE) writes++;
            }
        }
        memory.tick();
        clks ++;
        Stats::curTick++; // memory clock, global, for Statistics
    }
    // This a workaround for statistics set only initially lost in the end
    memory.finish();
    Stats::statlist.printall();
}
#endif