#ifndef __PROCESSOR_H
#define __PROCESSOR_H

#include "Cache.h"
#include "Config.h"
#include "Memory.h"
#include "Request.h"
#include "Statistics.h"
#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <ctype.h>
#include <functional>

#include <queue>
#include <utility>
#include <thread>
#include <mutex>
#include <condition_variable>


namespace ramulator 
{

class Trace {
public:
    Trace( ) { }; // default constructor which does nothing
    explicit Trace(const char* trace_fname);
    // trace file format 1:
    // [# of bubbles(non-mem instructions)] [read address(dec or hex)] <optional: write address(evicted cacheline)>
    virtual bool get_unfiltered_request(long& bubble_cnt, long& req_addr, Request::Type& req_type);
    virtual bool get_filtered_request(long& bubble_cnt, long& req_addr, Request::Type& req_type);
    // trace file format 2:
    // [address(hex)] [R/W]
    virtual bool get_dramtrace_request(long& req_addr, Request::Type& req_type);
    // static auto make_trace(const char* trace_fname) -> std::unique_ptr<Trace> { return std::unique_ptr<Trace>(new Trace(trace_fname)); };
    static auto make_trace(const char* trace_fname) -> Trace* { return new Trace(trace_fname); };

private:
    std::ifstream file;
    std::string trace_name;
};

struct CPUTraceEntry {
    long bubble_cnt;
    long rd_addr;
    long wr_addr;
};

class TraceThread : public Trace {
public:
    TraceThread() { TraceThread::max_queue_size = std::numeric_limits<unsigned int>::max(); };
    TraceThread(unsigned int max_queue_size) { TraceThread::max_queue_size = max_queue_size; };
    TraceThread(int core_id, unsigned int max_queue_size) : core_id(core_id) { TraceThread::max_queue_size = max_queue_size; };
    bool get_dramtrace_request(long& req_addr, Request::Type& req_type) override;
    static void enqueue(long req_addr, Request::Type req_type);
    static void enqueue(long req_addr, const char* req_type);
    static void cpu_enqueue(int core, long bubble_cnt, long read_addr, long write_addr = -1);
    static void notify_end();
    static void notify_end(int core);
    static std::queue<std::pair<long, Request::Type> > q;
    static std::mutex mtx;
    static std::condition_variable cv, queueFull;
    static std::queue<std::pair<long, Request::Type> >::size_type max_queue_size;
    static std::vector<std::queue<CPUTraceEntry> > queue_list;
    static int total_cores;
    int core_id;
    bool get_unfiltered_request (long &bubble_cnt, long &req_addr, Request::Type &req_type) override;
    bool get_filtered_request (long &bubble_cnt, long &req_addr, Request::Type &req_type) override;
};


// Always interface wih this factory class when generating a CPU thread.
class TraceThreadFactory {
private:
    int total_cores;
    unsigned int max_queue_size;
public:
    TraceThreadFactory() { };
    TraceThreadFactory(int total_cores, unsigned int max_queue_size): total_cores(total_cores), max_queue_size(max_queue_size)
    {
        TraceThread::queue_list = std::vector<std::queue<CPUTraceEntry> > (total_cores, std::queue<CPUTraceEntry>());
        TraceThread::total_cores = total_cores;
    };

    auto make_trace_thread(int core_id) -> TraceThread*
    {
        assert (core_id >= 0 && core_id < total_cores);
        // TraceThread tt(core_id, max_queue_size / total_cores);
        // return std::unique_ptr<TraceThread>(new TraceThread(core_id, max_queue_size / total_cores + 1));
        return new TraceThread(core_id, max_queue_size / total_cores + 1);
    }
    
};

class Window {
public:
    int ipc = 4;
    int depth = 128;

    Window() : ready_list(depth), addr_list(depth, -1) {}
    bool is_full();
    bool is_empty();
    void insert(bool ready, long addr);
    long retire();
    void set_ready(long addr, int mask);

private:
    int load = 0;
    int head = 0;
    int tail = 0;
    std::vector<bool> ready_list;
    std::vector<long> addr_list;
};


class Core {
public:
    long clk = 0;
    long retired = 0;
    int id = 0;
    function<bool(Request)> send;

//    Core(const Config& configs, int coreid,
//        const char* trace_fname,
//        function<bool(Request)> send_next, Cache* llc,
//        std::shared_ptr<CacheSystem> cachesys, MemoryBase& memory);
    Core(const Config& configs, int coreid,
        Trace* trace,
        function<bool(Request)> send_next, Cache* llc,
        std::shared_ptr<CacheSystem> cachesys, MemoryBase& memory);
    void tick();
    void receive(Request& req);
    double calc_ipc();
    bool finished();
    bool has_reached_limit();
    function<void(Request&)> callback;

    bool no_core_caches = true;
    bool no_shared_cache = true;
    int l1_size = 1 << 15;
    int l1_assoc = 1 << 3;
    int l1_blocksz = 1 << 6;
    int l1_mshr_num = 16;

    int l2_size = 1 << 18;
    int l2_assoc = 1 << 3;
    int l2_blocksz = 1 << 6;
    int l2_mshr_num = 16;
    std::vector<std::shared_ptr<Cache>> caches;
    Cache* llc;

    ScalarStat record_cycs;
    ScalarStat record_insts;
    long expected_limit_insts;
    // This is set true iff expected number of instructions has been executed or all instructions are executed.
    bool reached_limit = false;;

private:
    std::unique_ptr<Trace> trace;
    Window window;

    long bubble_cnt;
    long req_addr = -1;
    Request::Type req_type;
    bool more_reqs;
    long last = 0;

    ScalarStat memory_access_cycles;
    ScalarStat cpu_inst;
    MemoryBase& memory;
};

class Processor {
public:
    Processor(const Config& configs, vector<const char*> trace_list,
        function<bool(Request)> send, MemoryBase& memory);
    Processor(const Config& configs, TraceThreadFactory& trace_thread_factory,
        function<bool(Request)> send, MemoryBase& memory);
    void tick();
    void receive(Request& req);
    bool finished();
    bool has_reached_limit();

    std::vector<std::unique_ptr<Core>> cores;
    std::vector<double> ipcs;
    double ipc = 0;

    // When early_exit is true, the simulation exits when the earliest trace finishes.
    bool early_exit;

    bool no_core_caches = true;
    bool no_shared_cache = true;

    int l3_size = 1 << 23;
    int l3_assoc = 1 << 3;
    int l3_blocksz = 1 << 6;
    int mshr_per_bank = 16;

    std::shared_ptr<CacheSystem> cachesys;
    Cache llc;

    ScalarStat cpu_cycles;
};

}
#endif /* __PROCESSOR_H */
