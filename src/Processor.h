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
    TraceThread(std::mutex& mtx, std::condition_variable& cv, std::condition_variable& queueFull, unsigned int max_queue_size)
               : mtx(mtx), cv(cv), queueFull(queueFull), max_queue_size(max_queue_size) { };

protected:
    std::mutex& mtx;
    std::condition_variable& cv;
    std::condition_variable& queueFull;
    unsigned int max_queue_size = std::numeric_limits<unsigned int>::max();
};


class DRAMTraceThread : public TraceThread {
public:
    DRAMTraceThread(std::mutex& mtx, std::condition_variable& cv, std::condition_variable& queueFull, unsigned int max_queue_size,
                    std::queue<std::pair<long, Request::Type> >& q)
                   : TraceThread(mtx, cv, queueFull, max_queue_size), q(q) { };
    bool get_dramtrace_request(long& req_addr, Request::Type& req_type) override;

private: 
    std::queue<std::pair<long, Request::Type> >& q;
};


class CPUTraceThread : public TraceThread {
public:
    CPUTraceThread(std::mutex& mtx, std::condition_variable& cv, std::condition_variable& queueFull, unsigned int max_queue_size,
                    std::queue<CPUTraceEntry>& lq)
                   : TraceThread(mtx, cv, queueFull, max_queue_size), lq(lq) { };
    bool get_unfiltered_request (long &bubble_cnt, long &req_addr, Request::Type &req_type) override;
    bool get_filtered_request (long &bubble_cnt, long &req_addr, Request::Type &req_type) override;

private:
    std::queue<CPUTraceEntry>& lq;
};


// Always interface wih this factory class when generating a trace thread.
class TraceThreadFactory {
public:
    TraceThreadFactory() = delete;
    // DRAM Mode
    TraceThreadFactory(unsigned int max_queue_size) 
                      : max_queue_size(max_queue_size)
                      , dram_trace_thread(new DRAMTraceThread(mtx, cv, queueFull, max_queue_size, q)){ };
    // CPU Mode
    TraceThreadFactory(int total_cores, unsigned int max_queue_size)
                      : max_queue_size(max_queue_size)
                      , queue_list(total_cores, std::queue<CPUTraceEntry>())
                      , total_cores(total_cores) { };

    auto make_cpu_trace_thread(int core_id) -> CPUTraceThread*;
    auto get_dram_trace_thread() -> std::unique_ptr<DRAMTraceThread>;

    void dram_enqueue(long req_addr, Request::Type req_type);
    void dram_enqueue(long req_addr, const char* req_type);
    void cpu_enqueue(int core, long bubble_cnt, long read_addr, long write_addr = -1);
    void notify_end();
    void notify_end(int core);

private:
    std::mutex mtx;
    std::condition_variable cv;
    std::condition_variable queueFull;
    unsigned int max_queue_size;
    // DRAM Mode
    std::unique_ptr<DRAMTraceThread> dram_trace_thread; // DRAMTraceThread is a singleton
    std::queue<std::pair<long, Request::Type> > q;
    // CPU Mode
    std::vector<std::queue<CPUTraceEntry> > queue_list;
    int total_cores = 0;  // DRAM mode = 0, CPU mode > 0
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
