#ifndef __RAMULATOR_WRAPPER_H
#define __RAMULATOR_WRAPPER_H

#include "Processor.h"
#include "Config.h"
#include "Controller.h"
#include "SpeedyController.h"
#include "Memory.h"
#include "DRAM.h"
#include "Statistics.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdlib.h>
#include <functional>
#include <map>

/* Standards */
#include "Gem5Wrapper.h"
#include "DDR3.h"
#include "DDR4.h"
#include "DSARP.h"
#include "GDDR5.h"
#include "LPDDR3.h"
#include "LPDDR4.h"
#include "WideIO.h"
#include "WideIO2.h"
#include "HBM.h"
#include "SALP.h"
#include "ALDRAM.h"
#include "TLDRAM.h"

using namespace std;
using namespace ramulator;


namespace _RamulatorWrapper
{

template <typename T>
void run_dramtrace_on_thread(const Config &configs, Memory<T, Controller> &memory, TraceThreadFactory& trace_thread_factory)
{

    /* initialize DRAM trace */
    auto trace = trace_thread_factory.get_dram_trace_thread();

    /* run simulation */
    bool stall = false, end = false;
    int reads = 0, writes = 0, clks = 0;
    long addr = 0;
    Request::Type type = Request::Type::READ;
    map<int, int> latencies;
    auto read_complete = [&latencies](Request &r) { latencies[r.depart - r.arrive]++; };

    Request req(addr, type, read_complete);

    while (!end || memory.pending_requests())
    {
        if (!end && !stall)
        {
            end = !trace->get_dramtrace_request(addr, type);
        }

        if (!end)
        {
            req.addr = addr;
            req.type = type;
            stall = !memory.send(req);
            if (!stall)
            {
                if (type == Request::Type::READ)
                    reads++;
                else if (type == Request::Type::WRITE)
                    writes++;
            }
        }
        memory.tick();
        clks++;
        Stats::curTick++; // memory clock, global, for Statistics
    }
    // This a workaround for statistics set only initially lost in the end
    memory.finish();
    Stats::statlist.printall();
}

template <typename T>
void run_cputrace(const Config &configs, Memory<T, Controller> &memory, const int num_cores, TraceThreadFactory& trace_thread_factory)
{
    int cpu_tick = configs.get_cpu_tick();
    int mem_tick = configs.get_mem_tick();
    auto send = bind(&Memory<T, Controller>::send, &memory, placeholders::_1);
    Processor proc(configs, trace_thread_factory, send, memory);
    for (long i = 0;; i++)
    {
        proc.tick();
        Stats::curTick++; // processor clock, global, for Statistics
        if (i % cpu_tick == (cpu_tick - 1))
            for (int j = 0; j < mem_tick; j++)
                memory.tick();
        if (configs.calc_weighted_speedup())
        {
            if (proc.has_reached_limit())
            {
                break;
            }
        }
        else
        {
            if (configs.is_early_exit())
            {
                if (proc.finished())
                    break;
            }
            else
            {
                if (proc.finished() && (memory.pending_requests() == 0))
                    break;
            }
        }
    }
    // This a workaround for statistics set only initially lost in the end
    memory.finish();
    Stats::statlist.printall();
}

template <typename T>
void start_run(const Config &configs, T *spec, TraceThreadFactory& trace_thread_factory)
{
    // initiate controller and memory
    int C = configs.get_channels(), R = configs.get_ranks();
    // Check and Set channel, rank number
    spec->set_channel_number(C);
    spec->set_rank_number(R);
    std::vector<Controller<T> *> ctrls;
    for (int c = 0; c < C; c++)
    {
        DRAM<T> *channel = new DRAM<T>(spec, T::Level::Channel);
        channel->id = c;
        channel->regStats("");
        Controller<T> *ctrl = new Controller<T>(configs, channel);
        ctrls.push_back(ctrl);
    }
    Memory<T, Controller> memory(configs, ctrls);

    //   assert(trace_thread_factory.size() != 0);
    if (configs["trace_type"] == "CPU")
    {
        run_cputrace(configs, memory, configs.get_core_num(), trace_thread_factory);
    }
    else if (configs["trace_type"] == "DRAM")
    {
        // run_dramtrace(configs, memory, trace_thread_factory[0]);
        //****************************************//
        run_dramtrace_on_thread(configs, memory, trace_thread_factory);
        //std::thread worker(run_dramtrace_on_thread<T>, std::cref(configs), std::ref(memory));
        // std::thread test(sample_producer_thread, trace_thread_factory[0]);
        //worker.join();
        //test.join();
    }
}

void init(TraceThreadFactory& trace_thread_factory, const char *config_filename, const char *trace_type = "dram", bool stats = false, const char *stats_filename = "",
           int core_num = 1)
{
    if (false)
    {
        // printf("Usage: %s <configs-file> --mode=cpu,dram [--stats <filename>] <trace-filename1> <trace-filename2>\n"
        // "Example: %s ramulator-configs.cfg --mode=cpu cpu.trace cpu.trace\n", argv[0], argv[0]);
    }

    Config configs(config_filename);

    const std::string &standard = configs["standard"];
    assert(standard != "" || "DRAM standard should be specified.");

    // const char *trace_type = strstr(argv[2], "=");
    // trace_type++;
    if (strcmp(trace_type, "cpu") == 0)
    {
        configs.add("trace_type", "CPU");
    }
    else if (strcmp(trace_type, "dram") == 0)
    {
        configs.add("trace_type", "DRAM");
    }
    else
    {
        printf("invalid trace type: %s\n", trace_type);
        assert(false);
    }

    // int trace_start = 3;
    string stats_out;
    if (stats)
    {
        Stats::statlist.output(stats_filename);
        stats_out = stats_filename;
        // trace_start = 5;
    }
    else
    {
        Stats::statlist.output(standard + ".stats");
        stats_out = standard + string(".stats");
    }
    // configs.set_core_num(argc - trace_start);
    // configs.set_core_num(1);
    configs.set_core_num(core_num);

    if (standard == "DDR3")
    {
        DDR3 *ddr3 = new DDR3(configs["org"], configs["speed"]);
        start_run(configs, ddr3, trace_thread_factory);
    }
    else if (standard == "DDR4")
    {
        DDR4 *ddr4 = new DDR4(configs["org"], configs["speed"]);
        start_run(configs, ddr4, trace_thread_factory);
    }
    else if (standard == "SALP-MASA")
    {
        SALP *salp8 = new SALP(configs["org"], configs["speed"], "SALP-MASA", configs.get_subarrays());
        start_run(configs, salp8, trace_thread_factory);
    }
    else if (standard == "LPDDR3")
    {
        LPDDR3 *lpddr3 = new LPDDR3(configs["org"], configs["speed"]);
        start_run(configs, lpddr3, trace_thread_factory);
    }
    else if (standard == "LPDDR4")
    {
        // total cap: 2GB, 1/2 of others
        LPDDR4 *lpddr4 = new LPDDR4(configs["org"], configs["speed"]);
        start_run(configs, lpddr4, trace_thread_factory);
    }
    else if (standard == "GDDR5")
    {
        GDDR5 *gddr5 = new GDDR5(configs["org"], configs["speed"]);
        start_run(configs, gddr5, trace_thread_factory);
    }
    else if (standard == "HBM")
    {
        HBM *hbm = new HBM(configs["org"], configs["speed"]);
        start_run(configs, hbm, trace_thread_factory);
    }
    else if (standard == "WideIO")
    {
        // total cap: 1GB, 1/4 of others
        WideIO *wio = new WideIO(configs["org"], configs["speed"]);
        start_run(configs, wio, trace_thread_factory);
    }
    else if (standard == "WideIO2")
    {
        // total cap: 2GB, 1/2 of others
        WideIO2 *wio2 = new WideIO2(configs["org"], configs["speed"], configs.get_channels());
        wio2->channel_width *= 2;
        start_run(configs, wio2, trace_thread_factory);
    }
    // Various refresh mechanisms
    else if (standard == "DSARP")
    {
        DSARP *dsddr3_dsarp = new DSARP(configs["org"], configs["speed"], DSARP::Type::DSARP, configs.get_subarrays());
        start_run(configs, dsddr3_dsarp, trace_thread_factory);
    }
    else if (standard == "ALDRAM")
    {
        ALDRAM *aldram = new ALDRAM(configs["org"], configs["speed"]);
        start_run(configs, aldram, trace_thread_factory);
    }
    else if (standard == "TLDRAM")
    {
        TLDRAM *tldram = new TLDRAM(configs["org"], configs["speed"], configs.get_subarrays());
        start_run(configs, tldram, trace_thread_factory);
    }

    printf("[RamulatorWrapper] Simulation done. Statistics written to %s\n", stats_out.c_str());
}

}

using namespace _RamulatorWrapper;

class RamulatorWrapper
{
private:
    std::thread worker;
    const int max_queue_size = 10000000;
    TraceThreadFactory trace_thread_factory;

public:
    // DRAM mode constructor
    RamulatorWrapper(const char *config_filename, const char *stats_filename) : trace_thread_factory(max_queue_size) {
        worker = std::thread(init, std::ref(trace_thread_factory), config_filename, "dram", true, stats_filename, 1);
    }
    RamulatorWrapper(const char *config_filename): trace_thread_factory(max_queue_size)  {
        worker = std::thread(init, std::ref(trace_thread_factory), config_filename, "dram", false, "", 1);
    }
    // CPU mode constructor
    RamulatorWrapper(const char *config_filename, const char *stats_filename, int num_cores) : trace_thread_factory(num_cores, max_queue_size) {
        worker = std::thread(init, std::ref(trace_thread_factory), config_filename, "cpu", true, stats_filename, num_cores);
    }
    RamulatorWrapper(const char *config_filename, int num_cores) : trace_thread_factory(num_cores, max_queue_size) {
        worker = std::thread(init, std::ref(trace_thread_factory), config_filename, "cpu", false, "", num_cores);
    }
    ~RamulatorWrapper() {
        notify_end();
        worker.join();
    }
    // DRAM API
    void enqueue(long req_addr, const char* req_type) { trace_thread_factory.dram_enqueue(req_addr, req_type); }
    // CPU API
    void enqueue(int core, long bubble_cnt, long read_addr, long write_addr = -1) { trace_thread_factory.cpu_enqueue(core, bubble_cnt, read_addr, write_addr); };
    void cpu_enqueue(int core, long bubble_cnt, long read_addr, long write_addr = -1) { trace_thread_factory.cpu_enqueue(core, bubble_cnt, read_addr, write_addr); };
    void notify_end(int core) { trace_thread_factory.notify_end(core); }
    // For explicitly terminate the ramulator in simulation. This is automatically called from destructor.
    void notify_end() { trace_thread_factory.notify_end(); }
};

#endif
