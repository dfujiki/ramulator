#include "Processor.h"
#include <cassert>

using namespace std;
using namespace ramulator;

Processor::Processor(const Config& configs,
    vector<const char*> trace_list,
    function<bool(Request)> send_memory,
    MemoryBase& memory)
    : ipcs(trace_list.size(), -1),
    early_exit(configs.is_early_exit()),
    no_core_caches(!configs.has_core_caches()),
    no_shared_cache(!configs.has_l3_cache()),
    cachesys(new CacheSystem(configs, send_memory)),
    llc(l3_size, l3_assoc, l3_blocksz,
         mshr_per_bank * trace_list.size(),
         Cache::Level::L3, cachesys) {

  assert(cachesys != nullptr);
  int tracenum = trace_list.size();
  assert(tracenum > 0);
  printf("tracenum: %d\n", tracenum);
  for (int i = 0 ; i < tracenum ; ++i) {
    printf("trace_list[%d]: %s\n", i, trace_list[i]);
  }
  if (no_shared_cache) {
    for (int i = 0 ; i < tracenum ; ++i) {
      cores.emplace_back(new Core(
          configs, i, Trace::make_trace(trace_list[i]), send_memory, nullptr,
          cachesys, memory));
    }
  } else {
    for (int i = 0 ; i < tracenum ; ++i) {
      cores.emplace_back(new Core(configs, i, Trace::make_trace(trace_list[i]),
          std::bind(&Cache::send, &llc, std::placeholders::_1),
          &llc, cachesys, memory));
    }
  }
  for (int i = 0 ; i < tracenum ; ++i) {
    cores[i]->callback = std::bind(&Processor::receive, this,
        placeholders::_1);
  }

  // regStats
  cpu_cycles.name("cpu_cycles")
            .desc("cpu cycle number")
            .precision(0)
            ;
  cpu_cycles = 0;
}

Processor::Processor(const Config& configs,
    TraceThreadFactory& trace_thread_factory,
    function<bool(Request)> send_memory,
    MemoryBase& memory)
    : ipcs(configs.get_core_num(), -1),
    early_exit(configs.is_early_exit()),
    no_core_caches(!configs.has_core_caches()),
    no_shared_cache(!configs.has_l3_cache()),
    cachesys(new CacheSystem(configs, send_memory)),
    llc(l3_size, l3_assoc, l3_blocksz,
         mshr_per_bank * configs.get_core_num(),
         Cache::Level::L3, cachesys) {

  int tracenum = configs.get_core_num();
  assert(cachesys != nullptr);
  assert(tracenum > 0);
  printf("tracenum: %d\n", tracenum);

  // TraceThreadFactory trace_thread_factory(tracenum, 10000000);
  if (no_shared_cache) {
    for (int i = 0 ; i < tracenum ; ++i) {
      cores.emplace_back(new Core(
          configs, i, trace_thread_factory.make_cpu_trace_thread(i), send_memory, nullptr,
          cachesys, memory));
    }
  } else {
    for (int i = 0 ; i < tracenum ; ++i) {
      cores.emplace_back(new Core(configs, i, trace_thread_factory.make_cpu_trace_thread(i),
          std::bind(&Cache::send, &llc, std::placeholders::_1),
          &llc, cachesys, memory));
    }
  }
  for (int i = 0 ; i < tracenum ; ++i) {
    cores[i]->callback = std::bind(&Processor::receive, this,
        placeholders::_1);
  }

  // regStats
  cpu_cycles.name("cpu_cycles")
            .desc("cpu cycle number")
            .precision(0)
            ;
  cpu_cycles = 0;
}

void Processor::tick() {
  cpu_cycles++;
  if (!(no_core_caches && no_shared_cache)) {
    cachesys->tick();
  }
  for (unsigned int i = 0 ; i < cores.size() ; ++i) {
    Core* core = cores[i].get();
    core->tick();
  }
}

void Processor::receive(Request& req) {
  if (!no_shared_cache) {
    llc.callback(req);
  } else if (!cores[0]->no_core_caches) {
    // Assume all cores have caches or don't have caches
    // at the same time.
    for (unsigned int i = 0 ; i < cores.size() ; ++i) {
      Core* core = cores[i].get();
      core->caches[0]->callback(req);
    }
  }
  for (unsigned int i = 0 ; i < cores.size() ; ++i) {
    Core* core = cores[i].get();
    core->receive(req);
  }
}

bool Processor::finished() {
  if (early_exit) {
    for (unsigned int i = 0 ; i < cores.size(); ++i) {
      if (cores[i]->finished()) {
        for (unsigned int j = 0 ; j < cores.size() ; ++j) {
          ipc += cores[j]->calc_ipc();
        }
        return true;
      }
    }
    return false;
  } else {
    for (unsigned int i = 0 ; i < cores.size(); ++i) {
      if (!cores[i]->finished()) {
        return false;
      }
      if (ipcs[i] < 0) {
        ipcs[i] = cores[i]->calc_ipc();
        ipc += ipcs[i];
      }
    }
    return true;
  }
}

bool Processor::has_reached_limit() {
  for (unsigned int i = 0 ; i < cores.size() ; ++i) {
    if (!cores[i]->has_reached_limit()) {
      return false;
    }
  }
  return true;
}

Core::Core(const Config& configs, int coreid,
    Trace *trace,
    function<bool(Request)> send_next,
    Cache* llc, std::shared_ptr<CacheSystem> cachesys, MemoryBase& memory)
    : id(coreid), no_core_caches(!configs.has_core_caches()),
    no_shared_cache(!configs.has_l3_cache()),
    llc(llc), trace(trace), memory(memory)
{
  // Build cache hierarchy
  if (no_core_caches) {
    send = send_next;
  } else {
    // L2 caches[0]
    caches.emplace_back(new Cache(
        l2_size, l2_assoc, l2_blocksz, l2_mshr_num,
        Cache::Level::L2, cachesys));
    // L1 caches[1]
    caches.emplace_back(new Cache(
        l1_size, l1_assoc, l1_blocksz, l1_mshr_num,
        Cache::Level::L1, cachesys));
    send = bind(&Cache::send, caches[1].get(), placeholders::_1);
    if (llc != nullptr) {
      caches[0]->concatlower(llc);
    }
    caches[1]->concatlower(caches[0].get());
  }
  if (no_core_caches) {
    more_reqs = trace->get_filtered_request(
        bubble_cnt, req_addr, req_type);
    if (req_addr != -1)
        req_addr = memory.page_allocator(req_addr, id);
  } else {
    more_reqs = trace->get_unfiltered_request(
        bubble_cnt, req_addr, req_type);
    if (req_addr != -1)
        req_addr = memory.page_allocator(req_addr, id);
  }

  // set expected limit instruction for calculating weighted speedup
  expected_limit_insts = configs.get_expected_limit_insts();

  // regStats
  record_cycs.name("record_cycs_core_" + to_string(id))
             .desc("Record cycle number for calculating weighted speedup. (Only valid when expected limit instruction number is non zero in config file.)")
             .precision(0)
             ;

  record_insts.name("record_insts_core_" + to_string(id))
              .desc("Retired instruction number when record cycle number. (Only valid when expected limit instruction number is non zero in config file.)")
              .precision(0)
              ;

  memory_access_cycles.name("memory_access_cycles_core_" + to_string(id))
                      .desc("memory access cycles in memory time domain")
                      .precision(0)
                      ;
  memory_access_cycles = 0;
  cpu_inst.name("cpu_instructions_core_" + to_string(id))
          .desc("cpu instruction number")
          .precision(0)
          ;
  cpu_inst = 0;
}


double Core::calc_ipc()
{
    printf("[%d]retired: %ld, clk, %ld\n", id, retired, clk);
    return (double) retired / clk;
}

void Core::tick()
{
    clk++;

    retired += window.retire();

    if (expected_limit_insts == 0 && !more_reqs) return;

    // bubbles (non-memory operations)
    int inserted = 0;
    while (bubble_cnt > 0) {
        if (inserted == window.ipc) return;
        if (window.is_full()) return;

        window.insert(true, -1);
        inserted++;
        bubble_cnt--;
        cpu_inst++;
        if (long(cpu_inst.value()) == expected_limit_insts && !reached_limit) {
          record_cycs = clk;
          record_insts = long(cpu_inst.value());
          memory.record_core(id);
          reached_limit = true;
        }
    }

    if (req_type == Request::Type::READ) {
        // read request
        if (inserted == window.ipc) return;
        if (window.is_full()) return;

        Request req(req_addr, req_type, callback, id);
        if (!send(req)) return;

        window.insert(false, req_addr);
        cpu_inst++;
    }
    else if (req_type == Request::Type::WRITE) {
        // write request
        // assert(req_type == Request::Type::WRITE);
        Request req(req_addr, req_type, callback, id);
        if (!send(req)) return;
        cpu_inst++;
    } else {
        assert(req_type == Request::Type::IDLE);
    }
    if (long(cpu_inst.value()) == expected_limit_insts && !reached_limit) {
      record_cycs = clk;
      record_insts = long(cpu_inst.value());
      memory.record_core(id);
      reached_limit = true;
    }

    if (no_core_caches) {
      more_reqs = trace->get_filtered_request(
          bubble_cnt, req_addr, req_type);
      if (req_addr != -1) {
        req_addr = memory.page_allocator(req_addr, id);
      }
    } else {
      more_reqs = trace->get_unfiltered_request(
          bubble_cnt, req_addr, req_type);
      if (req_addr != -1) {
        req_addr = memory.page_allocator(req_addr, id);
      }
    }
    if (!more_reqs) {
      if (!reached_limit) { // if the length of this trace is shorter than expected length, then record it when the whole trace finishes, and set reached_limit to true.
        record_cycs = clk;
        record_insts = long(cpu_inst.value());
        memory.record_core(id);
        reached_limit = true;
      }
    }
}

bool Core::finished()
{
    return !more_reqs && window.is_empty();
}

bool Core::has_reached_limit() {
  return reached_limit;
}

void Core::receive(Request& req)
{
    window.set_ready(req.addr, ~(l1_blocksz - 1l));
    if (req.arrive != -1 && req.depart > last) {
      memory_access_cycles += (req.depart - max(last, req.arrive));
      last = req.depart;
    }
}

bool Window::is_full()
{
    return load == depth;
}

bool Window::is_empty()
{
    return load == 0;
}


void Window::insert(bool ready, long addr)
{
    assert(load <= depth);

    ready_list.at(head) = ready;
    addr_list.at(head) = addr;

    head = (head + 1) % depth;
    load++;
}


long Window::retire()
{
    assert(load <= depth);

    if (load == 0) return 0;

    int retired = 0;
    while (load > 0 && retired < ipc) {
        if (!ready_list.at(tail))
            break;

        tail = (tail + 1) % depth;
        load--;
        retired++;
    }

    return retired;
}


void Window::set_ready(long addr, int mask)
{
    if (load == 0) return;

    for (int i = 0; i < load; i++) {
        int index = (tail + i) % depth;
        if ((addr_list.at(index) & mask) != (addr & mask))
            continue;
        ready_list.at(index) = true;
    }
}



Trace::Trace(const char* trace_fname) : file(trace_fname), trace_name(trace_fname)
{
    if (!file.good()) {
        std::cerr << "Bad trace file: " << trace_fname << std::endl;
        exit(1);
    }
}

bool Trace::get_unfiltered_request(long& bubble_cnt, long& req_addr, Request::Type& req_type)
{
    string line;
    getline(file, line);
    if (file.eof()) {
      file.clear();
      file.seekg(0, file.beg);
      return false;
    }
    size_t pos, end;
    bubble_cnt = std::stoul(line, &pos, 10);
    pos = line.find_first_not_of(' ', pos+1);
    req_addr = std::stoul(line.substr(pos), &end, 0);

    pos = line.find_first_not_of(' ', pos+end);

    if (pos == string::npos || line.substr(pos)[0] == 'R')
        req_type = Request::Type::READ;
    else if (line.substr(pos)[0] == 'W')
        req_type = Request::Type::WRITE;
    else assert(false);
    return true;
}

bool Trace::get_filtered_request(long& bubble_cnt, long& req_addr, Request::Type& req_type)
{
    static bool has_write = false;
    static long write_addr;
    static int line_num = 0;
    if (has_write){
        bubble_cnt = 0;
        req_addr = write_addr;
        req_type = Request::Type::WRITE;
        has_write = false;
        return true;
    }
    string line;
    getline(file, line);
    line_num ++;
    if (file.eof() || line.size() == 0) {
        file.clear();
        file.seekg(0, file.beg);
        has_write = false;
        line_num = 0;
        return false;
    }

    size_t pos, end;
    bubble_cnt = std::stoul(line, &pos, 10);

    pos = line.find_first_not_of(' ', pos+1);
    req_addr = stoul(line.substr(pos), &end, 0);
    req_type = Request::Type::READ;

    pos = line.find_first_not_of(' ', pos+end);
    if (pos != string::npos){
        has_write = true;
        write_addr = stoul(line.substr(pos), NULL, 0);
    }
    return true;
}

bool Trace::get_dramtrace_request(long& req_addr, Request::Type& req_type)
{
    string line;
    getline(file, line);
    if (file.eof()) {
        return false;
    }
    size_t pos;
    req_addr = std::stoul(line, &pos, 16);

    pos = line.find_first_not_of(' ', pos+1);

    if (pos == string::npos || line.substr(pos)[0] == 'R')
        req_type = Request::Type::READ;
    else if (line.substr(pos)[0] == 'W')
        req_type = Request::Type::WRITE;
    else assert(false);
    return true;
}

bool DRAMTraceThread::get_dramtrace_request(long& req_addr, Request::Type& req_type)
{
    std::unique_lock<std::mutex> lck(mtx);
    while (q.empty()) {
        cv.wait(lck);
    }
    auto entry = q.front();
    q.pop();
    queueFull.notify_all();
    if (entry.second == Request::Type::END) return false;
    req_addr = entry.first;
    req_type = entry.second;
    return true;
}

bool CPUTraceThread::get_unfiltered_request(long& bubble_cnt, long& req_addr, Request::Type& req_type)
{
    std::unique_lock<std::mutex> lck(mtx);
    while (lq.empty()) {
        // if there is a core with non empty queue, send idle to this core and process the request.
        // the core can be explicitly terminated by sending terminate entry with notify_end()
        for (auto& q: queue_list) {
            if (!q.empty()){
                bubble_cnt = 0;
                req_type = Request::Type::IDLE;
                return true;
            }
        }
        cv.wait(lck);
    }
    auto entry = lq.front();
    // Termination condition
    if (entry.rd_addr == -1 && entry.wr_addr == -1) return false;
    lq.pop();
    queueFull.notify_all();
    assert (entry.rd_addr == -1 || entry.wr_addr == -1);
    bubble_cnt = entry.bubble_cnt;
    req_addr = entry.wr_addr != -1? entry.wr_addr
                                  : entry.rd_addr;
    req_type = entry.wr_addr != -1? Request::Type::WRITE
                                  : Request::Type::READ;
    return true;
}

bool CPUTraceThread::get_filtered_request(long& bubble_cnt, long& req_addr, Request::Type& req_type)
{
    static bool has_write = false;
    static long write_addr;
    // if there's unprocessed write, no need to get a lock
    if (has_write)
    {
        bubble_cnt = 0;
        req_addr = write_addr;
        req_type = Request::Type::WRITE;
        has_write = false;
        return true;
    }
    else
    {
        std::unique_lock<std::mutex> lck(mtx);
        while (lq.empty())
        {
            // if there is a core with non empty queue, send idle to this core and process the request.
            // the core can be explicitly terminated by sending terminate entry with notify_end()
            for (auto& q: queue_list) {
                if (!q.empty()){
                    bubble_cnt = 0;
                    req_type = Request::Type::IDLE;
                    return true;
                }
            }
            cv.wait(lck);
        }
        auto entry = lq.front();
        // Termination condition
        if (entry.rd_addr == -1 && entry.wr_addr == -1)
        {
            has_write = false;
            return false;
        }
        lq.pop();
        queueFull.notify_all();
        assert(entry.rd_addr != -1);

        bubble_cnt = entry.bubble_cnt;
        req_addr = entry.rd_addr;
        req_type = Request::Type::READ;

        if (entry.wr_addr != -1)
        {
            has_write = true;
            write_addr = entry.wr_addr;
        }

        return true;
    }
}

auto TraceThreadFactory::make_cpu_trace_thread(int core_id) -> CPUTraceThread*
{
    assert (core_id >= 0 && core_id < total_cores);
    return new CPUTraceThread(mtx, cv, queueFull, max_queue_size / total_cores + 1, queue_list.at(core_id), queue_list);
}

auto TraceThreadFactory::get_dram_trace_thread() -> std::unique_ptr<DRAMTraceThread>
{
    assert (dram_trace_thread);
    return std::move(dram_trace_thread);
}

void TraceThreadFactory::dram_enqueue(long req_addr, Request::Type req_type)
{
    assert(!total_cores);
    std::unique_lock<std::mutex> lck(mtx);
    while (q.size() >= max_queue_size) {
        queueFull.wait(lck);
    }
    q.push(std::make_pair(req_addr, req_type));
    cv.notify_all();
}

void TraceThreadFactory::dram_enqueue(long req_addr, const char* req_type)
{
    assert(!total_cores);
    std::unique_lock<std::mutex> lck(mtx);
    while (q.size() >= max_queue_size) {
        queueFull.wait(lck);
    }
    Request::Type ty = req_type[0] == 'R'? Request::Type::READ  : 
                       req_type[0] == 'W'? Request::Type::WRITE :
                       Request::Type::END;
    assert(ty != Request::Type::END);
    q.push(std::make_pair(req_addr, ty));
    cv.notify_all();
}

void TraceThreadFactory::notify_end()
{
    if (total_cores == 0)
        dram_enqueue(0, Request::Type::END);
    else {
        CPUTraceEntry END_TRACE{0, -1, -1};
        for (auto& lq: queue_list) {
            lq.push(END_TRACE);
        }
    }
}

void TraceThreadFactory::notify_end(int core) {
    assert(core >= 0 && core < total_cores);
    CPUTraceEntry END_TRACE{0, -1, -1};
    auto& lq = queue_list.at(core);
    std::unique_lock<std::mutex> lck(mtx);
    while (lq.size() >= max_queue_size) {
        queueFull.wait(lck);
    }
    lq.push(END_TRACE);
}

void TraceThreadFactory::cpu_enqueue(int core, long bubble_cnt, long read_addr, long write_addr)
{
    assert(core >= 0 && core < total_cores);
    CPUTraceEntry trace_entry {bubble_cnt, read_addr, write_addr};

    std::unique_lock<std::mutex> lck(mtx);
    auto& lq = queue_list.at(core);
    while (lq.size() >= max_queue_size) {
        queueFull.wait(lck);
    }
    lq.push(trace_entry);
    cv.notify_all();
}