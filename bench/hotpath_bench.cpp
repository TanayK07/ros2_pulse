// Isolated hot-path microbench. Legs:
//   OLD          - the #1204-style global mutex + per-message string hash this design replaced
//   single-entry, one (key,ctr) TLS slot per thread (the pre-issue-13 cache)
//   mini-map     - 16-slot direct-mapped TLS cache (pre-issue-15; kept as the historical record)
//   cache-256    - 256-slot TLS cache with the stride-breaking hash (CURRENT design, issue #15)
//   cache-256+gap, the current design plus the R5 gap accumulator (steady_clock read + exchange
//                  + guarded CAS-max, mirroring noteArrival in src/core/topic_registry.cpp).
//                  The (+gap − plain) delta is THE published R5 cost figure; a raw clock leg is
//                  printed alongside so the attribution ("~96% of it is the vDSO clock read")
//                  stays checkable rather than folkloric.
// each measured with a FIXED key per thread (best case) and ALTERNATING 4 keys per thread (the
// realistic camera-pipeline pattern that thrashed the single-entry cache, KNOWN_ISSUES #13).
// g++ -O2 -std=c++17 -pthread.
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
using Clock = std::chrono::steady_clock;
static constexpr int kTopics = 50;
static constexpr long kPer = 5'000'000;
static constexpr int kAlt = 4;  // alternating working-set size per thread

struct OldC { std::mutex mu; std::unordered_map<std::string, size_t> c;
  inline void on(const char* t){ std::lock_guard<std::mutex> l(mu); c[t]++; } };

struct Ctr { std::atomic<uint64_t> c{0}; };

struct RegBase { std::shared_mutex m; std::unordered_map<const void*, Ctr*> r;
  Ctr* reg(const void* k){ std::unique_lock<std::shared_mutex> l(m); auto* p=new Ctr(); r[k]=p; return p; } };

// pre-issue-13 cache: one slot per thread
struct NewSingle : RegBase {
  inline void on(const void* k){ thread_local const void* lk=nullptr; thread_local Ctr* lc=nullptr;
    if(k==lk){ lc->c.fetch_add(1,std::memory_order_relaxed); return; }
    std::shared_lock<std::shared_mutex> l(m); auto it=r.find(k);
    if(it!=r.end()){ it->second->c.fetch_add(1,std::memory_order_relaxed); lk=k; lc=it->second; } } };

// pre-issue-15 design: 16-slot direct-mapped TLS cache, bare (ptr>>4) index
struct NewMap : RegBase {
  struct Slot { const void* key=nullptr; Ctr* ctr=nullptr; };
  inline void on(const void* k){
    thread_local Slot cache[16];
    Slot& s = cache[(reinterpret_cast<uintptr_t>(k)>>4)&15];
    if(s.key==k){ s.ctr->c.fetch_add(1,std::memory_order_relaxed); return; }
    std::shared_lock<std::shared_mutex> l(m); auto it=r.find(k);
    if(it!=r.end()){ it->second->c.fetch_add(1,std::memory_order_relaxed); s={k,it->second}; } } };

// current design: 256-slot TLS cache + stride-breaking hash (mirrors src/core/topic_registry.cpp
// after KNOWN_ISSUES #15). Optionally runs the R5 gap accumulator on every hit, an exact replica
// of noteArrival: exchange partitions the predecessor timestamp between racing threads, and the
// now > prev guard drops reordered pairs instead of letting the unsigned subtraction underflow.
struct GapCtr { std::atomic<uint64_t> c{0}; std::atomic<uint64_t> last{0}; std::atomic<uint64_t> maxdt{0}; };
template<bool kGap>
struct New256 { std::shared_mutex m; std::unordered_map<const void*, GapCtr*> r;
  GapCtr* reg(const void* k){ std::unique_lock<std::shared_mutex> l(m); auto* p=new GapCtr(); r[k]=p; return p; }
  static inline void note(GapCtr* g){
    const auto now = static_cast<uint64_t>(Clock::now().time_since_epoch().count());
    const uint64_t prev = g->last.exchange(now, std::memory_order_relaxed);
    if(prev==0 || now<=prev) return;
    const uint64_t dt = now-prev;
    uint64_t cur = g->maxdt.load(std::memory_order_relaxed);
    while(dt>cur && !g->maxdt.compare_exchange_weak(cur,dt,std::memory_order_relaxed)) {}
  }
  inline void on(const void* k){
    struct Slot { const void* key=nullptr; GapCtr* ctr=nullptr; };
    thread_local Slot cache[256];
    const auto p = reinterpret_cast<uintptr_t>(k);
    Slot& s = cache[((p>>4)^(p>>10))&255];
    if(s.key==k){ s.ctr->c.fetch_add(1,std::memory_order_relaxed); if(kGap) note(s.ctr); return; }
    std::shared_lock<std::shared_mutex> l(m); auto it=r.find(k);
    if(it!=r.end()){ it->second->c.fetch_add(1,std::memory_order_relaxed); if(kGap) note(it->second); s={k,it->second}; } } };

template<typename F> double run(int n, F&& f){ std::vector<std::thread> t; auto s=Clock::now();
  for(int i=0;i<n;i++) t.emplace_back(f,i); for(auto& x:t) x.join();
  return std::chrono::duration<double>(Clock::now()-s).count(); }

template<typename C>
double leg(const char* name, C& c, int th, long tot, const std::vector<const void*>& ks, bool alternate){
  double s = run(th, [&](int t){
    if(alternate){
      const void* mine[kAlt];
      for(int j=0;j<kAlt;j++) mine[j]=ks[(t*kAlt+j)%kTopics];
      for(long i=0;i<kPer;i++) c.on(mine[i&(kAlt-1)]);
    } else {
      const void* k=ks[t%kTopics];
      for(long i=0;i<kPer;i++) c.on(k);
    }
  });
  std::printf("%-28s %7.1f ns/op  %8.1f M/s\n", name, s*1e9/tot, tot/s/1e6);
  return s*1e9/tot;
}

int main(int argc,char** argv){ int th=argc>1?atoi(argv[1]):8; long tot=(long)th*kPer;
  std::vector<std::string> nm(kTopics); std::vector<const void*> ks(kTopics);
  for(int i=0;i<kTopics;i++){ nm[i]="/sensor/topic_"+std::to_string(i)+"/data"; ks[i]=&nm[i]; }
  std::printf("threads=%d topics=%d total=%ld alt=%d\n",th,kTopics,tot,kAlt);

  { OldC o; for(int i=0;i<kTopics;i++) o.c[nm[i]]=0;
    double s=run(th,[&](int t){ const char* tp=nm[t%kTopics].c_str(); for(long i=0;i<kPer;i++) o.on(tp); });
    std::printf("%-28s %7.1f ns/op  %8.1f M/s\n","OLD mutex+strhash (fixed)", s*1e9/tot, tot/s/1e6); }

  { NewSingle n; for(int i=0;i<kTopics;i++) n.reg(ks[i]);
    leg("single-entry TLS (fixed)",   n, th, tot, ks, false);
    leg("single-entry TLS (alt-4)",   n, th, tot, ks, true); }

  { NewMap n; for(int i=0;i<kTopics;i++) n.reg(ks[i]);
    leg("mini-map-16 TLS (fixed)",    n, th, tot, ks, false);
    leg("mini-map-16 TLS (alt-4)",    n, th, tot, ks, true); }

  // Current design without and with the R5 gap accumulator. The pairwise delta of the same leg
  // shape IS the per-message cost of ROS_TOPIC_STATS_JITTER=1; report it explicitly so nobody
  // has to subtract by eye. alt-4 is the realistic pattern; fixed is the floor.
  double base_fixed, base_alt, gap_fixed, gap_alt;
  { New256<false> n; for(int i=0;i<kTopics;i++) n.reg(ks[i]);
    base_fixed = leg("cache-256 TLS (fixed)",   n, th, tot, ks, false);
    base_alt   = leg("cache-256 TLS (alt-4)",   n, th, tot, ks, true); }
  { New256<true> n; for(int i=0;i<kTopics;i++) n.reg(ks[i]);
    gap_fixed  = leg("cache-256+gap (fixed)",   n, th, tot, ks, false);
    gap_alt    = leg("cache-256+gap (alt-4)",   n, th, tot, ks, true); }

  // Attribution: the raw clock read with the same loop shape. Single-thread wall time IS CPU
  // time; the th-thread leg checks contention doesn't change the story (vDSO reads don't share).
  double clk_one, clk_n;
  { volatile uint64_t sink=0;
    auto s0=Clock::now(); for(long i=0;i<kPer;i++) sink+=Clock::now().time_since_epoch().count();
    clk_one=std::chrono::duration<double>(Clock::now()-s0).count()*1e9/kPer;
    std::printf("%-28s %7.1f ns/op\n","raw steady_clock (1 thread)", clk_one);
    double sN=run(th,[&](int){ volatile uint64_t x=0; for(long i=0;i<kPer;i++) x+=Clock::now().time_since_epoch().count(); });
    clk_n=sN*1e9/kPer;  // per-thread = CPU ns/op (threads run the whole wall interval)
    std::printf("%-28s %7.1f CPU ns/op\n","raw steady_clock (th thr)", clk_n); }

  // The published R5 figure is PER-MESSAGE CPU cost. The leg convention above is wall time
  // amortized over all threads' ops, which divides the per-message cost by the parallelism,
  // fine for throughput, misleading for "what does one message pay". Every thread runs the
  // whole interval, so CPU ns/msg = amortized ns/op x threads. Print both so neither reading
  // is left as an exercise.
  const double d_fixed=(gap_fixed-base_fixed)*th, d_alt=(gap_alt-base_alt)*th;
  std::printf("\nR5 gap cost, +gap minus plain (ROS_TOPIC_STATS_JITTER=1):\n");
  std::printf("  fixed : %+6.1f CPU ns/msg  (wall-amortized %+5.1f)\n", d_fixed, (gap_fixed-base_fixed));
  std::printf("  alt-4 : %+6.1f CPU ns/msg  (wall-amortized %+5.1f)\n", d_alt, (gap_alt-base_alt));
  std::printf("  clock read alone: %.1f CPU ns -> %.0f%% of the fixed-leg delta\n",
              clk_n, 100.0*clk_n/(d_fixed>0?d_fixed:1));
  return 0; }
