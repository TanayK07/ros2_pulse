// Isolated hot-path microbench. Legs:
//   OLD          — the #1204-style global mutex + per-message string hash this design replaced
//   single-entry — one (key,ctr) TLS slot per thread (the pre-issue-13 cache)
//   mini-map     — 16-slot direct-mapped TLS cache (current design)
// each measured with a FIXED key per thread (best case) and ALTERNATING 4 keys per thread (the
// realistic camera-pipeline pattern that thrashed the single-entry cache — KNOWN_ISSUES #13).
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

// current design: 16-slot direct-mapped TLS cache (mirrors src/core/topic_registry.cpp)
struct NewMap : RegBase {
  struct Slot { const void* key=nullptr; Ctr* ctr=nullptr; };
  inline void on(const void* k){
    thread_local Slot cache[16];
    Slot& s = cache[(reinterpret_cast<uintptr_t>(k)>>4)&15];
    if(s.key==k){ s.ctr->c.fetch_add(1,std::memory_order_relaxed); return; }
    std::shared_lock<std::shared_mutex> l(m); auto it=r.find(k);
    if(it!=r.end()){ it->second->c.fetch_add(1,std::memory_order_relaxed); s={k,it->second}; } } };

template<typename F> double run(int n, F&& f){ std::vector<std::thread> t; auto s=Clock::now();
  for(int i=0;i<n;i++) t.emplace_back(f,i); for(auto& x:t) x.join();
  return std::chrono::duration<double>(Clock::now()-s).count(); }

template<typename C>
void leg(const char* name, C& c, int th, long tot, const std::vector<const void*>& ks, bool alternate){
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
    leg("mini-map TLS (fixed)",       n, th, tot, ks, false);
    leg("mini-map TLS (alt-4)",       n, th, tot, ks, true); }
  return 0; }
