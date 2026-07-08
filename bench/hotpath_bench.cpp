// Isolated hot-path microbench: OLD (#1204 global mutex + per-msg string hash) vs NEW
// (per-endpoint atomic + thread-local cache). g++ -O2 -std=c++17 -pthread.
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
using Clock = std::chrono::steady_clock;
static constexpr int kTopics = 50;
static constexpr long kPer = 5'000'000;
struct OldC { std::mutex mu; std::unordered_map<std::string, size_t> c;
  inline void on(const char* t){ std::lock_guard<std::mutex> l(mu); c[t]++; } };
struct Ctr { std::atomic<uint64_t> c{0}; };
struct NewC { std::shared_mutex m; std::unordered_map<const void*, Ctr*> r;
  Ctr* reg(const void* k){ std::unique_lock<std::shared_mutex> l(m); auto* p=new Ctr(); r[k]=p; return p; }
  inline void on(const void* k){ thread_local const void* lk=nullptr; thread_local Ctr* lc=nullptr;
    if(k==lk){ lc->c.fetch_add(1,std::memory_order_relaxed); return; }
    std::shared_lock<std::shared_mutex> l(m); auto it=r.find(k);
    if(it!=r.end()){ it->second->c.fetch_add(1,std::memory_order_relaxed); lk=k; lc=it->second; } } };
template<typename F> double run(int n, F&& f){ std::vector<std::thread> t; auto s=Clock::now();
  for(int i=0;i<n;i++) t.emplace_back(f,i); for(auto& x:t) x.join();
  return std::chrono::duration<double>(Clock::now()-s).count(); }
int main(int argc,char** argv){ int th=argc>1?atoi(argv[1]):8; long tot=(long)th*kPer;
  std::vector<std::string> nm(kTopics); std::vector<const void*> ks(kTopics);
  for(int i=0;i<kTopics;i++){ nm[i]="/sensor/topic_"+std::to_string(i)+"/data"; ks[i]=&nm[i]; }
  printf("threads=%d topics=%d total=%ld\n",th,kTopics,tot);
  { OldC o; for(int i=0;i<kTopics;i++) o.c[nm[i]]=0;
    double s=run(th,[&](int t){ const char* tp=nm[t%kTopics].c_str(); for(long i=0;i<kPer;i++) o.on(tp); });
    printf("OLD (mutex+strhash)  %7.1f ns/op  %6.1f M/s\n", s*1e9/tot, tot/s/1e6);
    NewC n; for(int i=0;i<kTopics;i++) n.reg(ks[i]);
    double s2=run(th,[&](int t){ const void* k=ks[t%kTopics]; for(long i=0;i<kPer;i++) n.on(k); });
    printf("NEW (atomic+TLS)     %7.1f ns/op  %6.1f M/s  -> %.0fx\n", s2*1e9/tot, tot/s2/1e6, s/s2); }
  return 0; }
