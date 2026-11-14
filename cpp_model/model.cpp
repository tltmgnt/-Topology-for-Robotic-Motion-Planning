// Grid shortest-path model
// Repository: https://github.com/tltmgnt/-Topology-for-Robotic-Motion-Planning
// Standalone C++17, offline, synthetic reference model.
// Build: c++ -std=c++17 -O2 -Wall -Wextra -pedantic cpp_model/model.cpp -o model
// Run: ./model    (runs built-in assertions and prints a bounded demo result)
// Research-only: no trained AI, independent validation, external actions or approval.
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <numeric>
#include <queue>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

void require(bool ok,const char* msg){if(!ok)throw std::runtime_error(msg);}
namespace model {
int path_length(const std::vector<std::string>& map,int sx,int sy,int tx,int ty){int h=(int)map.size();if(!h)return -1;int w=(int)map[0].size();for(auto&s:map)if((int)s.size()!=w)throw std::invalid_argument("ragged map");if(sx<0||tx<0||sy<0||ty<0||sx>=w||tx>=w||sy>=h||ty>=h||map[sy][sx]=='#'||map[ty][tx]=='#')return -1;
 std::queue<std::pair<int,int>> q;std::vector<int> d(w*h,-1);d[sy*w+sx]=0;q.push({sx,sy});int dx[4]={1,-1,0,0},dy[4]={0,0,1,-1};while(!q.empty()){auto [x,y]=q.front();q.pop();if(x==tx&&y==ty)return d[y*w+x];for(int k=0;k<4;++k){int a=x+dx[k],b=y+dy[k];if(a>=0&&b>=0&&a<w&&b<h&&map[b][a]!='#'&&d[b*w+a]<0){d[b*w+a]=d[y*w+x]+1;q.push({a,b});}}}return -1;}
void demo(){int d=path_length({"...","##.","..."},0,0,2,2);require(d==4,"shortest path");std::cout<<"grid_steps="<<d<<" physical_clearance_certified=0";}
} // namespace model
int main(){try{std::cout<<std::boolalpha;model::demo();std::cout<<'\n';return 0;}catch(const std::exception&e){std::cerr<<"model test failed: "<<e.what()<<'\n';return 1;}}
