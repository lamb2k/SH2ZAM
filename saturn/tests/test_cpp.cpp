#include <sh2zam/sh2zam.hpp>
#include <cstdio>
static_assert(sh2z::fixed::from_raw(SH2Z_ONE).raw()==SH2Z_ONE,"constexpr raw constructor");
int main() {
    using sh2z::fixed;
    auto a=fixed::from_int(3),b=fixed::from_int(4);
    sh2z::vec3 v(a,b,fixed());
    if(v.length().integer()!=5||(a*b).integer()!=12||(b-a).integer()!=1) return 1;
    if((a+b).integer()!=7||(b/b)!=fixed::from_int(1)||b.sqrt()!=fixed::from_int(2)) return 2;
    auto identity=sh2z_mat34_identity();
    if(v.transformed(identity).value.x!=v.value.x) return 3;
    if((v-v).length().raw()!=0||(v+v).length().integer()!=10) return 4;
    if((v*fixed::from_int(2)).length().integer()!=10||v.dot(v).integer()!=25) return 5;
    if(v.direction().length().raw()<SH2Z_ONE-2) return 6;
    std::puts("C++11 facade and C linkage passed");
    return 0;
}
