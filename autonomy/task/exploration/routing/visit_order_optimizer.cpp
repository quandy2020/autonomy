/* Copyright 2026 The Openbot Authors. */
#include "autonomy/task/exploration/routing/visit_order_optimizer.hpp"
#include <algorithm>
#include <numeric>
#include <set>
#include <stdexcept>
namespace autonomy::task::exploration {
ExplorationRoute VisitOrderOptimizer::Optimize(const Pose2d& start,const std::vector<Viewpoint>& views,
    const PathProvider& provider,double speed,double angular_speed,const std::optional<Pose2d>& return_pose,bool keep_first,
    const std::function<bool()>& cancel_checker) const {
    if(!std::isfinite(speed) || speed<=0 || !std::isfinite(angular_speed) || angular_speed<=0 || views.size()>128)
        throw std::invalid_argument("invalid route optimization limits");
    ExplorationRoute result;if(views.empty())return result;
    const std::size_t n=views.size(),origin=n,destination=n+1;
    std::vector<Pose2d> poses;for(const auto& v:views)poses.push_back(v.pose);
    poses.push_back(start);poses.push_back(return_pose.value_or(start));
    std::vector<std::vector<double>> distances(n+2,std::vector<double>(n+2,0));
    for(std::size_t i=0;i<poses.size();++i)for(std::size_t j=0;j<poses.size();++j) {
        if(i==j)continue;
        if(cancel_checker && cancel_checker()){result.reachable=false;return result;}
        const auto route=provider.Route(poses[i],poses[j],cancel_checker);
        if(route.canceled){result.reachable=false;return result;}
        distances[i][j]=route.success?route.length:std::numeric_limits<double>::infinity();
    }
    auto edge_time=[&](std::size_t a,std::size_t b){return distances[a][b]/speed+
        std::abs(autonomy::navigation::NormalizeAngle(poses[b].yaw-poses[a].yaw))/angular_speed;};
    auto cost=[&](const std::vector<std::size_t>& order){double t=0;auto previous=origin;
        for(auto i:order){t+=edge_time(previous,i)+views[i].observation_time;previous=i;}
        if (return_pose) {
            t += edge_time(previous, destination);
        }
        return t;
    };
    std::set<std::size_t> remaining;for(std::size_t i=0;i<n;++i)remaining.insert(i);
    std::vector<std::size_t> order;auto previous=origin;
    if(keep_first){order.push_back(0);remaining.erase(0);previous=0;}
    while(!remaining.empty()) {
        if(cancel_checker && cancel_checker()){result.reachable=false;return result;}
        auto best=*remaining.begin();double time=std::numeric_limits<double>::infinity();
        for(auto i:remaining)if(edge_time(previous,i)<time){best=i;time=edge_time(previous,i);}
        if(!std::isfinite(time)){result.reachable=false;return result;}
        order.push_back(best);remaining.erase(best);previous=best;
    }
    // TARE-style visit-order refinement on actual route distances, not Euclidean TSP.
    for(unsigned pass=0;pass<20;++pass){bool improved=false;
        for(std::size_t i=keep_first?1:0;i+1<order.size();++i)for(std::size_t j=i+1;j<order.size();++j){
            if(cancel_checker && cancel_checker()){result.reachable=false;return result;}
            auto alternate=order;std::reverse(alternate.begin()+i,alternate.begin()+j+1);
            if(cost(alternate)+1e-9<cost(order)){order=std::move(alternate);improved=true;}
        }
        if(!improved)break;
    }
    result.total_time=cost(order);result.reachable=std::isfinite(result.total_time);previous=origin;
    for(auto i:order){result.views.push_back(views[i]);result.distance+=distances[previous][i];previous=i;}
    if (return_pose) {
        result.distance += distances[previous][destination];
    }
    return result;
}
}  // namespace autonomy::task::exploration
