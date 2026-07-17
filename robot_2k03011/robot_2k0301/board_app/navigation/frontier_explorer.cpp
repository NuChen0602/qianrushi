#include "navigation/frontier_explorer.h"

#include <array>
#include <queue>

namespace robot::navigation
{
FrontierGoal selectFrontier(const OccupancyGrid& map, const Pose2D& robot,
                            double minimum_distance, int minimum_cluster)
{
    FrontierGoal best;
    const int width = map.width(), height = map.height();
    std::vector<std::uint8_t> frontier(static_cast<std::size_t>(width * height), 0);
    for(int y = 1; y + 1 < height; ++y)
        for(int x = 1; x + 1 < width; ++x)
            if(!map.occupiedCell(x, y) && (map.unknownCell(x + 1, y) || map.unknownCell(x - 1, y) ||
               map.unknownCell(x, y + 1) || map.unknownCell(x, y - 1)))
                frontier[static_cast<std::size_t>(y * width + x)] = 1;
    std::vector<std::uint8_t> visited(frontier.size(), 0);
    const std::array<std::pair<int, int>, 4> directions = {{{1,0},{-1,0},{0,1},{0,-1}}};
    for(int sy = 1; sy + 1 < height; ++sy)
        for(int sx = 1; sx + 1 < width; ++sx)
        {
            const auto start = static_cast<std::size_t>(sy * width + sx);
            if(!frontier[start] || visited[start]) continue;
            std::queue<std::pair<int, int>> queue; queue.push({sx, sy}); visited[start] = 1;
            int count = 0; double sum_x = 0.0, sum_y = 0.0;
            while(!queue.empty())
            {
                const auto cell = queue.front(); queue.pop(); ++count; sum_x += cell.first; sum_y += cell.second;
                for(const auto& direction : directions)
                {
                    const int x = cell.first + direction.first, y = cell.second + direction.second;
                    const auto index = static_cast<std::size_t>(y * width + x);
                    if(x > 0 && y > 0 && x + 1 < width && y + 1 < height && frontier[index] && !visited[index])
                    { visited[index] = 1; queue.push({x, y}); }
                }
            }
            if(count < minimum_cluster) continue;
            const auto world = map.gridToWorld(static_cast<int>(std::lround(sum_x / count)),
                                               static_cast<int>(std::lround(sum_y / count)));
            const double distance = std::hypot(world.first - robot.x, world.second - robot.y);
            if(distance < minimum_distance) continue;
            const double score = count / (1.0 + distance);
            if(!best.valid || score > best.score)
                best = {{world.first, world.second, std::atan2(world.second - robot.y, world.first - robot.x)},
                        count, score, true};
        }
    return best;
}
} // namespace robot::navigation
