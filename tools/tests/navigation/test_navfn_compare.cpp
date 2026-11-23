#include "nav2_navfn_planner/navfn.hpp"

extern "C" {
#include "app_global_plan.h"
#include "app_buildmap.h"
}

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

extern "C" {
Map g_build_map{};
}
static bool have_pose = true;
static Pose robot{};

extern "C" bool App_BuildMap_GetRobotPose(Pose *pose)
{
    if (have_pose) *pose = robot;
    return have_pose;
}

struct Buffers {
    std::vector<uint8_t> costs, pending, lut;
    std::vector<float> potential, line, splits;
    std::vector<int> sites;
    std::vector<uint32_t> q0, q1, q2;
    AppGlobalPlanWorkspace workspace;

    Buffers(int width, int height)
        : costs(width * height), pending(width * height), lut(10002),
          potential(width * height), line(std::max(width, height)),
          splits(line.size() + 1), sites(line.size()), q0(10000), q1(10000), q2(10000),
          workspace{costs.data(), costs.size(),
                    {potential.data(), potential.size(), line.data(), sites.data(), splits.data(),
                     line.size(), lut.data(), lut.size()},
                    {potential.data(), pending.data(), potential.size(),
                     {q0.data(), q1.data(), q2.data()}, q0.size()}}
    {}
};

static Pose center(const NavCostmap &map, int x, int y)
{
    return {(float)(map.origin_x + (x + 0.5) * map.resolution),
            (float)(map.origin_y + (y + 0.5) * map.resolution), 0};
}

static void compare(const NavCostmap &map, Buffers &buffers, int sx, int sy, int gx, int gy,
                    bool allow_unknown, bool reachable)
{
    nav2_navfn_planner::NavFn reference(map.width, map.height);
    int start[] = {sx, sy}, goal[] = {gx, gy};
    reference.setCostmap(map.costs, true, allow_unknown);
    reference.setGoal(start);
    reference.setStart(goal);
    reference.calcNavFnDijkstra([] { return false; }, true);
    int count = reference.calcPath(NAV_PATH_MAX_POINTS);
    if ((count != 0) != reachable) {
        std::fprintf(stderr, "Nav2 (%d,%d)->(%d,%d), unknown=%d: count=%d, target potential=%g\n",
                     sx, sy, gx, gy, allow_unknown, count, reference.potarr[gy * map.width + gx]);
    }
    assert((count != 0) == reachable);

    NavFnConfig config{allow_unknown, 0, nullptr, nullptr};
    NavPath path{};
    NavPlanResult result = NavFn_Plan(&map, center(map, sx, sy), center(map, gx, gy), &config,
                                    &buffers.workspace.navfn, &path);
    assert(result == (reachable ? NAV_PLAN_OK : NAV_PLAN_NO_PATH));
    assert(path.count == count);
    for (size_t i = 0; i < buffers.potential.size(); ++i) {
        assert(std::fabs(reference.potarr[i] - buffers.potential[i]) <= 0.002f);
    }
    for (int i = 0; i < count; ++i) {
        int j = count - 1 - i;
        float x = (float)(map.origin_x + (reference.getPathX()[j] + 0.5) * map.resolution);
        float y = (float)(map.origin_y + (reference.getPathY()[j] + 0.5) * map.resolution);
        assert(std::fabs(path.points[i].x - x) < 0.0001f);
        assert(std::fabs(path.points[i].y - y) < 0.0001f);
    }
}

static uint8_t expected_cost(const Map &map, const NavCostmapConfig &config, int x, int y)
{
    int width = map.c.width;
    const Cell &current = map.cells[y * width + x];
    uint8_t original = current.observed ?
        (current.log_odds >= config.occupied_log_odds ? NAV_COST_LETHAL : NAV_COST_FREE) :
        NAV_COST_UNKNOWN;
    double nearest = std::numeric_limits<double>::infinity();
    for (int row = 0; row < map.c.height; ++row) {
        for (int col = 0; col < width; ++col) {
            const Cell &cell = map.cells[row * width + col];
            bool seed = cell.observed ? cell.log_odds >= config.occupied_log_odds :
                        config.inflate_around_unknown;
            if (seed) nearest = std::min(nearest, std::hypot(x - col, y - row));
        }
    }
    if (nearest > std::ceil((double)config.inflation_radius_m / map.c.resolution)) return original;
    double meters = std::round(nearest * 100) / 100 * map.c.resolution;
    uint8_t inflated = nearest == 0 ? NAV_COST_LETHAL :
        (meters <= config.inscribed_radius_m ? NAV_COST_INSCRIBED :
         (uint8_t)(252 * std::exp(-config.cost_scaling_factor *
                                  (meters - config.inscribed_radius_m))));
    if (original == NAV_COST_UNKNOWN &&
        (config.inflate_unknown ? inflated > 0 : inflated >= NAV_COST_INSCRIBED)) return inflated;
    return std::max(original, inflated);
}

int main()
{
    constexpr int w = 43, h = 31;
    Buffers buffers(w, h);
    std::vector<Cell> cells(w * h, Cell{-1, 0, 1});
    Map map{{w, h, 0.04f, -6, -6, -0.4f, 0.85f, -4, 4}, cells.data(), nullptr};
    NavCostmapConfig costs{1.5f, 0.08f, 0.20f, 3, false, false};
    NavCostmap view{};
    auto build = [&] {
        assert(NavCostmap_Build(&map, &costs, buffers.costs.data(), buffers.costs.size(),
                                &buffers.workspace.inflation, &view));
    };
    build();
    assert(std::all_of(buffers.costs.begin(), buffers.costs.end(), [](auto c) { return c == 0; }));
    compare(view, buffers, 5, 5, 34, 23, false, true);

    for (int y = 1; y < 19; ++y) cells[y * w + 20].log_odds = 2.0f;
    cells[8 * w + 22].observed = 0;
    cells[28 * w + 38].observed = 0;
    for (bool around_unknown : {false, true}) {
        for (bool inflate_unknown : {false, true}) {
            costs.inflate_around_unknown = around_unknown;
            costs.inflate_unknown = inflate_unknown;
            build();
            for (int y = 0; y < h; ++y)
                for (int x = 0; x < w; ++x)
                    assert(view.costs[y * w + x] == expected_cost(map, costs, x, y));
        }
    }
    costs.inflate_around_unknown = costs.inflate_unknown = false;
    build();
    compare(view, buffers, 9, 16, 31, 19, false, true);
    for (int y = 19; y < h; ++y) cells[y * w + 20].log_odds = 2.0f;
    build();
    compare(view, buffers, 5, 5, 34, 10, false, false);

    std::fill(cells.begin(), cells.end(), Cell{-1, 0, 1});
    for (int y = 0; y < h; ++y) cells[y * w + 20].observed = 0;
    build();
    compare(view, buffers, 5, 5, 34, 10, false, false);
    compare(view, buffers, 5, 5, 34, 10, true, true);

    // Application integration and explicit failure states on the real grid geometry.
    Buffers field(300, 300);
    std::vector<Cell> field_cells(300 * 300, Cell{-1, 0, 1});
    g_build_map = {{300, 300, 0.04f, -6, -6, -0.4f, 0.85f, -4, 4},
                   field_cells.data(), nullptr};
    robot = {0, 0, -0.4f};
    Pose goal{5.3f, 4.2f, 1.2f};
    NavFnConfig config{};
    NavPath path{};
    auto app_plan = [&] { return App_GlobalPlan_Create(goal, &costs, &config, &field.workspace, &path); };
    assert(app_plan() == NAV_PLAN_OK && path.count > 2);
    assert(path.points[0].x == robot.x && path.points[0].y == robot.y);
    assert(path.points[0].theta == robot.theta);
    assert(path.points[path.count - 1].x == goal.x && path.points[path.count - 1].y == goal.y);
    assert(path.points[path.count - 1].theta == goal.theta);
    have_pose = false;
    assert(app_plan() == NAV_PLAN_NO_POSE && path.count == 0);
    have_pose = true;
    config.max_cycles = 1;
    assert(app_plan() == NAV_PLAN_ITERATION_LIMIT && path.count == 0);
    config.max_cycles = 0;
    int cancel_checks = 0;
    config.cancel_context = &cancel_checks;
    config.cancel = [](void *context) { return ++*static_cast<int *>(context) >= 10; };
    assert(app_plan() == NAV_PLAN_CANCELLED && path.count == 0);
    config.cancel = nullptr;
    field.workspace.navfn.queue_capacity = 1;
    assert(app_plan() == NAV_PLAN_WORKSPACE_FULL && path.count == 0);
    field.workspace.navfn.queue_capacity = field.q0.size();
    goal = {-6.01f, 0, 0};
    assert(app_plan() == NAV_PLAN_OUTSIDE_MAP && path.count == 0);
    goal = {-5.99f, 0, 0};
    assert(app_plan() == NAV_PLAN_ENDPOINT_BLOCKED && path.count == 0);
    goal = {0.01f, 0.01f, 1.2f};
    assert(app_plan() == NAV_PLAN_OK && path.count == 2);
    goal = robot;
    goal.theta = 1.2f;
    assert(app_plan() == NAV_PLAN_OK && path.count == 1 && path.points[0].theta == goal.theta);

    // Five one-cell-wide turns force more output points than NavPath can hold.
    constexpr int left = 10, right = 289, bottom = 30;
    std::fill(field_cells.begin(), field_cells.end(), Cell{2, 0, 1});
    for (int lane = 0; lane < 5; ++lane) {
        int row = bottom + 4 * lane;
        for (int x = left; x <= right; ++x) field_cells[row * 300 + x].log_odds = -1;
        if (lane < 4) {
            int x = lane % 2 == 0 ? right : left;
            for (int y = 1; y < 4; ++y) field_cells[(row + y) * 300 + x].log_odds = -1;
        }
    }
    NavCostmap geometry{300, 300, 0.04f, -6, -6, field.costs.data()};
    robot = center(geometry, left, bottom);
    goal = center(geometry, right, bottom + 16);
    costs.inscribed_radius_m = costs.inflation_radius_m = 0;
    assert(app_plan() == NAV_PLAN_PATH_FULL && path.count == 0);

    std::puts("Nav2 NavFn potential/path parity, inflation and application tests passed");
}
