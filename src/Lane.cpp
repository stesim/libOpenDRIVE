#include "Lane.h"

#include <algorithm>
#include <iterator>
#include <type_traits>

namespace odr
{

HeightOffset::HeightOffset(double inner, double outer) : inner(inner), outer(outer) {}

LaneKey::LaneKey(std::string road_id, double lanesection_s0, int lane_id) : road_id(road_id), lanesection_s0(lanesection_s0), lane_id(lane_id) {}

std::string LaneKey::to_string() const { return string_format("%s/%f/%d", this->road_id.c_str(), this->lanesection_s0, this->lane_id); }

Lane::Lane(std::string road_id, double lanesection_s0, int id, bool level, std::string type) :
    key(road_id, lanesection_s0, id), id(id), level(level), type(type)
{
}

std::vector<RoadMark> Lane::get_roadmarks(const double s_start, const double s_end) const
{
    if ((s_start == s_end) || this->roadmark_groups.empty())
        return {};

    auto s_start_rm_iter =
        std::upper_bound(this->roadmark_groups.begin(),
                         this->roadmark_groups.end(),
                         s_start,
                         [](const double& s, const RoadMarkGroup& rmg) -> bool { return s < (rmg.lanesection_s0 + rmg.s_offset); });
    if (s_start_rm_iter != this->roadmark_groups.begin())
        s_start_rm_iter--;

    auto s_end_rm_iter = std::lower_bound(this->roadmark_groups.begin(),
                                          this->roadmark_groups.end(),
                                          s_end,
                                          [](const RoadMarkGroup& rmg, const double& s) -> bool { return (rmg.lanesection_s0 + rmg.s_offset) < s; });

    auto const add_broken_marks = [this](
        RoadMarkGroup const& roadmark_group,
        double roadmark_group_s0,
        double s_end_roadmark_group,
        double s_end,
        double width,
        double t_offset,
        std::vector<RoadMark>& roadmarks
    ) {
        for (double s_start_single_roadmark = roadmark_group_s0; s_start_single_roadmark < s_end_roadmark_group;
                s_start_single_roadmark += (ROADMARK_BROKEN_LENGTH + ROADMARK_BROKEN_SPACE))
        {
            const double s_end_single_roadmark = std::min(s_end, s_start_single_roadmark + ROADMARK_BROKEN_LENGTH);
            roadmarks.push_back(RoadMark(this->key.road_id,
                                            this->key.lanesection_s0,
                                            this->id,
                                            roadmark_group_s0,
                                            s_start_single_roadmark,
                                            s_end_single_roadmark,
                                            t_offset,
                                            width,
                                            roadmark_group.type,
                                            roadmark_group.color));
        }
    };

    auto const add_solid_mark = [this](
        RoadMarkGroup const& roadmark_group,
        double roadmark_group_s0,
        double s_start_roadmark_group,
        double s_end_roadmark_group,
        double s_end,
        double width,
        double t_offset,
        std::vector<RoadMark>& roadmarks
    ) {
        roadmarks.push_back(RoadMark(this->key.road_id,
                                        this->key.lanesection_s0,
                                        this->id,
                                        roadmark_group_s0,
                                        s_start_roadmark_group,
                                        s_end_roadmark_group,
                                        t_offset,
                                        width,
                                        roadmark_group.type,
                                        roadmark_group.color)); // (stesim): added color member
    };

    std::vector<RoadMark> roadmarks;
    for (auto rm_group_iter = s_start_rm_iter; rm_group_iter != s_end_rm_iter; rm_group_iter++)
    {
        const RoadMarkGroup& roadmark_group = *rm_group_iter;

        const double roadmark_group_s0 = roadmark_group.lanesection_s0 + roadmark_group.s_offset;
        const double s_start_roadmark_group = std::max(roadmark_group_s0, s_start);
        const double s_end_roadmark_group = (std::next(rm_group_iter) == s_end_rm_iter)
                                                ? s_end
                                                : std::min(std::next(rm_group_iter)->lanesection_s0 + std::next(rm_group_iter)->s_offset, s_end);

        double width = roadmark_group.width > 0
            ? roadmark_group.width
            : roadmark_group.weight == "bold" ? ROADMARK_WEIGHT_BOLD_WIDTH : ROADMARK_WEIGHT_STANDARD_WIDTH;
        if (!roadmark_group.roadmark_lines.empty())
        {
            for (const RoadMarksLine& roadmarks_line : roadmark_group.roadmark_lines)
            {
                if (roadmarks_line.width > 0)
                    width = roadmarks_line.width;

                if ((roadmarks_line.length + roadmarks_line.space) == 0)
                    continue;

                const double s0_roadmarks_line = roadmarks_line.group_s0 + roadmarks_line.s_offset;
                for (double s_start_single_roadmark = s0_roadmarks_line; s_start_single_roadmark < s_end_roadmark_group;
                     s_start_single_roadmark += (roadmarks_line.length + roadmarks_line.space))
                {
                    const double s_end_single_roadmark = std::min(s_end, s_start_single_roadmark + roadmarks_line.length);
                    roadmarks.push_back(RoadMark(this->key.road_id,
                                                 this->key.lanesection_s0,
                                                 this->id,
                                                 roadmarks_line.group_s0,
                                                 s_start_single_roadmark,
                                                 s_end_single_roadmark,
                                                 roadmarks_line.t_offset,
                                                 width,
                                                 roadmark_group.type + roadmarks_line.name,
                                                 roadmark_group.color)); // (stesim): added color member
                }
            }
        }
        else if (roadmark_group.type == "broken") // (stesim): added support for broken marks without explicit line definitions
        {
            add_broken_marks(roadmark_group, roadmark_group_s0, s_end_roadmark_group, s_end, width, 0.0, roadmarks);
        }
        else if (roadmark_group.type == "solid solid") // (stesim): added support for solid solid marks
        {
            add_solid_mark(roadmark_group, roadmark_group_s0, s_start_roadmark_group, s_end_roadmark_group, s_end, width, -width, roadmarks);
            add_solid_mark(roadmark_group, roadmark_group_s0, s_start_roadmark_group, s_end_roadmark_group, s_end, width, +width, roadmarks);
        }
        else if (roadmark_group.type == "solid broken") // (stesim): added support for solid broken marks
        {
            add_solid_mark(roadmark_group, roadmark_group_s0, s_start_roadmark_group, s_end_roadmark_group, s_end, width, -width, roadmarks);
            add_broken_marks(roadmark_group, roadmark_group_s0, s_end_roadmark_group, s_end, width, +width, roadmarks);
        }
        else if (roadmark_group.type == "broken solid") // (stesim): added support for broken solid marks
        {
            add_broken_marks(roadmark_group, roadmark_group_s0, s_end_roadmark_group, s_end, width, -width, roadmarks);
            add_solid_mark(roadmark_group, roadmark_group_s0, s_start_roadmark_group, s_end_roadmark_group, s_end, width, +width, roadmarks);
        }
        else if (roadmark_group.type == "broken broken") // (stesim): added support for broken broken marks
        {
            add_broken_marks(roadmark_group, roadmark_group_s0, s_end_roadmark_group, s_end, width, -width, roadmarks);
            add_broken_marks(roadmark_group, roadmark_group_s0, s_end_roadmark_group, s_end, width, +width, roadmarks);
        }
        else
        {
            add_solid_mark(roadmark_group, roadmark_group_s0, s_start_roadmark_group, s_end_roadmark_group, s_end, width, 0.0, roadmarks);
        }
    }

    return roadmarks;
}

} // namespace odr
