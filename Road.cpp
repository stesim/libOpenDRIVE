#include "Road.h"
#include "RefLine.h"

#include <array>
#include <cmath>
#include <iterator>
#include <math.h>
#include <utility>

namespace odr
{
double Crossfall::get_crossfall(double s, bool on_left_side) const
{
    const Poly3 poly = this->get_poly(s);

    if (this->s_start_to_poly.size() > 0)
    {
        auto target_poly_iter = this->s_start_to_poly.upper_bound(s);
        if (target_poly_iter != this->s_start_to_poly.begin())
            target_poly_iter--;

        Side side = Side::Both; // applicable side of the road
        if (this->sides.find(target_poly_iter->first) != this->sides.end())
            side = this->sides.at(target_poly_iter->first);

        if (on_left_side && side == Side::Right)
            return 0;
        else if (!on_left_side && side == Side::Left)
            return 0;

        return target_poly_iter->second.get(s);
    }

    return 0;
}

ConstLaneSectionSet Road::get_lanesections() const
{
    ConstLaneSectionSet lanesections;
    for (const auto& s_lansection : this->s_to_lanesection)
        lanesections.insert(s_lansection.second);

    return lanesections;
}

LaneSectionSet Road::get_lanesections()
{
    LaneSectionSet lanesections;
    for (const auto& s_lansection : this->s_to_lanesection)
        lanesections.insert(s_lansection.second);

    return lanesections;
}

std::shared_ptr<const LaneSection> Road::get_lanesection(double s) const
{
    if (this->s_to_lanesection.size() > 0)
    {
        auto target_lane_sec_iter = this->s_to_lanesection.upper_bound(s);
        if (target_lane_sec_iter != this->s_to_lanesection.begin())
            target_lane_sec_iter--;
        return target_lane_sec_iter->second;
    }

    return nullptr;
}

std::shared_ptr<LaneSection> Road::get_lanesection(double s)
{
    std::shared_ptr<LaneSection> lanesection = std::const_pointer_cast<LaneSection>(static_cast<const Road&>(*this).get_lanesection(s));
    return lanesection;
}

Vec3D Road::get_xyz(double s, double t, double h) const
{
    const Mat3D trans_mat = this->get_transformation_matrix(s);
    const Vec3D xyz = MatVecMultiplication(trans_mat, Vec3D{t, h, 1});

    return xyz;
}

Mat3D Road::get_transformation_matrix(double s) const
{
    const Vec3D  s_vec = this->ref_line->get_grad(s);
    const double superelevation = this->superelevation.get(s);

    const Vec3D e_t = normalize(Vec3D{-s_vec[1], s_vec[0], std::tan(superelevation) * std::abs(s_vec[1])});
    const Vec3D e_h = normalize(crossProduct(s_vec, e_t));
    const Vec3D p0 = this->ref_line->get_xyz(s);

    const Mat3D trans_mat{{{e_t[0], e_h[0], p0[0]}, {e_t[1], e_h[1], p0[1]}, {e_t[2], e_h[2], p0[2]}}};

    return trans_mat;
}

} // namespace odr