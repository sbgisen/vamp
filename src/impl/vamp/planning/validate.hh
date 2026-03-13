#pragma once

#include <cstdint>
#include <type_traits>

#include <vamp/utils.hh>
#include <vamp/vector.hh>
#include <vamp/collision/environment.hh>

namespace vamp::planning
{
    template <std::size_t n, std::size_t... I>
    inline constexpr auto generate_percents(std::index_sequence<I...>) -> std::array<float, n>
    {
        return {(static_cast<void>(I), static_cast<float>(I + 1) / static_cast<float>(n))...};
    }

    template <std::size_t n>
    struct Percents
    {
        inline static constexpr auto percents = generate_percents<n>(std::make_index_sequence<n>());
    };

    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline constexpr auto validate_vector(
        const typename Robot::Configuration &start,
        const typename Robot::Configuration &vector,
        float distance,
        const collision::Environment<FloatVector<rake>> &environment) -> bool
    {
        // TODO: Fix use of reinterpret_cast in pack() so that this can be constexpr
        const auto percents = FloatVector<rake>(Percents<rake>::percents);

        typename Robot::template ConfigurationBlock<rake> block;

        // HACK: broadcast() implicitly assumes that the rake is exactly VectorWidth
        for (auto i = 0U; i < Robot::dimension; ++i)
        {
            block[i] = start.broadcast(i) + (vector.broadcast(i) * percents);
        }

        const std::size_t n = std::max(std::ceil(distance / static_cast<float>(rake) * resolution), 1.F);

        bool valid = (environment.attachments) ? Robot::template fkcc_attach<rake>(environment, block) :
                                                 Robot::template fkcc<rake>(environment, block);
        if (not valid or n == 1)
        {
            return valid;
        }

        const auto backstep = vector / (rake * n);
        for (auto i = 1U; i < n; ++i)
        {
            for (auto j = 0U; j < Robot::dimension; ++j)
            {
                block[j] = block[j] - backstep.broadcast(j);
            }

            bool valid = (environment.attachments) ? Robot::template fkcc_attach<rake>(environment, block) :
                                                     Robot::template fkcc<rake>(environment, block);
            if (not valid)
            {
                return false;
            }
        }

        return true;
    }

    // SFINAE: detect Robot::validate_motion_impl<rake>(start, goal, env) -> bool
    template <typename Robot, std::size_t rake, typename Env, typename = void>
    struct has_validate_motion_impl : std::false_type {};

    template <typename Robot, std::size_t rake, typename Env>
    struct has_validate_motion_impl<Robot, rake, Env,
        std::void_t<decltype(Robot::template validate_motion_impl<rake>(
            std::declval<const typename Robot::Configuration &>(),
            std::declval<const typename Robot::Configuration &>(),
            std::declval<const Env &>()))>>
        : std::true_type {};

    // SFINAE: detect Robot::distance(a, b) -> float
    template <typename Robot, typename = void>
    struct has_robot_distance : std::false_type {};

    template <typename Robot>
    struct has_robot_distance<Robot,
        std::void_t<decltype(Robot::distance(
            std::declval<const typename Robot::Configuration &>(),
            std::declval<const typename Robot::Configuration &>()))>>
        : std::true_type {};

    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline constexpr auto validate_motion(
        const typename Robot::Configuration &start,
        const typename Robot::Configuration &goal,
        const collision::Environment<FloatVector<rake>> &environment) -> bool
    {
        if constexpr (has_validate_motion_impl<
            Robot, rake, collision::Environment<FloatVector<rake>>>::value)
        {
            return Robot::template validate_motion_impl<rake>(start, goal, environment);
        }
        else
        {
            auto vector = goal - start;
            return validate_vector<Robot, rake, resolution>(start, vector, vector.l2_norm(), environment);
        }
    }
}  // namespace vamp::planning
